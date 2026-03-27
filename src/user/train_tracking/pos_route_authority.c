#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "train_tracking/speed_table.h"
#include "traffic_window_internal.h"
#include "track.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

static route_plan_t g_authority_candidate_prefix;
static route_plan_t g_authority_best_prefix;

static int authority_is_canonical_sensor(int idx) {
    track_node *n;
    track_node *r;

    if (idx < 0 || idx >= TRACK_MAX) return 0;
    n = &g_track[idx];
    if (n->type != NODE_SENSOR) return 0;
    r = n->reverse;
    if (!r || r->type != NODE_SENSOR) return 1;
    return n < r;
}

static int32_t authority_path_dist(const uint16_t *path, int start_cursor,
                                   int end_cursor) {
    if (!path) return -1;
    if (end_cursor < start_cursor) return 0;
    return route_path_dist_from(path, start_cursor, end_cursor + 1);
}

static int32_t authority_early_stop_mm(const train_pos_t *pos) {
    int32_t tv;

    if (!pos) return 0;
    tv = speed_table_get_v(pos->train_ind, pos->goto_speed);
    if (tv <= 0) return 0;
    return (int32_t)((int64_t)tv * (int64_t)speed_table_get_early_stop(pos->train_ind, pos->goto_speed) / 1000000LL);
}

static int32_t authority_brake_dist_mm(const train_pos_t *pos) {
    int32_t tv;
    int32_t ta;

    if (!pos) return 0;
    tv = speed_table_get_v(pos->train_ind, pos->goto_speed);
    ta = speed_table_get_nominal_decel(pos->train_ind, pos->goto_speed);
    if (tv <= 0 || ta <= 0) return 0;
    return tv * tv / (2 * ta);
}

static void authority_sync_target_internal(train_pos_t *pos) {
    int end_cursor;
    int32_t dist;

    if (!pos || pos->route_path_count <= 0) return;
    if (pos->route_path_cursor < 0) pos->route_path_cursor = 0;
    if (pos->route_path_cursor >= pos->route_path_count) {
        pos->route_path_cursor = pos->route_path_count - 1;
    }

    end_cursor = pos->route_reserved_end_cursor;
    if (end_cursor < pos->route_path_cursor) end_cursor = pos->route_path_cursor;
    if (end_cursor >= pos->route_path_count) end_cursor = pos->route_path_count - 1;
    pos->route_reserved_end_cursor = end_cursor;

    pos->target_sensor = &g_track[pos->route_path[end_cursor]];
    pos->target_offset_mm = (end_cursor >= pos->route_path_count - 1 && !pos->midrev.active)
                                ? pos->orig_target_offset
                                : 0;

    dist = authority_path_dist(pos->route_path, pos->route_path_cursor, end_cursor);
    pos->dist_to_target_mm = (dist >= 0) ? dist + pos->target_offset_mm : 0;
    if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
}

static int authority_build_best_prefix(int requester_train, const uint16_t *path,
                                       int path_count, int start_cursor,
                                       int32_t min_window_mm,
                                       int32_t max_window_mm,
                                       route_plan_t *out_prefix,
                                       int *out_end_cursor) {
    int found = 0;

    if (!path || !out_prefix || !out_end_cursor) return 0;
    if (path_count <= 0 || start_cursor < 0 || start_cursor >= path_count) return 0;

    for (int end_cursor = start_cursor; end_cursor < path_count; end_cursor++) {
        int32_t dist_mm;
        if (!traffic_window_build_prefix_plan(path, path_count, start_cursor,
                                              end_cursor, &g_authority_candidate_prefix)) {
            break;
        }

        dist_mm = g_authority_candidate_prefix.total_dist_mm;
        if (dist_mm <= 0) continue;
        if (dist_mm > max_window_mm) break;

        if (end_cursor != path_count - 1 &&
            !authority_is_canonical_sensor((int)path[end_cursor])) {
            continue;
        }

        if (pos_route_switch_blocker(g_authority_candidate_prefix.sw_nums,
                                     g_authority_candidate_prefix.sw_dirs,
                                     g_authority_candidate_prefix.sw_count,
                                     requester_train) >= 0) {
            break;
        }
        if (!traffic_can_reserve_plan(requester_train,
                                      &g_authority_candidate_prefix)) break;

        if (dist_mm < min_window_mm) continue;

        g_authority_best_prefix = g_authority_candidate_prefix;
        *out_end_cursor = end_cursor;
        found = 1;
    }

    if (!found) return 0;
    *out_prefix = g_authority_best_prefix;
    return 1;
}

int32_t pos_route_authority_stop_dist_mm(const train_pos_t *pos) {
    int32_t d_brake;
    int32_t d_early;

    d_brake = authority_brake_dist_mm(pos);
    d_early = authority_early_stop_mm(pos);
    if (d_brake < 0) d_brake = 0;
    if (d_early < 0) d_early = 0;
    return d_brake + d_early;
}

int32_t pos_route_authority_min_mm(const train_pos_t *pos) {
    int32_t d_brake = authority_brake_dist_mm(pos);
    int32_t d_early = authority_early_stop_mm(pos);

    if (d_brake < 0) d_brake = 0;
    if (d_early < 0) d_early = 0;
    return GOTO_MIN_DIST_FACTOR * d_brake + d_early;
}

int32_t pos_route_authority_target_mm(const train_pos_t *pos) {
    return pos_route_authority_min_mm(pos);
}

int32_t pos_route_authority_max_mm(const train_pos_t *pos) {
    return pos_route_authority_min_mm(pos) + pos_route_authority_stop_dist_mm(pos);
}

int32_t pos_route_authority_extend_trigger_mm(const train_pos_t *pos) {
    return pos_route_authority_stop_dist_mm(pos);
}

int32_t pos_route_authority_remaining_mm(const train_pos_t *pos) {
    if (!pos || pos->route_path_count <= 0) return 0;
    if (pos->route_reserved_end_cursor < pos->route_path_cursor) return 0;
    return authority_path_dist(pos->route_path, pos->route_path_cursor,
                               pos->route_reserved_end_cursor);
}

int pos_route_authority_is_leg_goal_stop(const train_pos_t *pos) {
    if (!pos || pos->route_path_count <= 0) return 1;
    return pos->route_reserved_end_cursor >= pos->route_path_count - 1;
}

void pos_route_authority_reset(train_pos_t *pos) {
    if (!pos) return;
    pos->route_reserved_end_cursor = 0;
    pos->authority_seen_generation = traffic_get_change_generation();
    pos->authority_next_us = 0;
}

void pos_route_authority_sync_target(train_pos_t *pos) {
    authority_sync_target_internal(pos);
}

int pos_route_authority_prepare_launch(train_pos_t *pos, const route_plan_t *full_plan,
                                       route_plan_t *out_prefix,
                                       int *out_reserved_end_cursor) {
    int32_t min_window_mm;
    int32_t max_window_mm;

    if (!pos || !full_plan || !out_prefix || !out_reserved_end_cursor) return 0;
    min_window_mm = pos_route_authority_target_mm(pos);
    max_window_mm = pos_route_authority_max_mm(pos);

    return authority_build_best_prefix(pos->train_num, full_plan->path_nodes,
                                       full_plan->path_count, 0,
                                       min_window_mm, max_window_mm,
                                       out_prefix, out_reserved_end_cursor);
}

int pos_route_authority_try_top_up(train_pos_t *pos, uint64_t now_us, int force) {
    int new_end_cursor = -1;
    uint32_t generation;
    int extended = 0;

    if (!pos || pos->route_state != TRAIN_STATE_ON_ROUTE) return 0;
    if (pos->route_path_count <= 0) return 0;
    if (pos->route_path_cursor < 0 || pos->route_path_cursor >= pos->route_path_count) return 0;

    generation = traffic_get_change_generation();
    if (!force &&
        generation == pos->authority_seen_generation &&
        pos->authority_next_us > 0 && now_us < pos->authority_next_us) {
        return 0;
    }

    if (authority_build_best_prefix(pos->train_num, pos->route_path,
                                    pos->route_path_count, pos->route_path_cursor,
                                    pos_route_authority_target_mm(pos),
                                    pos_route_authority_max_mm(pos),
                                    &g_authority_candidate_prefix, &new_end_cursor) &&
        new_end_cursor > pos->route_reserved_end_cursor) {
        if (pos_apply_route_switches_safe(g_authority_candidate_prefix.sw_nums,
                                          g_authority_candidate_prefix.sw_dirs,
                                          g_authority_candidate_prefix.sw_count,
                                          pos->train_num) &&
            traffic_reserve_plan(pos->train_num, pos->cur_sensor,
                                 &g_authority_candidate_prefix)) {
            pos->route_reserved_end_cursor = new_end_cursor;
            authority_sync_target_internal(pos);
            extended = 1;
            ui_mark_switches_dirty();
            ui_mark_position_dirty();
        }
    }

    pos->authority_seen_generation = traffic_get_change_generation();
    pos->authority_next_us = now_us + REPLAN_INTERVAL_US;
    return extended;
}
