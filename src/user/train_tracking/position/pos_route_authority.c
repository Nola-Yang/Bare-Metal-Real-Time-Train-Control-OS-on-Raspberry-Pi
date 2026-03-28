#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "train_tracking/speed_table.h"
#include "../traffic/traffic_window_internal.h"
#include "track.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

static route_plan_t g_authority_candidate_prefix;
static route_plan_t g_authority_short_goal_prefix;

static int32_t authority_path_dist(const uint16_t *path, int start_cursor,
                                   int end_cursor) {
    if (!path) return -1;
    if (end_cursor < start_cursor) return 0;
    return route_path_dist_from(path, start_cursor, end_cursor + 1);
}

static int authority_effective_start_cursor(const train_pos_t *pos,
                                            const uint16_t *path,
                                            int path_count,
                                            int start_cursor) {
    int cur_idx;

    if (!pos || !path || path_count <= 1) return start_cursor;
    if (start_cursor != 0) return start_cursor;
    if (!pos->awaiting_post_launch_sensor || !pos->cur_sensor) return start_cursor;
    if (pos->pred.next_sensor != pos->cur_sensor) return start_cursor;

    cur_idx = (int)(pos->cur_sensor - g_track);
    if (cur_idx < 0 || cur_idx >= TRACK_MAX) return start_cursor;
    if ((int)path[1] != cur_idx) return start_cursor;
    return 1;
}

static int32_t authority_effective_path_dist(const train_pos_t *pos,
                                             const uint16_t *path,
                                             int path_count,
                                             int start_cursor,
                                             int end_cursor) {
    int effective_start_cursor;

    if (!pos || !path || path_count <= 0) return -1;
    effective_start_cursor =
        authority_effective_start_cursor(pos, path, path_count, start_cursor);
    return authority_path_dist(path, effective_start_cursor, end_cursor);
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

    dist = authority_effective_path_dist(pos, pos->route_path, pos->route_path_count,
                                         pos->route_path_cursor, end_cursor);
    pos->dist_to_target_mm = (dist >= 0) ? dist + pos->target_offset_mm : 0;
    if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
}

static int authority_build_best_prefix(const train_pos_t *pos,
                                       int requester_train, const uint16_t *path,
                                       int path_count, int start_cursor,
                                       int32_t min_window_mm,
                                       int32_t stop_dist_mm,
                                       int allow_short_goal,
                                       int min_end_cursor,
                                       route_plan_t *out_prefix,
                                       int *out_end_cursor,
                                       int *out_switch_blocker_owner) {
    int have_short_goal = 0;
    int short_goal_end_cursor = -1;

    if (!path || !out_prefix || !out_end_cursor) return 0;
    if (path_count <= 0 || start_cursor < 0 || start_cursor >= path_count) return 0;
    if (out_switch_blocker_owner) *out_switch_blocker_owner = -1;

    for (int end_cursor = start_cursor; end_cursor < path_count; end_cursor++) {
        int32_t dist_mm;
        int switch_blocker;
        if (!traffic_window_build_prefix_plan(path, path_count, start_cursor,
                                              end_cursor, &g_authority_candidate_prefix)) {
            break;
        }

        dist_mm = authority_effective_path_dist(pos, path, path_count,
                                                start_cursor, end_cursor);
        if (dist_mm < 0) break;
        if (dist_mm <= 0) continue;

        if (end_cursor != path_count - 1 &&
            g_track[path[end_cursor]].type != NODE_SENSOR) {
            continue;
        }

        switch_blocker = pos_route_switch_blocker(g_authority_candidate_prefix.sw_nums,
                                                  g_authority_candidate_prefix.sw_dirs,
                                                  g_authority_candidate_prefix.sw_count,
                                                  requester_train);
        if (switch_blocker >= 0) {
            if (out_switch_blocker_owner) *out_switch_blocker_owner = switch_blocker;
            break;
        }
        if (!traffic_can_reserve_plan(requester_train,
                                      &g_authority_candidate_prefix)) break;

        if (allow_short_goal &&
            end_cursor == path_count - 1 &&
            dist_mm >= stop_dist_mm &&
            end_cursor > min_end_cursor) {
            g_authority_short_goal_prefix = g_authority_candidate_prefix;
            short_goal_end_cursor = end_cursor;
            have_short_goal = 1;
        }

        if (dist_mm < min_window_mm) continue;
        if (end_cursor <= min_end_cursor) continue;

        *out_prefix = g_authority_candidate_prefix;
        *out_end_cursor = end_cursor;
        return 1;
    }

    if (have_short_goal) {
        *out_prefix = g_authority_short_goal_prefix;
        *out_end_cursor = short_goal_end_cursor;
        return 1;
    }

    return 0;
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

int32_t pos_route_authority_extend_trigger_mm(const train_pos_t *pos) {
    return pos_route_authority_stop_dist_mm(pos);
}

int32_t pos_route_authority_remaining_mm(const train_pos_t *pos) {
    if (!pos || pos->route_path_count <= 0) return 0;
    if (pos->route_reserved_end_cursor < pos->route_path_cursor) return 0;
    return authority_effective_path_dist(pos, pos->route_path, pos->route_path_count,
                                         pos->route_path_cursor,
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

static int authority_prepare_launch_internal(train_pos_t *pos,
                                             const route_plan_t *full_plan,
                                             route_plan_t *out_prefix,
                                             int *out_reserved_end_cursor,
                                             int *out_switch_blocker_owner,
                                             int allow_short_goal) {
    int32_t min_window_mm;
    int32_t stop_dist_mm;

    if (!pos || !full_plan || !out_prefix || !out_reserved_end_cursor) return 0;
    min_window_mm = pos_route_authority_target_mm(pos);
    stop_dist_mm = pos_route_authority_stop_dist_mm(pos);

    return authority_build_best_prefix(pos, pos->train_num, full_plan->path_nodes,
                                       full_plan->path_count, 0,
                                       min_window_mm,
                                       stop_dist_mm,
                                       allow_short_goal,
                                       -1,
                                       out_prefix, out_reserved_end_cursor,
                                       out_switch_blocker_owner);
}

int pos_route_authority_prepare_launch(train_pos_t *pos, const route_plan_t *full_plan,
                                       route_plan_t *out_prefix,
                                       int *out_reserved_end_cursor,
                                       int *out_switch_blocker_owner) {
    return authority_prepare_launch_internal(pos, full_plan, out_prefix,
                                             out_reserved_end_cursor,
                                             out_switch_blocker_owner, 1);
}

int pos_route_authority_prepare_launch_strict(train_pos_t *pos,
                                              const route_plan_t *full_plan,
                                              route_plan_t *out_prefix,
                                              int *out_reserved_end_cursor,
                                              int *out_switch_blocker_owner) {
    return authority_prepare_launch_internal(pos, full_plan, out_prefix,
                                             out_reserved_end_cursor,
                                             out_switch_blocker_owner, 0);
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

    if (authority_build_best_prefix(pos, pos->train_num, pos->route_path,
                                    pos->route_path_count, pos->route_path_cursor,
                                    pos_route_authority_target_mm(pos),
                                    pos_route_authority_stop_dist_mm(pos),
                                    1,
                                    pos->route_reserved_end_cursor,
                                    &g_authority_candidate_prefix, &new_end_cursor,
                                    NULL) &&
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
