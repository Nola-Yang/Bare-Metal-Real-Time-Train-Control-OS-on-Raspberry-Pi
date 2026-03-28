#include "train_tracking/position_priv.h"
#include "train_tracking/planner_core.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

static planner_workspace_t g_pos_authority_ws;
static route_plan_t g_pos_authority_top_up_plan;
static int g_pos_authority_owners[TRACK_MAX];
static char g_pos_authority_switch_state[MAX_SWITCHES];

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

static void authority_fill_view(const train_pos_t *pos, planner_train_view_t *out) {
    if (!out) return;
    *out = (planner_train_view_t){0};
    if (!pos) return;
    out->train_ind = pos->train_ind;
    out->goto_speed = pos->goto_speed;
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
                                       int requester_train,
                                       const uint16_t *path,
                                       int path_count,
                                       int start_cursor,
                                       int allow_short_goal,
                                       int min_end_cursor,
                                       route_plan_t *out_prefix,
                                       int *out_end_cursor,
                                       int *out_switch_blocker_owner) {
    planner_env_t env;
    planner_train_view_t view;
    route_plan_t full_plan = {0};
    int effective_start_cursor;

    if (!pos || !path || !out_prefix || !out_end_cursor) return 0;
    if (path_count <= 0 || start_cursor < 0 || start_cursor >= path_count) return 0;

    pos_planner_build_env(&env, g_pos_authority_owners, g_pos_authority_switch_state);
    pos_planner_fill_view(pos, &view);
    view.train_num = requester_train;

    full_plan.path_count = path_count;
    for (int i = 0; i < path_count; i++) {
        full_plan.path_nodes[i] = path[i];
    }

    effective_start_cursor =
        authority_effective_start_cursor(pos, path, path_count, start_cursor);
    return planner_prepare_launch_prefix(
        &env, &view, &full_plan, start_cursor, effective_start_cursor,
        allow_short_goal, min_end_cursor, &g_pos_authority_ws, out_prefix,
        out_end_cursor, out_switch_blocker_owner);
}

int32_t pos_route_authority_stop_dist_mm(const train_pos_t *pos) {
    planner_train_view_t view;

    authority_fill_view(pos, &view);
    return planner_view_stop_dist_mm(&view);
}

int32_t pos_route_authority_min_mm(const train_pos_t *pos) {
    planner_train_view_t view;

    authority_fill_view(pos, &view);
    return planner_view_min_window_mm(&view);
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
    return authority_build_best_prefix(pos, pos->train_num, full_plan->path_nodes,
                                       full_plan->path_count, 0,
                                       allow_short_goal, -1, out_prefix,
                                       out_reserved_end_cursor,
                                       out_switch_blocker_owner);
}

int pos_route_authority_prepare_launch(train_pos_t *pos,
                                       const route_plan_t *full_plan,
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
                                    1, pos->route_reserved_end_cursor,
                                    &g_pos_authority_top_up_plan, &new_end_cursor,
                                    NULL) &&
        new_end_cursor > pos->route_reserved_end_cursor) {
        if (pos_apply_route_switches_safe(g_pos_authority_top_up_plan.sw_nums,
                                          g_pos_authority_top_up_plan.sw_dirs,
                                          g_pos_authority_top_up_plan.sw_count,
                                          pos->train_num) &&
            traffic_reserve_plan(pos->train_num, pos->cur_sensor,
                                 &g_pos_authority_top_up_plan)) {
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
