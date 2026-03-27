#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "../traffic/traffic_window_internal.h"
#include "track.h"
#include "timer.h"
#include "kassert.h"
#include "ui.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static pos_route_eval_t g_pos_try_eval_main;
static route_plan_t g_pos_try_reserve_plan;
static route_plan_t g_pos_try_authority_plan;
static route_plan_t g_pos_midrev_second_leg_plan;

static uint8_t pos_direction_cmd_from_planned_sensor(const train_pos_t *pos,
                                                     track_node *planned_start) {
    track_node *forward_start;
    uint64_t ignored_dt_us = 0;

    if (!pos || !pos->cur_sensor || !planned_start) return 0;

    forward_start = pos->pred.next_sensor;
    if (!forward_start) {
        forward_start = predict_next_sensor((train_pos_t *)pos, pos->cur_sensor,
                                            &ignored_dt_us);
    }

    if (forward_start == planned_start) return TRACK_DIR_FORWARD;
    if (forward_start && forward_start->reverse == planned_start) {
        return TRACK_DIR_BACKWARD;
    }

    if (pos->cur_sensor == planned_start) return TRACK_DIR_FORWARD;
    if (pos->cur_sensor->reverse == planned_start) return TRACK_DIR_BACKWARD;

    return 0;
}

static uint8_t pos_blocker_mask_from_plan_and_switches(int requester_train,
                                                       const route_plan_t *plan) {
    if (!plan) return 0;
    return pos_route_blocker_mask_from_plan(requester_train, plan) |
           pos_route_blocker_mask_from_switches(plan->sw_nums, plan->sw_dirs,
                                                plan->sw_count, requester_train);
}

static int pos_replan_from_current_stop(train_pos_t *pos) {
    track_node *goal = NULL;
    int32_t offset_mm = 0;

    if (!pos) return 0;

    if (pos->midrev.active && pos->midrev.final_target != NULL) {
        goal = pos->midrev.final_target;
        offset_mm = pos->midrev.final_offset;
    } else if (pos->orig_user_target != NULL) {
        goal = pos->orig_user_target;
        offset_mm = pos->orig_target_offset;
    } else if (pos->pending_target != NULL) {
        goal = pos->pending_target;
        offset_mm = pos->pending_offset_mm;
    } else if (pos->target_sensor != NULL) {
        goal = pos->target_sensor;
        offset_mm = pos->target_offset_mm;
    }

    if (!goal) return 0;

    pos_prepare_goto_request(pos, goal, pos->goto_speed, offset_mm);
    return pos_try_direct_goto(pos);
}

static int pos_build_wait_plan(const train_pos_t *pos, route_plan_t *out_plan,
                               int *out_start_cursor) {
    int start_cursor;

    if (!pos || !out_plan || !out_start_cursor) return 0;
    if (pos->route_path_count <= 0) return 0;

    start_cursor = (pos->replan.wait_mode == POS_WAIT_RESUME_ROUTE)
                       ? pos->route_path_cursor
                       : 0;
    if (start_cursor < 0) start_cursor = 0;
    if (start_cursor >= pos->route_path_count) return 0;

    if (!traffic_window_build_prefix_plan(pos->route_path, pos->route_path_count,
                                          start_cursor, pos->route_path_count - 1,
                                          out_plan)) {
        return 0;
    }
    *out_start_cursor = start_cursor;
    return 1;
}

static int pos_try_launch_committed_route(train_pos_t *pos, uint64_t now_us) {
    int reserved_end_cursor = -1;
    int start_cursor = 0;
    int did_initial_reverse;
    int switch_blocker_owner = -1;
    track_node *cur_sensor_orig;
    uint8_t planned_dir_cmd = 0;
    pos_wait_mode_t wait_mode;

    if (!pos || !pos->cur_sensor) return 0;
    wait_mode = (pos_wait_mode_t)pos->replan.wait_mode;
    if (wait_mode != POS_WAIT_PRELAUNCH_ROUTE &&
        wait_mode != POS_WAIT_RESUME_ROUTE) {
        return 0;
    }
    if (!pos_build_wait_plan(pos, &g_pos_try_reserve_plan, &start_cursor)) {
        pos_enter_wait_resource(pos, now_us, 0, wait_mode);
        return 1;
    }

    if (!pos_route_authority_prepare_launch(pos, &g_pos_try_reserve_plan,
                                            &g_pos_try_authority_plan,
                                            &reserved_end_cursor,
                                            &switch_blocker_owner)) {
        if (wait_mode == POS_WAIT_RESUME_ROUTE &&
            switch_blocker_owner == pos->train_num) {
            return pos_replan_from_current_stop(pos);
        }
        uint8_t blocker_mask =
            pos_blocker_mask_from_plan_and_switches(pos->train_num,
                                                    &g_pos_try_reserve_plan);
        pos_enter_wait_resource(pos, now_us, blocker_mask, wait_mode);
        return 1;
    }

    if (!pos_apply_route_switches_safe(g_pos_try_authority_plan.sw_nums,
                                       g_pos_try_authority_plan.sw_dirs,
                                       g_pos_try_authority_plan.sw_count,
                                       pos->train_num)) {
        uint8_t blocker_mask =
            pos_route_blocker_mask_from_switches(g_pos_try_authority_plan.sw_nums,
                                                 g_pos_try_authority_plan.sw_dirs,
                                                 g_pos_try_authority_plan.sw_count,
                                                 pos->train_num);
        pos_enter_wait_resource(pos, now_us, blocker_mask, wait_mode);
        return 1;
    }

    if (!traffic_reserve_plan(pos->train_num,
                              pos->replan.launch_origin ? pos->replan.launch_origin
                                                        : pos->cur_sensor,
                              &g_pos_try_authority_plan)) {
        uint8_t blocker_mask = pos_route_blocker_mask_from_plan(
            pos->train_num, &g_pos_try_authority_plan);
        pos_enter_wait_resource(pos, now_us, blocker_mask, wait_mode);
        return 1;
    }
    if (g_pos_try_authority_plan.sw_count > 0) ui_mark_switches_dirty();

    cur_sensor_orig = pos->cur_sensor;
    did_initial_reverse =
        wait_mode == POS_WAIT_PRELAUNCH_ROUTE && pos->replan.need_initial_reverse;
    if (did_initial_reverse) {
        planned_dir_cmd = pos_direction_cmd_from_planned_sensor(
            pos, pos->replan.launch_origin);
        KASSERT(planned_dir_cmd == TRACK_DIR_FORWARD ||
                planned_dir_cmd == TRACK_DIR_BACKWARD);
        track_send_direction(pos->train_num, planned_dir_cmd);
        pos->cur_sensor = cur_sensor_orig->reverse;
        pos->going_forward = !pos->going_forward;
        pos->pred.next_sensor = cur_sensor_orig->reverse;
        pos->pred.alt_sensor = NULL;
        pos->pred.branch_node = NULL;
        pos->pred.trigger_time = 0;
        pos->pred.skipped_sensor_count = 0;
        pos->dead_track_deadline_us = 0;
    }
    pos->offroute_valid = 0;
    pos->offroute_expected_sensor = NULL;

    pos->route_reserved_end_cursor = start_cursor + reserved_end_cursor;
    pos_route_authority_sync_target(pos);

    pos->pending_target = NULL;
    pos->pending_offset_mm = 0;
    pos->replan.next_us = 0;
    pos->replan.seen_generation = traffic_get_change_generation();
    pos->replan.blocker_mask = 0;
    pos->replan.wait_mode = POS_WAIT_NONE;
    pos->replan.need_initial_reverse = 0;
    pos->replan.launch_origin = NULL;
    pos->authority_seen_generation = traffic_get_change_generation();
    pos->authority_next_us = 0;

    pos_arm_switch_settle(pos, g_pos_try_authority_plan.sw_count,
                          did_initial_reverse ? POS_SWITCH_SETTLE_REVERSED
                                              : POS_SWITCH_SETTLE_NORMAL,
                          now_us);
    ui_mark_position_dirty();
    return 1;
}

static int route_switch_needs_change(int sw_num, char desired_dir) {
    int sw_idx = track_switch_to_index(sw_num);
    if (sw_idx < 0) return 1;
    char current_dir = track_get_switch_state()[sw_idx].state;
    return current_dir != desired_dir;
}

int pos_route_switch_blocker(const int *sw_nums, const char *sw_dirs,
                             int sw_count, int requester_train) {
    for (int i = sw_count - 1; i >= 0; i--) {
        if (!route_switch_needs_change(sw_nums[i], sw_dirs[i])) continue;
        int owner = traffic_can_set_switch(sw_nums[i], requester_train);
        if (owner >= 0) return owner;
    }
    return -1;
}

int pos_apply_route_switches_safe(const int *sw_nums, const char *sw_dirs,
                                  int sw_count, int requester_train) {
    if (pos_route_switch_blocker(sw_nums, sw_dirs, sw_count, requester_train) >= 0) {
        return 0;
    }
    for (int i = sw_count - 1; i >= 0; i--) {
        if (!route_switch_needs_change(sw_nums[i], sw_dirs[i])) continue;
        track_set_switch(sw_nums[i], sw_dirs[i]);
        track_update_switch(sw_nums[i], sw_dirs[i]);
    }
    return 1;
}

static bool pos_is_waiting_resource(train_pos_t *pos) {
    return !pos || pos->route_state == TRAIN_STATE_WAIT_RESOURCE;
}

void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us, uint8_t blocker_mask,
                             pos_wait_mode_t wait_mode) {
    if (!pos) return;
    pos->replan.blocker_mask = blocker_mask;
    pos->replan.wait_mode = wait_mode;
    if (pos_is_waiting_resource(pos)) {
        ui_mark_position_dirty();
        return;
    }
    track_set_speed(pos->train_num, 0);
    if (wait_mode == POS_WAIT_RESUME_ROUTE ||
        wait_mode == POS_WAIT_MIDREV_SECOND_LEG) {
        pos_refresh_stop_reservation(pos);
    } else {
        traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                        TRAIN_BODY_MM,
                                        pos_release_keep_end(pos->cur_sensor,
                                                             pos->pred.next_sensor));
    }
    pos->route_state = TRAIN_STATE_WAIT_RESOURCE;
    pos->replan.retry_count = 0;
    pos->replan.next_us = now_us + REPLAN_INTERVAL_US;
    pos->replan.seen_generation = traffic_get_change_generation();
    pos_route_authority_reset(pos);
    pos->stopping_since_us = now_us;
    pos->effective_v = 0;
    pos_clear_prediction(pos);
    ui_mark_position_dirty();
}

static int pos_try_direct_goto_impl(train_pos_t *pos,
                                    int wait_on_unreachable) {
    track_node *user_target = pos->pending_target;
    int32_t     offset_mm   = pos->pending_offset_mm;
    route_plan_t *rp;
    track_node *chosen_origin;
    int need_initial_reverse;
    pos_route_eval_result_t eval_result;

    if (!pos->cur_sensor || !user_target) return 0;

    eval_result = pos_evaluate_target_plan(pos, user_target, &g_pos_try_eval_main);
    switch (eval_result) {
    case POS_ROUTE_EVAL_UNREACHABLE:
        if (!wait_on_unreachable) return 0;
        /* Treat planner misses like a transient wait so STOPPED callers and
         * replan loops keep retrying instead of tripping KASSERTs. */
        pos_enter_wait_resource(pos, read_timer(), 0, POS_WAIT_NONE);
        return 1;
    case POS_ROUTE_EVAL_READY:
        break;
    default:
        return 0;
    }

    rp = &g_pos_try_eval_main.plan;
    chosen_origin = g_pos_try_eval_main.chosen_origin;
    need_initial_reverse = g_pos_try_eval_main.need_initial_reverse;

    uint64_t now_us = read_timer();
    pos_commit_route_plan(pos, rp, chosen_origin, need_initial_reverse, offset_mm);
    pos->replan.wait_mode = POS_WAIT_PRELAUNCH_ROUTE;
    KASSERT(pos_try_launch_committed_route(pos, now_us));
    return 1;
}

int pos_try_direct_goto(train_pos_t *pos) {
    return pos_try_direct_goto_impl(pos, 1);
}

int pos_try_direct_goto_strict(train_pos_t *pos) {
    return pos_try_direct_goto_impl(pos, 0);
}

int pos_try_resume_committed_route(train_pos_t *pos, uint64_t now_us) {
    return pos_try_launch_committed_route(pos, now_us);
}

int pos_try_resume_wait_resource(train_pos_t *pos, uint64_t now_us) {
    int ok;

    if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    switch ((pos_wait_mode_t)pos->replan.wait_mode) {
    case POS_WAIT_PRELAUNCH_ROUTE:
    case POS_WAIT_RESUME_ROUTE:
        ok = pos_try_resume_committed_route(pos, now_us);
        KASSERT(ok);
        return 1;
    case POS_WAIT_MIDREV_SECOND_LEG:
        (void)pos_handle_midrev_resume(pos, now_us);
        return 1;
    case POS_WAIT_NONE:
    default:
        pos_restore_pending_target(pos);
        if (pos->pending_target == NULL) return 1;
        ok = pos_try_direct_goto(pos);
        KASSERT(ok);
        return 1;
    }
}

static void pos_enter_midrev_second_leg_wait(train_pos_t *pos, uint64_t now_us,
                                             uint8_t blocker_mask) {
    pos_enter_wait_resource(pos, now_us, blocker_mask, POS_WAIT_MIDREV_SECOND_LEG);
}

static int pos_build_midrev_second_leg_plan(const train_pos_t *pos,
                                            route_plan_t *out_plan) {
    if (!pos || !out_plan || pos->midrev.path2_count <= 0) return 0;

    *out_plan = (route_plan_t){0};
    out_plan->path_count = pos->midrev.path2_count;
    for (int i = 0; i < pos->midrev.path2_count; i++) {
        out_plan->path_nodes[i] = pos->midrev.path2[i];
    }
    return 1;
}

int pos_handle_midrev_resume(train_pos_t *pos, uint64_t now_us) {
    route_plan_t *second_leg_plan = &g_pos_midrev_second_leg_plan;
    track_node *final_target;
    track_node *planned_start;
    int32_t final_offset;
    int sw_count;
    int sw_owner;
    uint8_t planned_dir_cmd;

    if (!pos || !pos->midrev.active || !pos->midrev.final_target) return 0;

    final_target = pos->midrev.final_target;
    final_offset = pos->midrev.final_offset;
    sw_count = pos->midrev.sw_count;
    if (!pos_build_midrev_second_leg_plan(pos, second_leg_plan)) return 0;

    if (!traffic_can_reserve_plan(pos->train_num, second_leg_plan)) {
        uint8_t blocker_mask =
            pos_route_blocker_mask_from_plan(pos->train_num, second_leg_plan);
        pos_enter_midrev_second_leg_wait(pos, now_us, blocker_mask);
        return 0;
    }

    sw_owner = pos_route_switch_blocker(pos->midrev.sw_nums, pos->midrev.sw_dirs,
                                        pos->midrev.sw_count, pos->train_num);
    if (sw_owner == pos->train_num) {
        return pos_replan_from_current_stop(pos);
    }
    if (sw_owner >= 0) {
        uint8_t blocker_mask =
            pos_route_blocker_mask_from_switches(pos->midrev.sw_nums,
                                                 pos->midrev.sw_dirs,
                                                 pos->midrev.sw_count,
                                                 pos->train_num);
        pos_enter_midrev_second_leg_wait(pos, now_us, blocker_mask);
        return 0;
    }

    if (!pos_apply_route_switches_safe(pos->midrev.sw_nums, pos->midrev.sw_dirs,
                                       pos->midrev.sw_count, pos->train_num)) {
        uint8_t blocker_mask =
            pos_route_blocker_mask_from_switches(pos->midrev.sw_nums,
                                                 pos->midrev.sw_dirs,
                                                 pos->midrev.sw_count,
                                                 pos->train_num);
        pos_enter_midrev_second_leg_wait(pos, now_us, blocker_mask);
        return 0;
    }

    if (!traffic_reserve_plan(pos->train_num, pos->cur_sensor, second_leg_plan)) {
        uint8_t blocker_mask =
            pos_route_blocker_mask_from_plan(pos->train_num, second_leg_plan);
        pos_enter_midrev_second_leg_wait(pos, now_us, blocker_mask);
        return 0;
    }
    if (sw_count > 0) ui_mark_switches_dirty();

    pos->midrev.active = 0;

    KASSERT(second_leg_plan->path_count > 0);
    planned_start = &g_track[second_leg_plan->path_nodes[0]];
    planned_dir_cmd = pos_direction_cmd_from_planned_sensor(pos, planned_start);
    KASSERT(planned_dir_cmd == TRACK_DIR_FORWARD ||
            planned_dir_cmd == TRACK_DIR_BACKWARD);
    track_send_direction(pos->train_num, planned_dir_cmd);
    pos->prev_going_forward = pos->going_forward;
    pos->going_forward = !pos->going_forward;

    if (pos->cur_sensor && pos->cur_sensor->reverse) {
        pos->cur_sensor = pos->cur_sensor->reverse;
    }

    pos->target_sensor = final_target;
    pos->target_offset_mm = final_offset;

    pos->route_path_count = pos->midrev.path2_count;
    for (int i = 0; i < pos->midrev.path2_count; i++) {
        pos->route_path[i] = pos->midrev.path2[i];
    }
    pos->route_path_cursor = 0;
    pos_route_authority_reset(pos);
    pos->route_reserved_end_cursor = pos->route_path_count - 1;
    pos_route_authority_sync_target(pos);
    pos->replan.next_us = 0;
    pos->replan.seen_generation = traffic_get_change_generation();
    pos->replan.blocker_mask = 0;
    pos->replan.wait_mode = POS_WAIT_NONE;
    pos->replan.need_initial_reverse = 0;
    pos->replan.launch_origin = NULL;
    pos->authority_seen_generation = traffic_get_change_generation();
    pos->authority_next_us = 0;

    pos_arm_switch_settle(pos, sw_count, POS_SWITCH_SETTLE_REVERSED, now_us);
    return 1;
}
