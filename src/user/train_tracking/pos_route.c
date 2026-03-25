#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "timer.h"
#include "ui.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static pos_route_eval_t g_pos_try_eval_main;

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

void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us, uint8_t blocker_mask) {
    if (!pos) return;
    pos->replan.blocker_mask = blocker_mask;
    if (pos_is_waiting_resource(pos)) {
        ui_mark_position_dirty();
        return;
    }
    track_set_speed(pos->train_num, 0);
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM,
                                    pos_release_keep_end(pos->cur_sensor,
                                                         pos->pred.next_sensor));
    pos->route_state = TRAIN_STATE_WAIT_RESOURCE;
    pos->replan.retry_count = 0;
    pos->replan.next_us = now_us + REPLAN_INTERVAL_US;
    pos->replan.seen_generation = traffic_get_change_generation();
    pos->stopping_since_us = now_us;
    pos->effective_v = 0;
    pos_clear_prediction(pos);
    ui_mark_position_dirty();
}

static int pos_try_direct_goto_impl(train_pos_t *pos,
                                    int wait_on_unreachable) {
    track_node *user_target = pos->pending_target;
    int32_t     offset_mm   = pos->pending_offset_mm;
    track_node *cur_sensor_orig;
    route_plan_t *rp;
    route_plan_t reserve_plan;
    track_node *chosen_origin;
    int need_initial_reverse;

    if (!pos->cur_sensor || !user_target) return 0;

    switch (pos_evaluate_target_ready_now(pos, user_target, &g_pos_try_eval_main)) {
    case POS_ROUTE_EVAL_UNREACHABLE:
        if (!wait_on_unreachable) return 0;
        /* Treat planner misses like a transient wait so STOPPED callers and
         * replan loops keep retrying instead of tripping KASSERTs. */
        pos_enter_wait_resource(pos, read_timer(), 0);
        return 1;
    case POS_ROUTE_EVAL_BLOCKED:
        pos_enter_wait_resource(pos, read_timer(), g_pos_try_eval_main.blocker_mask);
        return 1;
    case POS_ROUTE_EVAL_READY:
        break;
    default:
        return 0;
    }

    cur_sensor_orig = pos->cur_sensor;
    rp = &g_pos_try_eval_main.plan;
    chosen_origin = g_pos_try_eval_main.chosen_origin;
    need_initial_reverse = g_pos_try_eval_main.need_initial_reverse;

    uint64_t now_us = read_timer();
    track_node *eff_start = chosen_origin;
    track_node *chosen_target = rp->has_reversal ? rp->reversal_sensor
                                                  : rp->chosen_target;
    reserve_plan = *rp;
    if (reserve_plan.has_reversal) {
        /* Reserve only the first leg up to the reversal point.
         * The second leg is reserved when the train actually reaches the midpoint. */
        reserve_plan.path_count2 = 0;
    }
    /* Keep the parked sensor window until the next real hit.
     * The new route is added on top of the existing stopped reservation. */
    if (!traffic_can_reserve_plan(pos->train_num, &reserve_plan)) {
        uint8_t blocker_mask = pos_route_blocker_mask_from_plan(pos->train_num,
                                                                &reserve_plan);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 1;
    }
    if (!pos_apply_route_switches_safe(rp->sw_nums, rp->sw_dirs, rp->sw_count,
                                       pos->train_num)) {
        uint8_t blocker_mask = pos_route_blocker_mask_from_switches(rp->sw_nums,
                                                                    rp->sw_dirs,
                                                                    rp->sw_count,
                                                                    pos->train_num);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 1;
    }
    if (!traffic_reserve_plan(pos->train_num, eff_start, &reserve_plan)) {
        uint8_t blocker_mask = pos_route_blocker_mask_from_plan(pos->train_num,
                                                                &reserve_plan);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 1;
    }
    if (rp->sw_count > 0) ui_mark_switches_dirty();

    if (need_initial_reverse) {
        track_reverse(pos->train_num);
        pos->cur_sensor    = cur_sensor_orig->reverse;
        pos->going_forward = !pos->going_forward;
        pos->pred.next_sensor  = cur_sensor_orig->reverse;
        pos->pred.alt_sensor   = NULL;
        pos->pred.branch_node  = NULL;
        pos->pred.trigger_time = 0;
        pos->pred.skipped_sensor_count = 0;
        pos->dead_track_deadline_us = 0;
    }
    pos->offroute_valid           = 0;
    pos->offroute_expected_sensor = NULL;

    /* Set up mid-route reversal state if needed. */
    if (rp->has_reversal) {
        pos->midrev.active       = 1;
        pos->midrev.sensor       = rp->reversal_sensor;
        pos->midrev.final_target = rp->chosen_target;
        pos->midrev.final_offset = offset_mm;
        pos->midrev.sw_count     = rp->sw_count2;
        for (int i = 0; i < rp->sw_count2; i++) {
            pos->midrev.sw_nums[i] = rp->sw_nums2[i];
            pos->midrev.sw_dirs[i] = rp->sw_dirs2[i];
        }
        pos->midrev.dist_after = rp->dist_after_reversal_mm;
        pos->target_sensor    = rp->reversal_sensor;
        pos->target_offset_mm = 0;
        pos->orig_user_target   = pos->midrev.final_target;
        pos->orig_target_offset = offset_mm;

        pos->route_path_count = rp->path_count;
        for (int i = 0; i < rp->path_count; i++) pos->route_path[i] = rp->path_nodes[i];
        pos->midrev.path2_count = rp->path_count2;
        for (int i = 0; i < rp->path_count2; i++) pos->midrev.path2[i] = rp->path_nodes2[i];
    } else {
        pos->midrev.active    = 0;
        pos->target_sensor    = chosen_target;
        pos->target_offset_mm = offset_mm;

        pos->route_path_count   = rp->path_count;
        pos->midrev.path2_count = 0;
        for (int i = 0; i < rp->path_count; i++) pos->route_path[i] = rp->path_nodes[i];
    }

    pos->route_path_cursor = 0;

    int32_t pd = route_path_dist_from(pos->route_path, 0, pos->route_path_count);
    pos->dist_to_target_mm = (pd >= 0) ? pd + pos->target_offset_mm : 0;
    if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;

    pos->pending_target    = NULL;
    pos->pending_offset_mm = 0;
    pos->replan.next_us = 0;
    pos->replan.seen_generation = traffic_get_change_generation();
    pos->replan.blocker_mask = 0;

    pos_arm_switch_settle(pos, rp->sw_count,
                          need_initial_reverse ? POS_SWITCH_SETTLE_REVERSED
                                               : POS_SWITCH_SETTLE_NORMAL,
                          now_us);

    ui_mark_position_dirty();
    return 1;
}

int pos_try_direct_goto(train_pos_t *pos) {
    return pos_try_direct_goto_impl(pos, 1);
}

int pos_try_direct_goto_strict(train_pos_t *pos) {
    return pos_try_direct_goto_impl(pos, 0);
}
