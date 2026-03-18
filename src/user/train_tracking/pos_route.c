#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "train_tracking/speed_table.h"
#include "timer.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

#define MAX_SENSORS 80

static uint8_t      g_pos_try_blocked[TRACK_MAX];
static char         g_pos_try_fixed_sw_dirs[TRACK_MAX];
static route_plan_t g_pos_try_rp;
static route_plan_t g_pos_try_rp_unconstrained;
static route_plan_t g_pos_try_rp_temp;

#ifdef TRACK_D
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {227, 232, 242, 229, 230};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {153, 153, 153, 153, 153};

    static const int32_t GOTO_DECEL_OVERRIDE[MAX_SENSORS] =
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#else
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {226, 224, 226, 222, 236};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {158, 156, 167, 165, 153};

    static const int32_t GOTO_DECEL_OVERRIDE[MAX_SENSORS] =
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#endif

int32_t speed_table_get_v(int32_t train_ind, int user_speed) {
    if (user_speed != GOTO_USER_SPEED) return 0;
    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;
    return GOTO_SPEED_MM_S[train_ind];
}

int32_t speed_table_get_decel(int32_t train_ind, int user_speed, track_node *target) {
    if (user_speed != GOTO_USER_SPEED) return 0;
    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;

    if (target->type == NODE_SENSOR) {
        int32_t override = GOTO_DECEL_OVERRIDE[target->num];
        return (override > -1) ? override : GOTO_DECEL_MM_S2[train_ind];
    }
    return GOTO_DECEL_MM_S2[train_ind];
}

static int route_switch_needs_change(int sw_num, char desired_dir) {
    int sw_idx = track_switch_to_index(sw_num);
    if (sw_idx < 0) return 1;
    char current_dir = track_get_switch_state()[sw_idx].state;
    return current_dir != desired_dir;
}

static void pos_build_fixed_switch_dirs(int requester_train, char fixed_sw_dirs[TRACK_MAX]) {
    for (int i = 0; i < TRACK_MAX; i++) fixed_sw_dirs[i] = '?';

    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        if (n->type != NODE_BRANCH) continue;

        int sw_idx = track_switch_to_index(n->num);
        if (sw_idx < 0) continue;

        char current_dir = track_get_switch_state()[sw_idx].state;
        if (current_dir != 'S' && current_dir != 'C') continue;

        if (traffic_can_set_switch(n->num, requester_train) == requester_train) {
            fixed_sw_dirs[i] = current_dir;
        }
    }
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

void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us) {
    if (!pos) return;
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

int pos_try_direct_goto(train_pos_t *pos) {
    track_node *user_target = pos->pending_target;
    int32_t     offset_mm   = pos->pending_offset_mm;

    if (!pos->cur_sensor || !user_target) return 0;

    track_node *cur_sensor_orig = pos->cur_sensor;

    /* Plan from pred.next_sensor if available.
     * Never use cur_sensor directly — the train has already passed it.
     * Reverse planning starts from prediction->reverse, except at a dead-end
     * where no forward prediction exists and we must fall back to cur->reverse. */
    track_node *plan_start = pos->pred.next_sensor;
    if (!plan_start) {
        uint64_t dt_ignored = 0;
        plan_start = predict_next_sensor(pos, pos->cur_sensor, &dt_ignored);
    }
    track_node *reverse_plan_start = plan_start
                                     ? plan_start->reverse
                                     : cur_sensor_orig->reverse;

    int32_t tv        = speed_table_get_v(pos->train_ind, GOTO_USER_SPEED);
    int32_t ta        = GOTO_DECEL_MM_S2[pos->train_ind];
    int32_t d_brake   = tv * tv / (2 * ta);
    int32_t d_stop    = d_brake + (int32_t)((int64_t)tv * (int64_t)STOP_EARLY_US[pos->train_ind] / 1000000LL);
    int32_t threshold = GOTO_MIN_DIST_FACTOR * d_stop;

    /*
     * Two candidate origins:
     *   origins[0] = plan_start           (prediction)          — forward direction
     *   origins[1] = reverse_plan_start   (prediction->reverse, or cur->reverse
     *                                     when forward prediction is unavailable)
     */
    track_node *origins[2] = { plan_start, reverse_plan_start };
    uint8_t *blocked = g_pos_try_blocked;
    char *fixed_sw_dirs = g_pos_try_fixed_sw_dirs;
    route_plan_t *rp = &g_pos_try_rp;
    route_plan_t *rp_unconstrained = &g_pos_try_rp_unconstrained;
    route_plan_t *rp_temp = &g_pos_try_rp_temp;
    traffic_build_constraints(pos->train_num, blocked);
    pos_build_fixed_switch_dirs(pos->train_num, fixed_sw_dirs);

    track_node  *chosen_origin = NULL;
    int32_t      best_total    = 0;
    int          need_initial_reverse = 0;
    int          blocked_by_reservation = 0;

    for (int o = 0; o < 2; o++) {
        if (!origins[o]) continue;
        if (!bfs_find_route_optimal_constrained(origins[o], user_target, d_stop,
                                                blocked, fixed_sw_dirs, rp_temp)) {
            if (!blocked_by_reservation &&
                bfs_find_route_optimal_constrained(origins[o], user_target, d_stop,
                                                   NULL, fixed_sw_dirs, rp_unconstrained)) {
                blocked_by_reservation = 1;
            }
            continue;
        }

        int32_t effective_d = rp_temp->has_reversal
                              ? rp_temp->dist_to_reversal_mm + rp_temp->dist_after_reversal_mm
                              : rp_temp->total_dist_mm;
        if (effective_d <= threshold) continue;
        if (chosen_origin == NULL || rp_temp->total_dist_mm < best_total) {
            *rp                   = *rp_temp;
            chosen_origin         = origins[o];
            best_total            = rp_temp->total_dist_mm;
            need_initial_reverse  = (o == 1);
        }
    }

    if (!chosen_origin) {
        /* no long-enough direct route found.
         * Reverse immediately, drive to a far sensor, reverse again, then
         * continue to the real target.  Covers the EXIT dead-end case where
         * the target and current position are on the same short segment. */
        track_node *boot_start = reverse_plan_start;
        int allow_bootstrap = !blocked_by_reservation ||
                              pos->route_state == TRAIN_STATE_WAIT_RESOURCE;
        if (!boot_start || !allow_bootstrap ||
            !bfs_find_bootstrap_midrev(boot_start, user_target, d_stop,
                                       blocked, fixed_sw_dirs, rp)) {
            if (blocked_by_reservation) {
                pos_enter_wait_resource(pos, read_timer());
                return 1;
            }
            return 0;
        }
        chosen_origin        = boot_start;
        need_initial_reverse = 1;
    }

    uint64_t now_us = read_timer();
    track_node *eff_start = chosen_origin;
    track_node *chosen_target = rp->has_reversal ? rp->reversal_sensor
                                                  : rp->chosen_target;
    route_plan_t reserve_plan = *rp;
    if (reserve_plan.has_reversal) {
        /* Reserve only the first leg up to the reversal point.
         * The second leg is reserved when the train actually reaches the midpoint. */
        reserve_plan.path_count2 = 0;
    }
    /* Keep the parked sensor window until the next real hit.
     * The new route is added on top of the existing stopped reservation. */
    if (!traffic_can_reserve_plan(pos->train_num, &reserve_plan)) {
        pos_enter_wait_resource(pos, now_us);
        return 1;
    }
    if (!pos_apply_route_switches_safe(rp->sw_nums, rp->sw_dirs, rp->sw_count,
                                       pos->train_num)) {
        pos_enter_wait_resource(pos, now_us);
        return 1;
    }
    if (!traffic_reserve_plan(pos->train_num, eff_start, &reserve_plan)) {
        pos_enter_wait_resource(pos, now_us);
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
    pos_wait_switch_settle(rp->sw_count);

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

    pos_launch_at_goto_speed(pos, now_us);

    int32_t pd = route_path_dist_from(pos->route_path, 0, pos->route_path_count);
    pos->dist_to_target_mm = (pd >= 0) ? pd + pos->target_offset_mm : 0;
    if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;

    pos->route_rem_tick_us = now_us;

    pos->pending_target    = NULL;
    pos->pending_offset_mm = 0;
    pos->route_state = TRAIN_STATE_ON_ROUTE;
    pos->replan.next_us = 0;
    pos->replan.seen_generation = traffic_get_change_generation();

    if (!need_initial_reverse) {
        uint64_t dt = 0;
        pos->pred.next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
        pos->pred.trigger_time = now_us + dt;
        pos->pred.skipped_sensor_count = 0;
    }

    pos_refresh_dead_track_deadline(pos, now_us);

    ui_mark_position_dirty();
    return 1;
}
