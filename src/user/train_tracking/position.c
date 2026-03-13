#include "train_tracking/position.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "train_tracking/speed_table.h"
#include "timer.h"
#include "kassert.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>


train_pos_t g_pos[MAX_POS_TRAINS];

#define MAX_SENSORS 80


static uint8_t      g_pos_try_blocked[TRACK_MAX];
static route_plan_t g_pos_try_rp;
static route_plan_t g_pos_try_rp_unconstrained;
static route_plan_t g_pos_try_rp_temp;

#ifdef TRACK_D
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {227, 232, 242, 229, 230};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {153, 153, 153, 153, 153};

    static const int32_t GOTO_DECEL_OVERRIDE[MAX_SENSORS] = 
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 120, -1, -1, -1, -1, 101,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     130, -1, 140  , -1, -1, -1, 120, -1, -1, -1, -1, -1, -1, 200, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 270, -1,
     -1, -1, 200, -1, -1, -1, 270, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#else
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {226, 224, 226, 222, 236};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {167, 167, 167, 167, 167};

    static const int32_t GOTO_DECEL_OVERRIDE[MAX_SENSORS] = 
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, 200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
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

/* ===== Fixed loop switch settings ===== */

#define LOOP_SW_COUNT 7

/* Track A and Track B use the same inner loop, switch settings are identical.
 * Forward loop path:
 *   A3->BR14(S)->MR11->C13->E7->D7->MR9->BR8(S)->D9->E12->BR7(S)->D11->C16->MR6->C6->MR15->B15->A3
 * Reverse loop path (reverse sensors):
 *   A4->B16->BR15(S)->C5->BR6(S)->C15->D12->E11->D10->BR9(S)->D8->E8->C14->BR11(C)->MR14->A4
 */
static const int  LOOP_SW_NUMS_A[LOOP_SW_COUNT] = { 7,   8,   14,  11,  9,   6,   15  };
static const char LOOP_SW_DIRS_A[LOOP_SW_COUNT] = { 'S', 'S', 'S', 'C', 'S', 'S', 'S' };
static const int  LOOP_SW_NUMS_B[LOOP_SW_COUNT] = { 7,   8,   14,  11,  9,   6,   15  };
static const char LOOP_SW_DIRS_B[LOOP_SW_COUNT] = { 'S', 'S', 'S', 'C', 'S', 'S', 'S' };

/* ===== Position slot management ===== */

static train_pos_t *find_pos(int train_num) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        if (g_pos[i].train_num == train_num) return &g_pos[i];
    }
    return NULL;
}

static int32_t train_num_to_ind(int train_num) {
    if (13 <= train_num && train_num <= 15) {
        return train_num - 13;
    } else if (17 <= train_num && train_num <= 18) {
        return train_num - 14;
    } else if (train_num == 55) {
        return 0; /* use train-13 speed/decel calibration */
    }

    return -1;
}

static train_pos_t *find_or_create_pos(int train_num) {
    train_pos_t *p = find_pos(train_num);
    if (p) return p;
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        if (g_pos[i].train_num < 0) {
            int32_t train_ind = train_num_to_ind(train_num);
            if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return NULL;

            train_pos_t *slot = &g_pos[i];
            slot->train_num             = train_num;
            slot->train_ind             = train_ind;
            slot->cur_sensor            = NULL;
            slot->cur_sensor_time       = 0;
            slot->effective_v           = 0;
            slot->user_speed            = 0;
            slot->pred_next_sensor      = NULL;
            slot->pred_alt_sensor       = NULL;
            slot->pred_branch_node      = NULL;
            slot->pred_trigger_time     = 0;
            slot->last_time_err_us      = 0;
            slot->last_dist_err_mm      = 0;
            slot->route_state           = TRAIN_STATE_UNKNOWN;
            slot->target_sensor         = NULL;
            slot->target_offset_mm      = 0;
            slot->dist_to_target_mm     = 0;
            slot->pending_target        = NULL;
            slot->pending_offset_mm     = 0;
            slot->queued_target         = NULL;
            slot->queued_offset_mm      = 0;
            slot->queued_valid          = 0;
            slot->going_forward         = 1;
            slot->position_known        = 0;
            slot->orig_user_target      = NULL;
            slot->orig_target_offset    = 0;
            slot->last_plan_valid       = 0;
            slot->last_plan_loop_start  = NULL;
            slot->last_plan_target      = NULL;
            slot->last_plan_sw_count    = 0;
            for (int k = 0; k < 20; k++) {
                slot->last_plan_sw_nums[k] = 0;
                slot->last_plan_sw_dirs[k] = '?';
            }
            slot->offroute_valid           = 0;
            slot->offroute_expected_sensor = NULL;
            slot->offroute_actual_sensor   = NULL;
            slot->stopping_since_us       = 0;
            slot->wait_since_us           = 0;
            slot->next_replan_us          = 0;
            slot->dead_track_deadline_us  = 0;
            for (int s = 0; s < 15; s++) slot->cached_v[s] = 0;
            slot->speed_warmup_mm = 0;
            slot->midrev_active       = 0;
            slot->midrev_sensor       = NULL;
            slot->midrev_final_target = NULL;
            slot->midrev_final_offset = 0;
            slot->midrev_sw_count     = 0;
            slot->midrev_dist_after   = 0;
            for (int k = 0; k < 20; k++) {
                slot->midrev_sw_nums[k] = 0;
                slot->midrev_sw_dirs[k] = '?';
            }
            slot->last_attr_score = 0;
            slot->last_attr_conf  = 0;
            slot->find_dir_only   = 0;
            return slot;
        }
    }
    return NULL;
}

static int state_is_goto_active(train_route_state_t st) {
    return (st == TRAIN_STATE_STOPPING_GOTO     ||
            st == TRAIN_STATE_LOOP_FIND_DIR     ||
            st == TRAIN_STATE_ON_ROUTE          ||
            st == TRAIN_STATE_STOPPING          ||
            st == TRAIN_STATE_RECOVERY_STOPPING ||
            st == TRAIN_STATE_DEAD_TRACK        ||
            st == TRAIN_STATE_WAIT_RESOURCE);
}

static int apply_route_switches_safe(const int *sw_nums, const char *sw_dirs,
                                     int sw_count, int requester_train) {
    for (int i = sw_count - 1; i >= 0; i--) {
        int owner = traffic_can_set_switch(sw_nums[i], requester_train);
        if (owner >= 0) return 0;
    }
    for (int i = sw_count - 1; i >= 0; i--) {
        track_set_switch(sw_nums[i], sw_dirs[i]);
        track_update_switch(sw_nums[i], sw_dirs[i]);
    }
    resend_unreliable_switches(sw_nums, sw_dirs, sw_count);
    return 1;
}

void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us) {
    if (!pos) return;
    track_set_speed(pos->train_num, 0);
    traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);
    pos->route_state = TRAIN_STATE_WAIT_RESOURCE;
    pos->wait_since_us = now_us;
    pos->next_replan_us = now_us + REPLAN_INTERVAL_US;
    pos->stopping_since_us = now_us;
    pos->effective_v = 0;
    pos->pred_next_sensor = NULL;
    pos->pred_alt_sensor  = NULL;
    pos->pred_trigger_time = 0;
    pos->dead_track_deadline_us = 0;
    ui_mark_position_dirty();
}

/* ===== Public API ===== */

void pos_init(void) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        g_pos[i].train_num = -1;
    }
    traffic_init();
}

void pos_on_reverse(int train_num) {
    train_pos_t *pos = find_pos(train_num);
    if (!pos) return;

    pos->going_forward = !pos->going_forward;
    if (pos->cur_sensor && pos->cur_sensor->reverse)
        pos->cur_sensor = pos->cur_sensor->reverse;

    pos->pred_next_sensor  = NULL;
    pos->pred_alt_sensor   = NULL;
    pos->pred_trigger_time = 0;
    traffic_release_train(train_num);

    ui_mark_position_dirty();
}

void pos_on_speed_change(int train_num, int user_speed) {
    train_pos_t *pos = find_or_create_pos(train_num);
    if (!pos) return;

    /* Save calibrated EMA before a stop wipes effective_v.
     * Must happen before user_speed is updated so we know which slot to save. */
    if (user_speed == 0 &&
        pos->user_speed > 0 && pos->user_speed <= 14 &&
        pos->effective_v > 0) {
        pos->cached_v[pos->user_speed] = pos->effective_v;
    }

    pos->user_speed = user_speed;

    if (user_speed == 0 && state_is_goto_active(pos->route_state)) {
        traffic_release_train(train_num);
    }

    if (user_speed > 0 && user_speed <= 14) {
        int32_t cv = pos->cached_v[user_speed];
        pos->effective_v = (cv > 0) ? cv : speed_table_get_v(pos->train_ind, user_speed);
        pos->speed_warmup_mm = 400;
        /* Transition to KNOWN when the train resumes from a known-position state. */
        if (pos->cur_sensor != NULL &&
            (pos->route_state == TRAIN_STATE_STOPPED  ||
             pos->route_state == TRAIN_STATE_STOPPING_TR)) {
            pos->route_state = TRAIN_STATE_KNOWN;
        }
    } else {
        /* Speed set to 0.  Keep effective_v at its current value so that
         * the braking-time estimate in pos_on_tick is accurate */
        if (pos->route_state == TRAIN_STATE_KNOWN ||
            (pos->route_state == TRAIN_STATE_UNKNOWN &&
             pos->cur_sensor != NULL)) {
            pos->stopping_since_us = read_timer();
            pos->route_state       = TRAIN_STATE_STOPPING_TR;
        } else {
            pos->effective_v = 0;
        }
    }
}


void pos_apply_loop_switches(void) {
#ifdef TRACK_D
    const int  *sw_nums = LOOP_SW_NUMS_A;
    const char *sw_dirs = LOOP_SW_DIRS_A;
#else
    const int  *sw_nums = LOOP_SW_NUMS_B;
    const char *sw_dirs = LOOP_SW_DIRS_B;
#endif
    for (int i = 0; i < LOOP_SW_COUNT; i++) {
        track_set_switch(sw_nums[i], sw_dirs[i]);
    }
    ui_mark_switches_dirty();
}


int pos_try_direct_goto(train_pos_t *pos) {
    track_node *user_target = pos->pending_target;
    int32_t     offset_mm   = pos->pending_offset_mm;

    if (!pos->cur_sensor || !user_target) return 0;

    track_node *cur_sensor_orig = pos->cur_sensor; 

    /* Plan from pred_next_sensor if available.
     * Never use cur_sensor directly — the train has already passed it.
     * Fallback: walk the graph for the real next sensor; if the path leads
     * to an EXIT (dead end), use cur_sensor->reverse instead (needs reversal). */
    track_node *plan_start = pos->pred_next_sensor;
    int plan_start_needs_reverse = 0;
    if (!plan_start) {
        uint64_t dt_ignored = 0;
        plan_start = predict_next_sensor(pos, pos->cur_sensor, &dt_ignored);
    }
    if (!plan_start) {
        /* Dead-end forward path — must reverse; treat like origins[1]. */
        plan_start = cur_sensor_orig->reverse;
        plan_start_needs_reverse = 1;
    }

    int32_t tv        = GOTO_SPEED_MM_S[pos->train_ind];
    int32_t ta        = GOTO_DECEL_MM_S2[pos->train_ind];
    int32_t d_brake   = tv * tv / (2 * ta);
    int32_t threshold = GOTO_MIN_DIST_FACTOR * d_brake;

    /*
     * Two candidate origins:
     *   origins[0] = plan_start  (pred_next_sensor or graph-walk next) — forward direction
     *   origins[1] = cur_sensor->reverse                               — reversed direction
     * When plan_start_needs_reverse, origins[0] IS the reverse node;
     * skip origins[1] to avoid duplicate planning.
     */
    track_node *origins[2] = { plan_start,
                                (!plan_start_needs_reverse && cur_sensor_orig->reverse)
                                    ? cur_sensor_orig->reverse : NULL };
    uint8_t *blocked = g_pos_try_blocked;
    route_plan_t *rp = &g_pos_try_rp;
    route_plan_t *rp_unconstrained = &g_pos_try_rp_unconstrained;
    route_plan_t *rp_temp = &g_pos_try_rp_temp;
    traffic_build_constraints(pos->train_num, blocked);

    track_node  *chosen_origin = NULL;
    int32_t      best_total    = 0;
    int          need_initial_reverse = 0;
    int          blocked_by_reservation = 0;

    for (int o = 0; o < 2; o++) {
        if (!origins[o]) continue;
        if (!bfs_find_route_optimal_constrained(origins[o], user_target, d_brake, blocked, rp_temp)) {
            if (!blocked_by_reservation &&
                bfs_find_route_optimal(origins[o], user_target, d_brake, rp_unconstrained)) {
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
            need_initial_reverse  = (o == 1) || (o == 0 && plan_start_needs_reverse);
        }
    }

    if (!chosen_origin) {
        if (blocked_by_reservation) {
            pos_enter_wait_resource(pos, read_timer());
            return 1;
        }
        return 0;
    }

    uint64_t now_us = read_timer();
    track_node *eff_start = chosen_origin;
    track_node *chosen_target = rp->has_reversal ? rp->reversal_sensor
                                                  : rp->chosen_target;

    for (int i = rp->sw_count - 1; i >= 0; i--) {
        int owner = traffic_can_set_switch(rp->sw_nums[i], pos->train_num);
        if (owner >= 0) {
            pos_enter_wait_resource(pos, now_us);
            return 1;
        }
    }

    traffic_release_train(pos->train_num);
    if (!traffic_reserve_plan(pos->train_num, eff_start, rp)) {
        pos_enter_wait_resource(pos, now_us);
        return 1;
    }

    if (need_initial_reverse) {
        track_reverse(pos->train_num);
        pos->cur_sensor      = cur_sensor_orig->reverse;
        pos->going_forward   = !pos->going_forward;
        pos->pred_next_sensor       = cur_sensor_orig->reverse;
        pos->pred_alt_sensor        = NULL;
        pos->pred_trigger_time      = 0;
        pos->dead_track_deadline_us = 0;
    }

    if (!apply_route_switches_safe(rp->sw_nums, rp->sw_dirs, rp->sw_count, pos->train_num)) {
        traffic_release_train(pos->train_num);
        pos_enter_wait_resource(pos, now_us);
        return 1;
    }
    if (rp->sw_count > 0) ui_mark_switches_dirty();


    pos->last_plan_valid      = 1;
    pos->last_plan_loop_start = eff_start;
    pos->last_plan_target     = chosen_target;
    pos->last_plan_sw_count   = rp->sw_count;
    for (int i = 0; i < rp->sw_count; i++) {
        pos->last_plan_sw_nums[i] = rp->sw_nums[i];
        pos->last_plan_sw_dirs[i] = rp->sw_dirs[i];
    }
    pos->offroute_valid           = 0;
    pos->offroute_expected_sensor = NULL;
    pos->offroute_actual_sensor   = NULL;

    /* Set up mid-route reversal state if needed. */
    if (rp->has_reversal) {
        pos->midrev_active       = 1;
        pos->midrev_sensor       = rp->reversal_sensor;
        pos->midrev_final_target = rp->chosen_target;
        pos->midrev_final_offset = offset_mm;
        pos->midrev_sw_count     = rp->sw_count2;
        for (int i = 0; i < rp->sw_count2; i++) {
            pos->midrev_sw_nums[i] = rp->sw_nums2[i];
            pos->midrev_sw_dirs[i] = rp->sw_dirs2[i];
        }
        pos->midrev_dist_after = rp->dist_after_reversal_mm;
        pos->target_sensor    = rp->reversal_sensor;
        pos->target_offset_mm = 0;
        pos->orig_user_target   = pos->midrev_final_target;
        pos->orig_target_offset = offset_mm;
    } else {
        pos->midrev_active    = 0;
        pos->target_sensor    = chosen_target;
        pos->target_offset_mm = offset_mm;
    }

    pos->user_speed  = GOTO_USER_SPEED;
    int can_spd = 1 + (pos->user_speed - 1) * 77;
    track_set_speed(pos->train_num, can_spd);
    int32_t cv = pos->cached_v[pos->user_speed];
    pos->effective_v     = (cv > 0) ? cv : speed_table_get_v(pos->train_ind, pos->user_speed);
    pos->speed_warmup_mm = 400;
    pos->cur_sensor_time = now_us;

    int32_t dist_first_leg = rp->has_reversal
                             ? rp->dist_to_reversal_mm
                             : follow_dist(eff_start, chosen_target, 200);
    if (dist_first_leg < 0)
        dist_first_leg = follow_dist(pos->cur_sensor, pos->target_sensor, 200);
    pos->dist_to_target_mm = (dist_first_leg >= 0)
                             ? ((dist_first_leg + pos->target_offset_mm > 0)
                                ? dist_first_leg + pos->target_offset_mm : 0)
                             : 0;

    pos->pending_target    = NULL;
    pos->pending_offset_mm = 0;
    pos->route_state = TRAIN_STATE_ON_ROUTE;
    pos->wait_since_us = 0;
    pos->next_replan_us = 0;

    if (!need_initial_reverse) {
        uint64_t dt = 0;
        pos->pred_next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
        pos->pred_trigger_time = now_us + dt;
    }

    if (pos->pred_next_sensor != NULL && pos->pred_trigger_time > now_us) {
        uint64_t T1 = pos->pred_trigger_time - now_us;
        uint64_t T2 = 0;
        predict_next_sensor(pos, pos->pred_next_sensor, &T2);
        pos->dead_track_deadline_us =
            now_us + DEAD_TRACK_DEADLINE_MULTIPLIER * (T1 + T2);
    } else {
        pos->dead_track_deadline_us = 0;
    }

    ui_mark_position_dirty();
    return 1;
}

int pos_goto(int train_num, track_node *target, int32_t offset_mm) {
    KASSERT(target != NULL);
    if (!target) return 0;

    train_pos_t *pos = find_or_create_pos(train_num);
    KASSERT(pos != NULL);
    if (!pos) return 0;

    if (state_is_goto_active(pos->route_state)) {
        pos->queued_target = target;
        pos->queued_offset_mm = offset_mm;
        pos->queued_valid = 1;
        ui_mark_position_dirty();
        return 1;
    }

    traffic_release_train(train_num);

    pos->pending_target     = target;
    pos->pending_offset_mm  = offset_mm;
    pos->orig_user_target   = target;
    pos->orig_target_offset = offset_mm;
    pos->last_plan_valid    = 0;
    pos->last_plan_loop_start = NULL;
    pos->last_plan_target   = NULL;
    pos->last_plan_sw_count = 0;
    pos->offroute_valid           = 0;
    pos->offroute_expected_sensor = NULL;
    pos->offroute_actual_sensor   = NULL;


    pos->target_sensor     = target;
    pos->target_offset_mm  = offset_mm;
    pos->dist_to_target_mm = 0;
    pos->wait_since_us = 0;
    pos->next_replan_us = 0;
    ui_mark_position_dirty();

    if (pos->route_state == TRAIN_STATE_UNKNOWN) {
        /* Position unknown — start moving to trigger a sensor and acquire position.
         * handle_sensor will stop the train and transition to STOPPING_GOTO. */
        pos->user_speed = GOTO_USER_SPEED;
        int can_spd = 1 + (pos->user_speed - 1) * 77;
        track_set_speed(train_num, can_spd);
        pos->effective_v     = speed_table_get_v(pos->train_ind, pos->user_speed);
        pos->cur_sensor_time = read_timer();
        pos->route_state     = TRAIN_STATE_LOOP_FIND_DIR;

    } else if (pos->route_state == TRAIN_STATE_KNOWN) {
        /* Position known, train running via tr. Stop and replan. */
        track_set_speed(train_num, 0);
        pos->stopping_since_us = read_timer();
        pos->route_state = TRAIN_STATE_STOPPING_GOTO;

    } else if (pos->route_state == TRAIN_STATE_STOPPING_TR) {
        /* Train already decelerating; redirect post-stop to replan. */
        if (pos->user_speed == 0) pos->user_speed = GOTO_USER_SPEED;
        pos->route_state = TRAIN_STATE_STOPPING_GOTO;

    } else if (pos->route_state == TRAIN_STATE_STOPPED) {
        int ok = pos_try_direct_goto(pos);
        KASSERT(ok);
    }

    return 1;
}

int pos_start_direction_find(int train_num) {
    train_pos_t *pos = find_or_create_pos(train_num);
    if (!pos) return 0;
    if (pos->route_state != TRAIN_STATE_UNKNOWN) return 0;

    traffic_release_train(train_num);

    pos->pending_target           = NULL;
    pos->pending_offset_mm        = 0;
    pos->orig_user_target         = NULL;
    pos->orig_target_offset       = 0;
    pos->target_sensor            = NULL;
    pos->target_offset_mm         = 0;
    pos->dist_to_target_mm        = 0;
    pos->wait_since_us            = 0;
    pos->next_replan_us           = 0;
    pos->last_plan_valid          = 0;
    pos->offroute_valid           = 0;
    pos->offroute_expected_sensor = NULL;
    pos->offroute_actual_sensor   = NULL;
    pos->find_dir_only            = 1;

    pos->user_speed  = GOTO_USER_SPEED;
    int can_spd = 1 + (pos->user_speed - 1) * 77;
    track_set_speed(train_num, can_spd);
    pos->effective_v     = speed_table_get_v(pos->train_ind, pos->user_speed);
    pos->cur_sensor_time = read_timer();
    pos->route_state     = TRAIN_STATE_LOOP_FIND_DIR;

    ui_mark_position_dirty();
    return 1;
}

int pos_is_train_goto_active(int train_num) {
    train_pos_t *pos = find_pos(train_num);
    if (!pos) return 0;
    return state_is_goto_active(pos->route_state);
}

int pos_is_train_position_known(int train_num) {
    train_pos_t *pos = find_pos(train_num);
    if (!pos) return 0;
    return pos->cur_sensor != NULL && pos->position_known;
}

train_pos_t *pos_get(int train_num) {
    return find_pos(train_num);
}

train_pos_t *pos_get_by_index(int i) {
    if (i < 0 || i >= MAX_POS_TRAINS) return NULL;
    return &g_pos[i];
}

int pos_queue_goto(int train_num, track_node *target, int32_t offset_mm) {
    train_pos_t *pos = find_or_create_pos(train_num);
    if (!pos || !target) return 0;
    pos->queued_target = target;
    pos->queued_offset_mm = offset_mm;
    pos->queued_valid = 1;
    ui_mark_position_dirty();
    return 1;
}

void pos_mark_routes_dirty(void) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (pos->train_num < 0) continue;
        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            pos->next_replan_us = 0;
        }
    }
}

track_node *pos_find_node(const char *name) {
    if (!name) return NULL;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type != NODE_NONE && g_track[i].name != NULL) {
            const char *a = g_track[i].name;
            const char *b = name;
            while (*a && *b && *a == *b) { a++; b++; }
            if (*a == '\0' && *b == '\0') return &g_track[i];
        }
    }
    return NULL;
}
