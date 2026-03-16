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

/* Time from sending the command to the train actually starting
 * to move */
#define GO_LATENCY_US 50000ULL

#ifdef TRACK_D
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {227, 232, 242, 229, 230};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {153, 153, 153, 153, 153};
    static const int32_t GOTO_ACCEL_MM_S2[MAX_PHYSICAL_TRAINS] =
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
    static const int32_t GOTO_ACCEL_MM_S2[MAX_PHYSICAL_TRAINS] =
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

/* Start train moving at GOTO_USER_SPEED to acquire position (FIND_POS).
 * Used by pos_goto (UNKNOWN state) and pos_start_direction_find. */
static void pos_begin_pos_find(train_pos_t *pos) {
    pos->user_speed      = GOTO_USER_SPEED;
    int can_spd          = 1 + (GOTO_USER_SPEED - 1) * 77;
    track_set_speed(pos->train_num, can_spd);
    pos->effective_v     = 0;               /* will be ramped by tick */
    pos->speed_warmup_mm = 800;
    pos->cur_sensor_time = read_timer();
    pos->is_accelerating = 1;
    pos->accel_start_us  = pos->cur_sensor_time + GO_LATENCY_US;
    pos->route_state     = TRAIN_STATE_FIND_POS;
}

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
            slot->pred.next_sensor      = NULL;
            slot->pred.alt_sensor       = NULL;
            slot->pred.branch_node      = NULL;
            slot->pred.trigger_time     = 0;
            slot->pred.last_time_err_us = 0;
            slot->pred.last_dist_err_mm = 0;
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
            slot->position_known        = 1;
            track_send_direction(train_num, 0x01);  /* init direction: forward */
            slot->orig_user_target      = NULL;
            slot->orig_target_offset    = 0;
            slot->offroute_valid           = 0;
            slot->offroute_expected_sensor = NULL;
            slot->stopping_since_us        = 0;
            slot->replan.next_us           = 0;
            slot->replan.retry_count       = 0;
            slot->replan.rand_state        = (uint32_t)(slot->train_num * 1234567u + 1u);
            slot->dead_track_deadline_us   = 0;
            for (int s = 0; s < 15; s++) slot->cached_v[s] = 0;
            slot->speed_warmup_mm      = 0;
            slot->accel_a_eff          = GOTO_ACCEL_MM_S2[train_ind];
            slot->is_accelerating      = 0;
            slot->accel_start_us       = 0;
            slot->midrev.active        = 0;
            slot->midrev.sensor        = NULL;
            slot->midrev.final_target  = NULL;
            slot->midrev.final_offset  = 0;
            slot->route_path_count   = 0;
            slot->route_path_cursor  = 0;
            slot->route_rem_tick_us  = 0;
            slot->midrev.path2_count   = 0;
            slot->midrev.sw_count      = 0;
            slot->midrev.dist_after    = 0;
            for (int k = 0; k < 20; k++) {
                slot->midrev.sw_nums[k] = 0;
                slot->midrev.sw_dirs[k] = '?';
            }
            slot->find_pos_only = 0;
            return slot;
        }
    }
    return NULL;
}


void pos_clear_prediction(train_pos_t *pos) {
    pos->pred.next_sensor  = NULL;
    pos->pred.alt_sensor   = NULL;
    pos->pred.branch_node  = NULL;
    pos->pred.trigger_time = 0;
    pos->dead_track_deadline_us = 0;
}

void pos_launch_at_goto_speed(train_pos_t *pos, uint64_t now_us) {
    pos->user_speed      = GOTO_USER_SPEED;
    int can_spd          = 1 + (GOTO_USER_SPEED - 1) * 77;
    track_set_speed(pos->train_num, can_spd);
    pos->effective_v     = 0;               
    pos->speed_warmup_mm = 800;
    pos->cur_sensor_time = now_us;
    pos->is_accelerating = 1;
    pos->accel_start_us  = now_us + GO_LATENCY_US;
}

void pos_restore_pending_target(train_pos_t *pos) {
    if (pos->pending_target == NULL && pos->orig_user_target != NULL) {
        pos->pending_target    = pos->orig_user_target;
        pos->pending_offset_mm = pos->orig_target_offset;
    }
}

void pos_save_ema_and_stop(train_pos_t *pos) {
    if (pos->user_speed > 0 && pos->user_speed <= 14)
        pos->cached_v[pos->user_speed] = pos->effective_v;
    pos->effective_v = 0;
}

static int state_is_goto_active(train_route_state_t st) {
    return (st == TRAIN_STATE_STOPPING_GOTO     ||
            st == TRAIN_STATE_FIND_POS     ||
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
    return 1;
}

void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us) {
    if (!pos) return;
    track_set_speed(pos->train_num, 0);
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    pos->going_forward, TRAIN_BODY_MM, NULL);
    pos->route_state = TRAIN_STATE_WAIT_RESOURCE;
    pos->replan.retry_count = 0;
    pos->replan.next_us = now_us + REPLAN_INTERVAL_US;
    pos->stopping_since_us = now_us;
    pos->effective_v = 0;
    pos_clear_prediction(pos);
    ui_mark_position_dirty();
}

/* ===== Public API ===== */

void pos_init(void) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        g_pos[i].train_num = -1;
    }
    traffic_init();
    route_init();
}

void pos_on_reverse(int train_num) {
    train_pos_t *pos = find_pos(train_num);
    if (!pos) return;

    pos->going_forward = !pos->going_forward;
    if (pos->cur_sensor && pos->cur_sensor->reverse)
        pos->cur_sensor = pos->cur_sensor->reverse;

    pos_clear_prediction(pos);
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


    if (user_speed > 0 && user_speed <= 14) {
        int32_t cv = pos->cached_v[user_speed];
        pos->effective_v = (cv > 0) ? cv : speed_table_get_v(pos->train_ind, user_speed);
        pos->speed_warmup_mm = 800;
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




int pos_try_direct_goto(train_pos_t *pos) {
    track_node *user_target = pos->pending_target;
    int32_t     offset_mm   = pos->pending_offset_mm;

    if (!pos->cur_sensor || !user_target) return 0;

    track_node *cur_sensor_orig = pos->cur_sensor;

    /* Plan from pred.next_sensor if available.
     * Never use cur_sensor directly — the train has already passed it.
     * Fallback: walk the graph for the real next sensor; if the path leads
     * to an EXIT (dead end), use cur_sensor->reverse instead (needs reversal). */
    track_node *plan_start = pos->pred.next_sensor;
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
    int32_t d_stop    = d_brake + (int32_t)((int64_t)tv * (int64_t)STOP_EARLY_US[pos->train_ind] / 1000000LL);
    int32_t threshold = GOTO_MIN_DIST_FACTOR * d_stop;

    /*
     * Two candidate origins:
     *   origins[0] = plan_start  (pred.next_sensor or graph-walk next) — forward direction
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
        if (!bfs_find_route_optimal_constrained(origins[o], user_target, d_stop, blocked, rp_temp)) {
            if (!blocked_by_reservation &&
                bfs_find_route_optimal(origins[o], user_target, d_stop, rp_unconstrained)) {
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
        /* no long-enough direct route found.
         * Reverse immediately, drive to a far sensor, reverse again, then
         * continue to the real target.  Covers the EXIT dead-end case where
         * the target and current position are on the same short segment. */
        track_node *boot_start = cur_sensor_orig->reverse;
        if (!boot_start ||
            !bfs_find_bootstrap_midrev(boot_start, user_target, d_stop, blocked, rp)) {
            return 0;
        }
        chosen_origin        = boot_start;
        need_initial_reverse = 1;
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
        pos->cur_sensor    = cur_sensor_orig->reverse;
        pos->going_forward = !pos->going_forward;
        pos->pred.next_sensor  = cur_sensor_orig->reverse;
        pos->pred.alt_sensor   = NULL;
        pos->pred.branch_node  = NULL;
        pos->pred.trigger_time = 0;
        pos->dead_track_deadline_us = 0;
    }

    if (!apply_route_switches_safe(rp->sw_nums, rp->sw_dirs, rp->sw_count, pos->train_num)) {
        traffic_release_train(pos->train_num);
        pos_enter_wait_resource(pos, now_us);
        return 1;
    }
    if (rp->sw_count > 0) ui_mark_switches_dirty();


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

    if (!need_initial_reverse) {
        uint64_t dt = 0;
        pos->pred.next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
        pos->pred.trigger_time = now_us + dt;
    }

    if (pos->pred.next_sensor != NULL && pos->pred.trigger_time > now_us) {
        uint64_t T1 = pos->pred.trigger_time - now_us;
        uint64_t T2 = 0;
        predict_next_sensor(pos, pos->pred.next_sensor, &T2);
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
    pos->offroute_valid           = 0;
    pos->offroute_expected_sensor = NULL;

    pos->target_sensor     = target;
    pos->target_offset_mm  = offset_mm;
    pos->dist_to_target_mm = 0;
    pos->replan.next_us = 0;
    ui_mark_position_dirty();

    if (pos->route_state == TRAIN_STATE_UNKNOWN) {
        /* Position unknown — start moving to trigger a sensor and acquire position.
         * handle_sensor will stop the train and transition to STOPPING_GOTO. */
        pos_begin_pos_find(pos);

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
    pos->replan.next_us           = 0;
    pos->offroute_valid           = 0;
    pos->offroute_expected_sensor = NULL;
    pos->find_pos_only            = 1;

    pos_begin_pos_find(pos);

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
            pos->replan.next_us = 0;
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
