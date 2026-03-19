/*
 * pos_event.c — position-tracking event handlers
 *
 * Handles two event sources that drive the train position FSM:
 *   pos_on_sensor_trigger()  — a sensor fired (CAN RX interrupt path)
 *   pos_on_tick()            — 10 ms periodic tick (dead-reckoning + timeouts)
 *
 * Internal call structure:
 *
 *   pos_on_sensor_trigger
 *     traffic_attribute_sensor        (ownership)
 *     correct_alt_branch_switch       (switch state correction)
 *     handle_sensor
 *       update_sensor_stats           (EMA, warmup, prediction error)
 *       handle_dead_track_sensor      (ignore sensor while terminal dead-track)
 *       update_next_prediction        (predict next sensor + deadline)
 *
 *   pos_on_tick
 *     pos_replan_on_tick      (WAIT_RESOURCE backoff replan)
 *     per-train loop:
 *       tick_handle_recovery_stopping
 *       tick_handle_stopping_tr
 *       tick_handle_stopping_goto
 *       tick_handle_stopping          (STOPPING→STOPPED, calls handle_midrev_resume
 *                                      or handle_normal_stop)
 *       tick_check_brake_point        (ON_ROUTE distance tracking + brake trigger)
 *       tick_check_dead_track         (deadline expiry → DEAD_TRACK)
 *       tick_advance_prediction       (skip stale sensor, advance pred)
 */

#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "train_tracking/speed_table.h"
#include "timer.h"
#include "kassert.h"
#include "ui.h"
#include "util.h"
#include "train_control.h"
#include <stdint.h>
#include <stddef.h>

/* Stop command lead time for overshoot compensation (microseconds). */
#ifdef TRACK_D
    uint64_t STOP_EARLY_US[MAX_PHYSICAL_TRAINS] = {930000ULL, 930000ULL, 930000ULL, 930000ULL, 930000ULL};
#else
    uint64_t STOP_EARLY_US[MAX_PHYSICAL_TRAINS] = {1200000ULL, 1200000ULL, 1200000ULL, 1200000ULL, 1200000ULL};
#endif

// Offsets at end of train with undershoot of 2cm
static uint32_t Train_Forward_Stop_Offset = 64;
static uint32_t Train_Reverse_Stop_Offset = 176;


/* ===== Acceleration model helper ===== */

/* Compute kinematic effective_v for a train in the acceleration phase.
 * v(t) = a * t_moving, clamped to v_goto once full speed is reached.
 * Clears is_accelerating automatically when v_goto is reached.
 */
static void accel_update_v(train_pos_t *pos, uint64_t now_us) {
    if (!pos->is_accelerating) return;

    int64_t t_moving_us = (int64_t)now_us - (int64_t)pos->accel_start_us;
    if (t_moving_us <= 0) {
        pos->effective_v = 0;   /* still within GO_LATENCY_US window */
        return;
    }

    int32_t v_goto    = speed_table_get_v(pos->train_ind, GOTO_USER_SPEED);
    int64_t t_accel_us = (int64_t)v_goto * 1000000LL / (int64_t)pos->accel_a_eff;

    if (t_moving_us >= t_accel_us) {
        pos->effective_v  = v_goto;
        pos->is_accelerating = 0;   /* reached full speed */
    } else {
        pos->effective_v =
            (int32_t)((int64_t)pos->accel_a_eff * t_moving_us / 1000000LL);
    }
}

/* ===== Shared brake-time helper ===== */

/* Estimate time (us) for a braking train to reach a full stop. */
static uint64_t calc_brake_us(train_pos_t *pos) {
    int32_t decel = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
    if (pos->effective_v > 0 && decel > 0)
        return STOP_EARLY_US[pos->train_ind] + (uint64_t)pos->effective_v * 1500000ULL / (uint64_t)decel;
    return 1000000ULL;
}

static int brake_elapsed(train_pos_t *pos, uint64_t now_us) {
    return now_us >= pos->stopping_since_us + calc_brake_us(pos);
}

/* Return 1 when `hit` definitively lies on the alternate leg of the next branch.
 * This covers both the first alternate sensor and a short run beyond it when the
 * train missed that first sensor but still took the wrong turnout. */
static int hit_matches_alt_branch(const train_pos_t *pos, track_node *hit) {
    if (!pos || !hit) return 0;
    if (pos->pred.branch_node == NULL || pos->pred.alt_sensor == NULL) return 0;
    if (pos->pred.alt_sensor == hit) return 1;

    int32_t alt_dist = follow_dist(pos->pred.alt_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
    if (alt_dist < 0) return 0;

    if (pos->pred.next_sensor == NULL) return 1;
    return follow_dist(pos->pred.next_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS) < 0;
}

/* Return the remaining route_path cursor for a sensor hit, or -1 when the hit
 * is not on the planned sensor sequence from the current cursor onward.  This
 * deliberately ignores mutex-only reservation padding around the route. */
static int route_path_hit_cursor(const train_pos_t *pos, track_node *hit) {
    if (!pos || !hit || pos->route_path_count <= 0) return -1;

    int start = pos->route_path_cursor;
    if (start < 0) start = 0;
    if (start >= pos->route_path_count) return -1;

    int hit_idx = (int)(hit - g_track);
    for (int k = start; k < pos->route_path_count; k++) {
        int idx = (int)pos->route_path[k];
        if (idx < 0 || idx >= TRACK_MAX) continue;
        if (g_track[idx].type != NODE_SENSOR) continue;
        if (idx == hit_idx) return k;
    }
    return -1;
}


/* ===== handle_sensor sub-functions ===== */

/* Update EMA speed, warmup, and prediction-error stats on a sensor hit.
 * Sets *was_predicted for use by the FSM. */
static void update_sensor_stats(train_pos_t *pos, track_node *hit,
                                uint64_t time_us,
                                int *was_predicted) {
    *was_predicted = (pos->pred.next_sensor == hit);

    /* Warmup: suppress EMA calibration for the first 400 mm after a speed change. */
    if (pos->speed_warmup_mm > 0 && pos->cur_sensor != NULL) {
        int32_t seg = follow_dist(pos->cur_sensor, hit, 100);
        if (seg > 0) {
            pos->speed_warmup_mm -= seg;
            if (pos->speed_warmup_mm < 0) pos->speed_warmup_mm = 0;
        }
    }
    int in_warmup = (pos->speed_warmup_mm > 0);

    /* Prediction error */
    if (*was_predicted && pos->pred.trigger_time > 0) {
        pos->pred.last_time_err_us = (int64_t)time_us - (int64_t)pos->pred.trigger_time;
        int64_t derr = (int64_t)pos->effective_v * pos->pred.last_time_err_us / 1000000LL;
        if (derr >  99999) derr =  99999;
        if (derr < -99999) derr = -99999;
        pos->pred.last_dist_err_mm = (int32_t)derr;
        ui_mark_prediction_dirty();
    }

    /* EMA speed update — only while cruising ON_ROUTE, predicted hit, dt > 10 ms. */
    if (!in_warmup && pos->route_state == TRAIN_STATE_ON_ROUTE &&
        *was_predicted && pos->cur_sensor && pos->effective_v > 0) {
        int32_t meas_dist = follow_dist(pos->cur_sensor, hit, 100);
        uint64_t dt = time_us - pos->cur_sensor_time;
        if (dt > 10000 && meas_dist > 0) {
            int32_t meas_v = (int32_t)((int64_t)meas_dist * 1000000LL / (int64_t)dt);
            if (meas_v > 1800) meas_v = 1800;
            pos->effective_v = (int32_t)((7LL * pos->effective_v + meas_v) / 8);
        }
    }
}

/* DEAD_TRACK is terminal: ignore any later attributed sensor and keep the train
 * parked in place. Returns 1 if this function handled the event. */
static int handle_dead_track_sensor(train_pos_t *pos) {
    if (!pos) return 1;
    track_set_speed(pos->train_num, 0);
    pos->effective_v = 0;
    pos->is_accelerating = 0;
    ui_mark_position_dirty();
    return 1;
}

static void enter_terminal_dead_track(train_pos_t *pos) {
    if (!pos) return;

    int can_rescue_dead_track =
        pos->route_state == TRAIN_STATE_ON_ROUTE &&
        pos->accel_start_us > 0 &&
        pos->cur_sensor_time < pos->accel_start_us &&
        pos->orig_user_target != NULL;

    if (can_rescue_dead_track) {
        pos->dead_track_recover.valid = 1;
        pos->dead_track_recover.orig_target = pos->orig_user_target;
        pos->dead_track_recover.orig_offset_mm = pos->orig_target_offset;
    } else {
        pos->dead_track_recover.valid = 0;
        pos->dead_track_recover.orig_target = NULL;
        pos->dead_track_recover.orig_offset_mm = 0;
    }

    track_node *guessed_end = pos->offroute_expected_sensor
                              ? pos->offroute_expected_sensor
                              : pos->pred.next_sensor;
    guessed_end = pos_release_keep_end(pos->cur_sensor, guessed_end);

    track_set_speed(pos->train_num, 0);
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM,
                                    guessed_end);

    pos->effective_v              = 0;
    pos->user_speed               = 0;
    pos->is_accelerating          = 0;
    pos->route_state              = TRAIN_STATE_DEAD_TRACK;
    pos->target_sensor            = NULL;
    pos->target_offset_mm         = 0;
    pos->dist_to_target_mm        = 0;
    pos->pending_target           = NULL;
    pos->pending_offset_mm        = 0;
    pos->queued_target            = NULL;
    pos->queued_offset_mm         = 0;
    pos->queued_valid             = 0;
    pos->orig_user_target         = NULL;
    pos->orig_target_offset       = 0;
    pos->find_pos_only            = 0;
    pos->midrev.active            = 0;
    pos->replan.next_us           = 0;
    pos->replan.retry_count       = 0;
    pos->force_offroute_on_next_sensor = 0;
    pos->dead_track_rescue_pending = 0;
    pos->offroute_valid           = 1;
    pos->offroute_expected_sensor = guessed_end;
    pos_clear_prediction(pos);
}

static void pos_revive_dead_track_for_current_hit(train_pos_t *pos) {
    if (!pos || pos->route_state != TRAIN_STATE_DEAD_TRACK ||
        !pos->dead_track_recover.valid || pos->dead_track_recover.orig_target == NULL) {
        return;
    }

    track_node *orig_target = pos->dead_track_recover.orig_target;
    int32_t orig_offset_mm = pos->dead_track_recover.orig_offset_mm;

    pos->dead_track_recover.valid = 0;
    pos->dead_track_recover.orig_target = NULL;
    pos->dead_track_recover.orig_offset_mm = 0;

    pos->route_state = TRAIN_STATE_ON_ROUTE;
    pos->target_sensor = orig_target;
    pos->target_offset_mm = orig_offset_mm;
    pos->pending_target = orig_target;
    pos->pending_offset_mm = orig_offset_mm;
    pos->orig_user_target = orig_target;
    pos->orig_target_offset = orig_offset_mm;
    pos->dist_to_target_mm = 0;
    pos->effective_v = 0;
    pos->user_speed = 0;
    pos->is_accelerating = 0;
    pos->route_path_count = 0;
    pos->route_path_cursor = 0;
    pos->route_rem_tick_us = 0;
    pos->offroute_valid = 0;
    pos->offroute_expected_sensor = NULL;
    pos->force_offroute_on_next_sensor = 1;
    pos->dead_track_rescue_pending = 1;
    pos_clear_prediction(pos);
}

/* Recompute pred.next_sensor and update the dead-track deadline from the
 * next predicted progress sensor only. In FIND_POS, timing is suppressed
 * (trigger_time and deadline stay 0). */
static void update_next_prediction(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    uint64_t dt_pred = 0;
    pos->pred.next_sensor = predict_next_sensor(pos, hit, &dt_pred);
    pos->pred.skipped_sensor_count = 0;

    if (pos->route_state == TRAIN_STATE_FIND_POS) {
        pos->pred.trigger_time      = 0;
        pos->dead_track_deadline_us = 0;
        return;
    }

    pos->pred.trigger_time = (dt_pred > 0) ? time_us + dt_pred : 0;
    if (pos->pred.next_sensor != NULL) KASSERT(pos->pred.trigger_time > time_us);
    else KASSERT(pos->pred.trigger_time == 0);
    if (dt_pred > 0) {
        pos->dead_track_deadline_us = time_us + DEAD_TRACK_TIMEOUT;
    } else {
        pos->dead_track_deadline_us = 0;
    }
}

/* ===== Per-train sensor FSM ===== */

static void handle_sensor(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    accel_update_v(pos, time_us);
    int took_alt_branch = hit_matches_alt_branch(pos, hit);
    int route_hit_cursor = route_path_hit_cursor(pos, hit);

    int was_predicted;
    update_sensor_stats(pos, hit, time_us, &was_predicted);

    pos->cur_sensor      = hit;
    pos->cur_sensor_time = time_us;
    track_node *keep_end = pos_release_keep_end(hit, NULL);

    /* Off-route: ON_ROUTE hit outside the remaining planned sensor path, even
     * if the node was still reserved only because of mutex safety padding. */
    if (pos->route_state == TRAIN_STATE_ON_ROUTE && pos->target_sensor != NULL) {
        if (pos->force_offroute_on_next_sensor ||
            took_alt_branch ||
            route_hit_cursor < 0) {
            pos->force_offroute_on_next_sensor = 0;
            track_node *expected_sensor = pos->pred.next_sensor;
            traffic_release_train_keep_body(pos->train_num, hit,
                                            TRAIN_BODY_MM, keep_end);
            pos->offroute_valid           = 1;
            pos->offroute_expected_sensor = expected_sensor;
            pos_clear_prediction(pos);
            track_set_speed(pos->train_num, 0);
            pos->route_state       = TRAIN_STATE_RECOVERY_STOPPING;
            pos->stopping_since_us = time_us;
            ui_mark_position_dirty();
            return;
        }
    }

    if (pos->route_state == TRAIN_STATE_UNKNOWN) {
        pos->route_state = TRAIN_STATE_KNOWN;
        ui_mark_position_dirty();
    }

    if (pos->route_state == TRAIN_STATE_DEAD_TRACK) {
        handle_dead_track_sensor(pos);
        return;
    }

    /* FIND_POS: stop on first sensor to acquire position. */
    if (pos->route_state == TRAIN_STATE_FIND_POS) {
        track_set_speed(pos->train_num, 0);
        pos->stopping_since_us = time_us;
        pos->route_state       = TRAIN_STATE_STOPPING_GOTO;
        ui_mark_position_dirty();
    }

    /* On every on-route sensor hit: advance route_path_cursor to hit and snap
     * dist_to_target_mm to the exact geometric distance from here to the target
     * along the planned path.  */
    if (pos->route_state == TRAIN_STATE_ON_ROUTE && pos->target_sensor) {
        if (route_hit_cursor >= 0) {
            pos->route_path_cursor = route_hit_cursor;
            int32_t pd = route_path_dist_from(pos->route_path, route_hit_cursor,
                                              pos->route_path_count);
            if (pd >= 0) {
                pos->dist_to_target_mm = pd + pos->target_offset_mm;
                if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
            }
            pos->route_rem_tick_us = time_us;
        }
    }

    if (!pos->offroute_valid) {
        pos->offroute_expected_sensor = NULL;
    }
    update_next_prediction(pos, hit, time_us);
    if (pos->route_state == TRAIN_STATE_ON_ROUTE) {
        traffic_refresh_route_reservation(pos->train_num, hit,
                                          pos->pred.next_sensor,
                                          pos->route_path,
                                          pos->route_path_cursor,
                                          pos->route_path_count);
    }

    ui_mark_position_dirty();
}

static void start_queued_goto_if_any(train_pos_t *pos) {
    if (!pos || !pos->queued_valid || !pos->queued_target) return;
    track_node *qt = pos->queued_target;
    int32_t     qo = pos->queued_offset_mm;
    pos->queued_target    = NULL;
    pos->queued_offset_mm = 0;
    pos->queued_valid     = 0;
        pos_goto(pos->train_num, qt, qo);
}

/* A midpoint reversal may stop at the reversal sensor and then block before the
 * second leg can be reserved. Collapse that half-complete midrev into a plain
 * WAIT on the final target*/
static void collapse_midrev_wait_target(train_pos_t *pos) {
    if (!pos || !pos->midrev.active) return;

    track_node *final_target = pos->midrev.final_target;
    int32_t final_offset = pos->midrev.final_offset;

    pos->target_sensor       = final_target;
    pos->target_offset_mm    = final_offset;
    pos->pending_target      = final_target;
    pos->pending_offset_mm   = final_offset;
    pos->orig_user_target    = final_target;
    pos->orig_target_offset  = final_offset;
    pos->dist_to_target_mm   = 0;

    pos->midrev.active       = 0;
    pos->midrev.sensor       = NULL;
    pos->midrev.final_target = NULL;
    pos->midrev.final_offset = 0;
    pos->midrev.sw_count     = 0;
    pos->midrev.dist_after   = 0;
    pos->midrev.path2_count  = 0;
}


/* ===== pos_on_tick sub-functions ===== */

/* Process all trains waiting for a resource replan (WAIT_RESOURCE).
 * Implements exponential backoff with jitter. */
void pos_replan_on_tick(uint64_t now_us) {
    static const int ORDER[6] = {13, 14, 15, 17, 18, 55};
    for (int wi = 0; wi < 6; wi++) {
        train_pos_t *pos = pos_get(ORDER[wi]);
        if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

        uint32_t generation = traffic_get_change_generation();
        int woke_on_change = 0;
        if (generation != pos->replan.seen_generation) {
            pos->replan.seen_generation = generation;
            pos->replan.retry_count = 0;
            woke_on_change = 1;
        }
        if (!woke_on_change &&
            pos->replan.next_us > 0 && now_us < pos->replan.next_us) {
            continue;
        }

        int backoff_exp = pos->replan.retry_count;
        if (backoff_exp > REPLAN_MAX_BACKOFF) backoff_exp = REPLAN_MAX_BACKOFF;
        uint64_t backoff_us = REPLAN_INTERVAL_US << backoff_exp;

        pos->replan.rand_state  = pos->replan.rand_state * 1664525u + 1013904223u;
        uint64_t jitter_us      = (pos->replan.rand_state >> 16) % (uint32_t)REPLAN_INTERVAL_US;
        pos->replan.next_us     = now_us + backoff_us + jitter_us;
        pos->replan.retry_count++;

        pos_restore_pending_target(pos);

        if (pos->pending_target == NULL) continue;

        int ok = pos_try_direct_goto(pos);
        KASSERT(ok);
    }
}

/* Resume the second leg of a mid-route reversal: preflight the second leg,
 * set switches before they become reserved, then reverse and restart.
 * If the stored second leg now needs changing a self-reserved switch,
 * discard it and replan from the current stopped position instead. */
static int handle_midrev_resume(train_pos_t *pos, uint64_t now_us) {
    route_plan_t second_leg_plan = {0};

    second_leg_plan.path_count = pos->midrev.path2_count;
    for (int j = 0; j < pos->midrev.path2_count; j++) {
        second_leg_plan.path_nodes[j] = pos->midrev.path2[j];
    }
    if (!traffic_can_reserve_plan(pos->train_num, &second_leg_plan)) {
        collapse_midrev_wait_target(pos);
        pos_enter_wait_resource(pos, now_us);
        return 0;
    }
    int sw_owner = pos_route_switch_blocker(pos->midrev.sw_nums, pos->midrev.sw_dirs,
                                            pos->midrev.sw_count, pos->train_num);
    if (sw_owner == pos->train_num) {
        track_node *final_target = pos->midrev.final_target;
        int32_t final_offset = pos->midrev.final_offset;

        traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                        TRAIN_BODY_MM,
                                        pos_release_keep_end(pos->cur_sensor,
                                                             pos->pred.next_sensor));
        pos->midrev.active = 0;
        pos->pending_target = final_target;
        pos->pending_offset_mm = final_offset;
        pos->orig_user_target = final_target;
        pos->orig_target_offset = final_offset;
        return pos_try_direct_goto(pos);
    }
    if (sw_owner >= 0) {
        collapse_midrev_wait_target(pos);
        pos_enter_wait_resource(pos, now_us);
        return 0;
    }
    if (!traffic_reserve_plan(pos->train_num, pos->cur_sensor, &second_leg_plan)) {
        collapse_midrev_wait_target(pos);
        pos_enter_wait_resource(pos, now_us);
        return 0;
    }
    if (pos->midrev.sw_count > 0) ui_mark_switches_dirty();

    pos->midrev.active = 0;

    track_reverse(pos->train_num);
    pos->going_forward = !pos->going_forward;
    if (pos->cur_sensor && pos->cur_sensor->reverse)
        pos->cur_sensor = pos->cur_sensor->reverse;

    pos_wait_switch_settle(pos->midrev.sw_count);

    pos->target_sensor    = pos->midrev.final_target;
    pos->target_offset_mm = pos->midrev.final_offset;

    /* Switch active path to the stored second leg. */
    pos->route_path_count = pos->midrev.path2_count;
    for (int j = 0; j < pos->midrev.path2_count; j++)
        pos->route_path[j] = pos->midrev.path2[j];
    pos->route_path_cursor = 0;

    
    int32_t pd2 = route_path_dist_from(pos->route_path, 0, pos->route_path_count);
    int32_t d2 = (pd2 >= 0) ? pd2 : pos->midrev.dist_after;
    pos->dist_to_target_mm = d2 + pos->midrev.final_offset;
    if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
    
    pos->route_rem_tick_us = now_us;

    pos_launch_at_goto_speed(pos, now_us);
    pos->stopping_since_us = 0;

    /* First expected sensor after reversal is the reversal sensor itself.
     * trigger_time=0 suppresses the time gate so WAIT_RESOURCE delays don't
     * cause it to be skipped. */
    pos->pred.next_sensor       = pos->cur_sensor;
    pos->pred.alt_sensor        = NULL;
    pos->pred.branch_node       = NULL;
    pos->pred.trigger_time      = 0;
    pos->pred.skipped_sensor_count = 0;
    pos_refresh_dead_track_deadline(pos, now_us);

    pos->route_state = TRAIN_STATE_ON_ROUTE;
    return 1;
}

/* Normal stop: clear route target, release reservations, check for queued goto. */
static void handle_normal_stop(train_pos_t *pos) {
    pos->route_state        = TRAIN_STATE_STOPPED;
    pos->orig_user_target   = NULL;
    pos->orig_target_offset = 0;
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM,
                                    pos_release_keep_end(pos->cur_sensor,
                                                         pos->pred.next_sensor));
    start_queued_goto_if_any(pos);
}

/* Handle STOPPING -> STOPPED transition (with mid-route reversal if active).
 * Returns 1 if the transition fired (caller should not process further states). */
static int tick_handle_stopping(train_pos_t *pos, uint64_t now_us) {
    if (pos->route_state == TRAIN_STATE_STOPPED) return 1;
    if (!brake_elapsed(pos, now_us)) return 1; 

    pos_save_ema_and_stop(pos);

    if (pos->midrev.active) {
        handle_midrev_resume(pos, now_us);  
    } else {
        handle_normal_stop(pos);
    }
    ui_mark_position_dirty();
    return 1;
}

static int do_recovery_replan(train_pos_t *pos) {
    pos_save_ema_and_stop(pos);
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM,
                                    pos_release_keep_end(pos->cur_sensor,
                                                         pos->pred.next_sensor));

    pos_restore_pending_target(pos);
    KASSERT(pos->pending_target != NULL);
    int ok = pos_try_direct_goto(pos);
    KASSERT(ok);
    return 1;
}

/* Handle RECOVERY_STOPPING -> replan.
 * Returns 1 when braking is complete and replan was attempted. */
static int tick_handle_recovery_stopping(train_pos_t *pos, uint64_t now_us) {
    if (pos->dead_track_rescue_pending && pos->effective_v == 0) {
        pos->dead_track_rescue_pending = 0;
        return do_recovery_replan(pos);
    }

    if (!brake_elapsed(pos, now_us)) return 0;

    pos->dead_track_rescue_pending = 0;
    return do_recovery_replan(pos);
}

/* Handle STOPPING_TR ->STOPPED.
 * Returns 1 when braking is complete. */
static int tick_handle_stopping_tr(train_pos_t *pos, uint64_t now_us) {
    if (!brake_elapsed(pos, now_us)) return 0;

    pos->route_state = TRAIN_STATE_STOPPED;
    pos->effective_v = 0;
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM,
                                    pos_release_keep_end(pos->cur_sensor,
                                                         pos->pred.next_sensor));
    start_queued_goto_if_any(pos);
    ui_mark_position_dirty();
    return 1;
}

/* Handle STOPPING_GOTO -> replan (or STOPPED if find_pos_only).
 * Returns 1 when braking is complete. */
static int tick_handle_stopping_goto(train_pos_t *pos, uint64_t now_us) {
    if (!brake_elapsed(pos, now_us)) return 0;

    pos_save_ema_and_stop(pos);
    if (pos->find_pos_only) {
        pos->find_pos_only = 0;
        pos->route_state   = TRAIN_STATE_STOPPED;
        traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                        TRAIN_BODY_MM,
                                        pos_release_keep_end(pos->cur_sensor,
                                                             pos->pred.next_sensor));
        ui_mark_position_dirty();
    } else {
        int ok = pos_try_direct_goto(pos);
        KASSERT(ok);
    }
    return 1;
}

/* Continuously estimate remaining distance to target and issue the stop
 * command when the train enters the braking window.
 * Returns 1 if the stop was issued (train is now STOPPING). */
static int tick_check_brake_point(train_pos_t *pos, uint64_t now_us) {
    if (pos->route_state != TRAIN_STATE_ON_ROUTE) return 0;
    if (!pos->target_sensor || !pos->cur_sensor || pos->effective_v <= 0) return 0;

    if (pos->route_rem_tick_us > 0 && now_us > pos->route_rem_tick_us) {
        uint64_t dt = now_us - pos->route_rem_tick_us;
        int32_t delta = (int32_t)((int64_t)pos->effective_v * (int64_t)dt / 1000000LL);
        pos->dist_to_target_mm -= delta;
        if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
    }
    pos->route_rem_tick_us = now_us;

    int32_t rem = pos->dist_to_target_mm;

    int32_t a = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
    if (a > 0) {
        int32_t d_brake = (int32_t)((int64_t)pos->effective_v * pos->effective_v / (2LL * a));
        int32_t d_early = d_brake + (int32_t)((int64_t)pos->effective_v * (int64_t)STOP_EARLY_US[pos->train_ind] / 1000000LL);

        if (pos->going_forward) {
            d_early += Train_Forward_Stop_Offset;
        } else {
            d_early += Train_Reverse_Stop_Offset;
        }

        if (rem <= d_early) {
            pos->route_state       = TRAIN_STATE_STOPPING;
            pos->stopping_since_us = now_us;
            track_set_speed(pos->train_num, 0);
            ui_mark_position_dirty();
            return 1;
        }
    }

    ui_mark_position_dirty();
    return 0;
}

/* Detect dead-track: no sensor fired before the deadline.
 * Returns 1 if dead-track was declared. */
static int tick_check_dead_track(train_pos_t *pos, uint64_t now_us) {
    if (pos->route_state != TRAIN_STATE_FIND_POS &&
        pos->route_state != TRAIN_STATE_ON_ROUTE &&
        pos->route_state != TRAIN_STATE_STOPPING) return 0;
    if (pos->dead_track_deadline_us == 0) return 0;
    if (now_us <= pos->dead_track_deadline_us) return 0;

    (void)now_us;
    enter_terminal_dead_track(pos);
    ui_mark_position_dirty();
    
    add_dead_train_to_retry(pos->train_num);
    Create(TRAIN_COURIER_PRIORITY, retry_dead_train_task);
    return 1;
}

/* Advance a stale prediction: if more than 2× the expected interval has
 * elapsed without a sensor, skip to the next predicted node and refresh the
 * dead-track deadline from that new predicted progress point.
 * Returns 1 if a skip was applied. */
static int tick_advance_prediction(train_pos_t *pos, uint64_t now_us) {
    if (pos->pred.trigger_time == 0 || pos->pred.next_sensor == NULL) return 0;
    if (pos->cur_sensor_time == 0) return 0;
    if (pos->pred.trigger_time <= pos->cur_sensor_time) return 0;
    if (now_us <= 2 * pos->pred.trigger_time - pos->cur_sensor_time) return 0;
    if (pos->pred.skipped_sensor_count >= 1) return 0;

    track_node *skipped = pos->pred.next_sensor;
    uint64_t prev_trigger_time = pos->pred.trigger_time;
    if (pos->offroute_valid == 0 && pos->offroute_expected_sensor == NULL) {
        pos->offroute_expected_sensor = skipped;
    }
    uint64_t dt = 0;
    pos->pred.next_sensor  = predict_next_sensor(pos, skipped, &dt);
    pos->pred.trigger_time = (dt > 0) ? (prev_trigger_time + dt) : 0;
    pos->pred.skipped_sensor_count = 1;
    pos_refresh_dead_track_deadline(pos, now_us);

    if (pos->target_sensor && pos->route_state == TRAIN_STATE_ON_ROUTE) {
        int32_t skip_dist = follow_dist(skipped,
            (pos->pred.next_sensor ? pos->pred.next_sensor : pos->target_sensor), 50);
        if (skip_dist > 0) {
            pos->dist_to_target_mm -= skip_dist;
            if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
        }
    }
    return 1;
}

/* ===== Public event API ===== */

void pos_on_sensor_trigger(uint16_t sensor_id, uint64_t time_us) {
    int track_idx = (int)sensor_id - 1;
    if (track_idx < 0 || track_idx >= TRACK_MAX) return;

    track_node *hit = &g_track[track_idx];
    if (hit->type != NODE_SENSOR) return;

    traffic_attr_result_t attr = traffic_attribute_sensor(hit, time_us);
    if (!attr.owner) return;

    train_pos_t *owner = attr.owner;
    if (attr.revive_dead_track) {
        pos_revive_dead_track_for_current_hit(owner);
    }

    /* If the train took the alt-direction branch, correct the stored switch state. */
    if (hit_matches_alt_branch(owner, hit) && owner->pred.branch_node != NULL) {
        int sw_idx = track_switch_to_index(owner->pred.branch_node->num);
        if (sw_idx >= 0) {
            char stored = track_get_switch_state()[sw_idx].state;
            char actual = (stored == 'S') ? 'C' : 'S';
            track_update_switch(owner->pred.branch_node->num, actual);
            ui_mark_switches_dirty();
        }
    }

    handle_sensor(owner, hit, time_us);
}

void pos_on_tick(uint64_t now_us) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (pos->train_num < 0) continue;
        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) continue;
        if (pos->route_state == TRAIN_STATE_DEAD_TRACK) continue;

        /* Update kinematic velocity every tick for accelerating trains.
         * For stopping/stopped states, the train is no longer accelerating —
         * clear the flag */
        if (pos->is_accelerating) {
            if (pos->route_state == TRAIN_STATE_ON_ROUTE ||
                pos->route_state == TRAIN_STATE_FIND_POS  ||
                pos->route_state == TRAIN_STATE_KNOWN) {
                accel_update_v(pos, now_us);
            } else {
                pos->is_accelerating = 0;
            }
        }

        if (pos->route_state == TRAIN_STATE_RECOVERY_STOPPING) {
            tick_handle_recovery_stopping(pos, now_us);
            continue;
        }
        if (pos->route_state == TRAIN_STATE_STOPPING_TR) {
            tick_handle_stopping_tr(pos, now_us);
            continue;
        }
        if (pos->route_state == TRAIN_STATE_STOPPING_GOTO) {
            tick_handle_stopping_goto(pos, now_us);
            continue;
        }
        if (pos->route_state == TRAIN_STATE_STOPPING) {
            if (tick_check_dead_track(pos, now_us)) continue;
            tick_handle_stopping(pos, now_us);
            continue;
        }
        if (pos->route_state == TRAIN_STATE_STOPPED) {
            tick_handle_stopping(pos, now_us);
            continue;
        }

        if (tick_check_dead_track(pos, now_us))  continue;
        if (tick_check_brake_point(pos, now_us)) continue;
        tick_advance_prediction(pos, now_us);
    }
}
