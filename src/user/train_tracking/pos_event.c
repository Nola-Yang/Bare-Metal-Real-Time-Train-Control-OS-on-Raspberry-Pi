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
 *       infer_direction               (UNKNOWN/LOOP_FIND_DIR direction detect)
 *       handle_dead_track_sensor      (dead-track recovery on sensor fire)
 *       update_next_prediction        (predict next sensor + deadline)
 *
 *   pos_on_tick
 *     tick_replan_waiting_trains      (WAIT_RESOURCE backoff replan)
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
#include <stdint.h>
#include <stddef.h>

/* Stop command lead time for overshoot compensation (microseconds). */
#ifdef TRACK_D
    uint64_t STOP_EARLY_US[MAX_PHYSICAL_TRAINS] = {930000ULL, 930000ULL, 930000ULL, 930000ULL, 930000ULL};
#else
    uint64_t STOP_EARLY_US[MAX_PHYSICAL_TRAINS] = {1000000ULL, 1000000ULL, 1000000ULL, 1000000ULL, 1000000ULL};
#endif


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


/* ===== handle_sensor sub-functions ===== */

/* Update EMA speed, warmup, and prediction-error stats on a sensor hit.
 * Sets *was_predicted and *is_skip for use by the FSM. */
static void update_sensor_stats(train_pos_t *pos, track_node *hit,
                                uint64_t time_us,
                                int *was_predicted, int *is_skip) {
    *was_predicted = (pos->pred.next_sensor == hit);
    *is_skip = 0;
    if (pos->pred.next_sensor && !*was_predicted) {
        if (follow_dist(pos->pred.next_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS) >= 0)
            *is_skip = 1;
    }

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

/* Infer travel direction from two consecutive sensor hits.
 * Called only in UNKNOWN / LOOP_FIND_DIR when prev_sensor is known. */
static void infer_direction(train_pos_t *pos, track_node *prev, track_node *hit) {
    if (follow_dist(prev, hit, 20) >= 0) {
        pos->going_forward = 1;
    } else if (prev->reverse != NULL && follow_dist(prev->reverse, hit, 20) >= 0) {
        pos->going_forward = 0;
    } else {
        KASSERT(0 && "Two consecutive sensors with no path between them?");
    }
    pos->position_known = 1;
}

/* Handle a sensor hit while in DEAD_TRACK state.
 * Clears the dead-track flag, restores pending target, and replans.
 * Returns 1 if this function handled the event (caller should return). */
static int handle_dead_track_sensor(train_pos_t *pos) {
    pos->offroute_valid = 0;
    pos->effective_v    = 0;
    traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);

    pos_restore_pending_target(pos);

    if (pos->pending_target == NULL) {
        pos->find_dir_only = 0;
        pos->route_state   = TRAIN_STATE_STOPPED;
        ui_mark_position_dirty();
        return 1;
    }
    int ok = pos_try_direct_goto(pos);
    KASSERT(ok);
    ui_mark_position_dirty();
    return 1;
}

/* Recompute pred.next_sensor and update the dead-track deadline.
 * In LOOP_FIND_DIR, timing is suppressed (trigger_time and deadline stay 0). */
static void update_next_prediction(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    uint64_t dt_pred = 0;
    pos->pred.next_sensor = predict_next_sensor(pos, hit, &dt_pred);

    if (pos->route_state == TRAIN_STATE_LOOP_FIND_DIR) {
        pos->pred.trigger_time      = 0;
        pos->dead_track_deadline_us = 0;
        return;
    }

    pos->pred.trigger_time = (dt_pred > 0) ? time_us + dt_pred : 0;
    if (pos->pred.next_sensor != NULL && dt_pred > 0) {
        uint64_t T2 = 0;
        predict_next_sensor(pos, pos->pred.next_sensor, &T2);
        pos->dead_track_deadline_us =
            time_us + DEAD_TRACK_DEADLINE_MULTIPLIER * (dt_pred + T2);
    } else {
        pos->dead_track_deadline_us = 0;
    }
}

/* ===== Per-train sensor FSM ===== */

static void handle_sensor(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    int was_predicted, is_skip;
    update_sensor_stats(pos, hit, time_us, &was_predicted, &is_skip);

    track_node *prev_sensor = pos->cur_sensor;
    pos->cur_sensor      = hit;
    pos->cur_sensor_time = time_us;

    if (prev_sensor && prev_sensor != hit)
        traffic_release_passed(pos->train_num, prev_sensor, hit);

    /* Off-route: ON_ROUTE hit outside our reservation -> stop and replan. */
    if (pos->route_state == TRAIN_STATE_ON_ROUTE && pos->target_sensor != NULL) {
        if (!traffic_is_reserved_by(hit, pos->train_num)) {
            pos->offroute_valid           = 1;
            pos->offroute_expected_sensor = pos->pred.next_sensor;
            pos_clear_prediction(pos);
            track_set_speed(pos->train_num, 0);
            pos->route_state       = TRAIN_STATE_RECOVERY_STOPPING;
            pos->stopping_since_us = time_us;
            ui_mark_position_dirty();
            return;
        }
    }

    /* Direction inference for UNKNOWN / LOOP_FIND_DIR. */
    if ((pos->route_state == TRAIN_STATE_UNKNOWN ||
         pos->route_state == TRAIN_STATE_LOOP_FIND_DIR) && prev_sensor != NULL) {
        infer_direction(pos, prev_sensor, hit);
    }

    if (pos->route_state == TRAIN_STATE_UNKNOWN && prev_sensor != NULL) {
        pos->route_state = TRAIN_STATE_KNOWN;
        ui_mark_position_dirty();
    }

    if (pos->route_state == TRAIN_STATE_DEAD_TRACK) {
        handle_dead_track_sensor(pos);
        return;
    }

    /* LOOP_FIND_DIR: stop after direction is confirmed (second sensor). */
    if (pos->route_state == TRAIN_STATE_LOOP_FIND_DIR && prev_sensor != NULL) {
        track_set_speed(pos->train_num, 0);
        pos->stopping_since_us = time_us;
        pos->route_state       = TRAIN_STATE_STOPPING_GOTO;
        ui_mark_position_dirty();
    }

    update_next_prediction(pos, hit, time_us);
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


/* ===== pos_on_tick sub-functions ===== */

/* Process all trains waiting for a resource replan (WAIT_RESOURCE).
 * Implements exponential backoff with jitter. */
static void tick_replan_waiting_trains(uint64_t now_us) {
    static const int ORDER[6] = {13, 14, 15, 17, 18, 55};
    for (int wi = 0; wi < 6; wi++) {
        train_pos_t *pos = pos_get(ORDER[wi]);
        if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        if (pos->replan.next_us > 0 && now_us < pos->replan.next_us) continue;

        int backoff_exp = pos->replan.retry_count;
        if (backoff_exp > REPLAN_MAX_BACKOFF) backoff_exp = REPLAN_MAX_BACKOFF;
        uint64_t backoff_us = REPLAN_INTERVAL_US << backoff_exp;

        pos->replan.rand_state  = pos->replan.rand_state * 1664525u + 1013904223u;
        uint64_t jitter_us      = (pos->replan.rand_state >> 16) % (uint32_t)REPLAN_INTERVAL_US;
        pos->replan.next_us     = now_us + backoff_us + jitter_us;
        pos->replan.retry_count++;

        pos_restore_pending_target(pos);
        if (pos->pending_target != NULL) {
            int ok = pos_try_direct_goto(pos);
            KASSERT(ok);
        }
    }
}

/* Resume the second leg of a mid-route reversal: reverse the train, apply
 * second-leg switches, restore the second-leg path, and restart at GOTO speed.
 * Returns 0 if a switch was blocked (enters WAIT_RESOURCE); 1 on success. */
static int handle_midrev_resume(train_pos_t *pos, uint64_t now_us) {
    pos->midrev.active = 0;

    track_reverse(pos->train_num);
    pos->going_forward = !pos->going_forward;
    if (pos->cur_sensor && pos->cur_sensor->reverse)
        pos->cur_sensor = pos->cur_sensor->reverse;

    for (int j = pos->midrev.sw_count - 1; j >= 0; j--) {
        if (traffic_can_set_switch(pos->midrev.sw_nums[j], pos->train_num) >= 0) {
            pos_enter_wait_resource(pos, now_us);
            return 0;
        }
    }

    for (int j = pos->midrev.sw_count - 1; j >= 0; j--) {
        track_set_switch(pos->midrev.sw_nums[j], pos->midrev.sw_dirs[j]);
        track_update_switch(pos->midrev.sw_nums[j], pos->midrev.sw_dirs[j]);
    }
    resend_unreliable_switches(pos->midrev.sw_nums, pos->midrev.sw_dirs, pos->midrev.sw_count);
    if (pos->midrev.sw_count > 0) ui_mark_switches_dirty();

    pos->target_sensor    = pos->midrev.final_target;
    pos->target_offset_mm = pos->midrev.final_offset;

    /* Switch active path to the stored second leg. */
    pos->route_path_count = pos->midrev.path2_count;
    for (int j = 0; j < pos->midrev.path2_count; j++)
        pos->route_path[j] = pos->midrev.path2[j];
    pos->route_path_cursor = 0;

    int32_t pd2 = route_path_dist_from(pos->route_path, 0, pos->route_path_count);
    if (pd2 >= 0) {
        pos->route_dist_anchor_mm = pd2 + pos->midrev.final_offset;
    } else {
        pos->route_dist_anchor_mm = pos->midrev.dist_after + pos->midrev.final_offset;
    }
    if (pos->route_dist_anchor_mm < 0) pos->route_dist_anchor_mm = 0;
    pos->dist_to_target_mm = pos->route_dist_anchor_mm;

    pos_launch_at_goto_speed(pos, now_us);
    pos->stopping_since_us = 0;

    /* First expected sensor after reversal is the reversal sensor itself.
     * trigger_time=0 suppresses the time gate so WAIT_RESOURCE delays don't
     * cause it to be skipped. */
    pos->pred.next_sensor       = pos->cur_sensor;
    pos->pred.alt_sensor        = NULL;
    pos->pred.branch_node       = NULL;
    pos->pred.trigger_time      = 0;
    pos->dead_track_deadline_us = 0;

    pos->route_state = TRAIN_STATE_ON_ROUTE;
    return 1;
}

/* Normal stop: clear route target, release reservations, check for queued goto. */
static void handle_normal_stop(train_pos_t *pos) {
    pos->route_state        = TRAIN_STATE_STOPPED;
    pos->orig_user_target   = NULL;
    pos->orig_target_offset = 0;
    /* Keep the stretch from cur_sensor to target_sensor reserved —
     * the train is physically somewhere in that segment. */
    if (pos->target_sensor != NULL && pos->target_sensor != pos->cur_sensor) {
        traffic_release_train_keep_range(pos->train_num,
                                         pos->cur_sensor, pos->target_sensor);
    } else {
        traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);
    }
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

/* Handle RECOVERY_STOPPING -> replan.
 * Returns 1 when braking is complete and replan was attempted. */
static int tick_handle_recovery_stopping(train_pos_t *pos, uint64_t now_us) {
    if (!brake_elapsed(pos, now_us)) return 0;

    pos_save_ema_and_stop(pos);
    traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);

    pos_restore_pending_target(pos);
    KASSERT(pos->pending_target != NULL);
    int ok = pos_try_direct_goto(pos);
    KASSERT(ok);
    return 1;
}

/* Handle STOPPING_TR ->STOPPED.
 * Returns 1 when braking is complete. */
static int tick_handle_stopping_tr(train_pos_t *pos, uint64_t now_us) {
    if (!brake_elapsed(pos, now_us)) return 0;

    pos->route_state = TRAIN_STATE_STOPPED;
    pos->effective_v = 0;
    traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);
    start_queued_goto_if_any(pos);
    ui_mark_position_dirty();
    return 1;
}

/* Handle STOPPING_GOTO -> replan (or STOPPED if find_dir_only).
 * Returns 1 when braking is complete. */
static int tick_handle_stopping_goto(train_pos_t *pos, uint64_t now_us) {
    if (!brake_elapsed(pos, now_us)) return 0;

    pos_save_ema_and_stop(pos);
    if (pos->find_dir_only) {
        pos->find_dir_only = 0;
        pos->route_state   = TRAIN_STATE_STOPPED;
        traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);
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

    int32_t dist_from_cur = (pos->route_dist_anchor_mm > 0)
        ? pos->route_dist_anchor_mm
        : follow_dist(pos->cur_sensor, pos->target_sensor, 150) + pos->target_offset_mm;

    if (dist_from_cur < 0) return 0;

    uint64_t elapsed = now_us - pos->cur_sensor_time;
    int32_t traveled = (int32_t)((int64_t)pos->effective_v * (int64_t)elapsed / 1000000LL);
    int32_t rem = dist_from_cur - traveled;
    if (rem < 0) rem = 0;
    pos->dist_to_target_mm = rem;

    int32_t a = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
    if (a > 0) {
        int32_t d_brake = (int32_t)((int64_t)pos->effective_v * pos->effective_v / (2LL * a));
        int32_t d_early = d_brake + (int32_t)(
            (int64_t)pos->effective_v * (int64_t)STOP_EARLY_US[pos->train_ind] / 1000000LL);
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
    if (pos->route_state != TRAIN_STATE_LOOP_FIND_DIR &&
        pos->route_state != TRAIN_STATE_ON_ROUTE) return 0;
    if (pos->dead_track_deadline_us == 0) return 0;
    if (now_us <= pos->dead_track_deadline_us) return 0;

    pos->effective_v              = 0;
    pos->route_state              = TRAIN_STATE_DEAD_TRACK;
    pos->target_sensor            = NULL;
    pos->target_offset_mm         = 0;
    pos->offroute_valid           = 1;
    pos->offroute_expected_sensor = pos->pred.next_sensor;
    pos_clear_prediction(pos);
    traffic_release_train(pos->train_num);
    ui_mark_position_dirty();
    return 1;
}

/* Advance a stale prediction: if more than 2× the expected interval has
 * elapsed without a sensor, skip to the next predicted node. */
static void tick_advance_prediction(train_pos_t *pos, uint64_t now_us) {
    if (pos->pred.trigger_time == 0 || pos->pred.next_sensor == NULL) return;
    if (pos->cur_sensor_time == 0) return;
    if (pos->pred.trigger_time <= pos->cur_sensor_time) return;
    if (now_us <= 2 * pos->pred.trigger_time - pos->cur_sensor_time) return;

    track_node *skipped = pos->pred.next_sensor;
    uint64_t dt = 0;
    pos->pred.next_sensor  = predict_next_sensor(pos, skipped, &dt);
    pos->pred.trigger_time = now_us + dt;

    if (pos->target_sensor && pos->route_state == TRAIN_STATE_ON_ROUTE) {
        int32_t skip_dist = follow_dist(skipped,
            (pos->pred.next_sensor ? pos->pred.next_sensor : pos->target_sensor), 50);
        if (skip_dist > 0) {
            pos->dist_to_target_mm -= skip_dist;
            if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
        }
    }
}


/* ===== Public event API ===== */

void pos_on_sensor_trigger(uint16_t sensor_id, uint64_t time_us) {
    int track_idx = (int)sensor_id - 1;
    if (track_idx < 0 || track_idx >= TRACK_MAX) return;

    track_node *hit = &g_track[track_idx];
    if (hit->type != NODE_SENSOR) return;

    train_pos_t *owner = traffic_attribute_sensor(hit, time_us);
    if (!owner) return;

    /* If the train took the alt-direction branch, correct the stored switch state. */
    if (owner->pred.alt_sensor == hit && owner->pred.branch_node != NULL) {
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
    tick_replan_waiting_trains(now_us);

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (pos->train_num < 0) continue;
        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) continue;
        if (pos->route_state == TRAIN_STATE_DEAD_TRACK) continue;

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
        if (pos->route_state == TRAIN_STATE_STOPPING ||
            pos->route_state == TRAIN_STATE_STOPPED) {
            tick_handle_stopping(pos, now_us);
            continue;
        }

        if (tick_check_brake_point(pos, now_us)) continue;
        if (tick_check_dead_track(pos, now_us))  continue;
        tick_advance_prediction(pos, now_us);
    }
}
