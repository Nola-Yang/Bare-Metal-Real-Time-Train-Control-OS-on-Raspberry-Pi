/*
 * pos_event.c — position-tracking event handlers
 *
 * Handles two event sources that drive the train position FSM:
 *   pos_on_sensor_trigger()  — a sensor fired (CAN RX interrupt path)
 *   pos_on_tick()            — 10 ms periodic tick (dead-reckoning + timeouts)
 *
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


/* Estimate time (us) for a braking train to reach a full stop.
 * Uses v * 1.5s/a_eff; falls back to 1s default. */
static uint64_t calc_brake_us(train_pos_t *pos) {
    int32_t decel = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
    if (pos->effective_v > 0 && decel > 0)
        return (uint64_t)pos->effective_v * 1500000ULL / (uint64_t)decel;
    return 1000000ULL;
}

/* ===== Sensor-hit statistics helper ===== */

/* Compute skip/prediction/EMA stats for a sensor hit.
 * Writes *out_was_predicted and *out_is_skip for use in FSM transitions. */
static void update_sensor_stats(train_pos_t *pos, track_node *hit,
                                uint64_t time_us,
                                int *out_was_predicted, int *out_is_skip) {

    /* Skip detection: sensor reachable from predicted next -> missed intermediates */
    *out_was_predicted = (pos->pred_next_sensor == hit);
    *out_is_skip = 0;
    if (pos->pred_next_sensor && !*out_was_predicted) {
        if (follow_dist(pos->pred_next_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS) >= 0) {
            *out_is_skip = 1;
        }
    }

    /* Speed-change warm-up: suppress EMA calibration for the
     * first 400 mm after any speed adjustment. */
    if (pos->speed_warmup_mm > 0 && pos->cur_sensor != NULL) {
        int32_t seg = follow_dist(pos->cur_sensor, hit, 100);
        if (seg > 0) {
            pos->speed_warmup_mm -= seg;
            if (pos->speed_warmup_mm < 0) pos->speed_warmup_mm = 0;
        }
    }
    int in_warmup = (pos->speed_warmup_mm > 0);

    /* Prediction error accounting */
    if (*out_was_predicted && pos->pred_trigger_time > 0) {
        pos->last_time_err_us =
            (int64_t)time_us - (int64_t)pos->pred_trigger_time;
        {
            int64_t derr = (int64_t)pos->effective_v
                           * pos->last_time_err_us / 1000000LL;
            if (derr >  99999) derr =  99999;
            if (derr < -99999) derr = -99999;
            pos->last_dist_err_mm = (int32_t)derr;
        }
        ui_mark_prediction_dirty();
    }

    /* EMA speed update
     *
     * ON_ROUTE: steady cruise to target; primary source of calibration.
     *
     * All other states are excluded:
     *   UNKNOWN / KNOWN
     *   LOOP_FIND_DIR            — train still accelerating from rest
     *   STOPPING* / RECOVERY_*  — decelerating
     *
     * Also require dt > 10 ms to reject sensor noise spikes.
     *
     * Require *out_was_predicted:
     */
    {
        train_route_state_t st = pos->route_state;
        int ema_valid = !in_warmup &&
                        (st == TRAIN_STATE_ON_ROUTE);
        if (ema_valid && *out_was_predicted && pos->cur_sensor && pos->effective_v > 0) {
            int32_t meas_dist = follow_dist(pos->cur_sensor, hit, 100);
            uint64_t dt = time_us - pos->cur_sensor_time;
            if (dt > 10000 && meas_dist > 0) {
                int32_t meas_v =
                    (int32_t)((int64_t)meas_dist * 1000000LL / (int64_t)dt);
                
                if (meas_v > 1800) meas_v = 1800;
                pos->effective_v = (int32_t)((7LL * pos->effective_v + meas_v) / 8);
            }
        }
    }
}

/* ===== Per-train sensor FSM ===== */

static void handle_sensor(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    int b_was_predicted, b_is_skip;
    update_sensor_stats(pos, hit, time_us, &b_was_predicted, &b_is_skip);

    track_node *prev_sensor = pos->cur_sensor;
    pos->cur_sensor      = hit;
    pos->cur_sensor_time = time_us;

    if (prev_sensor && prev_sensor != hit) {
        traffic_release_passed(pos->train_num, prev_sensor, hit);
    }

    /* Reservation-based off-route detection.
     * If ON_ROUTE and the sensor we just hit is not in our reservation, the
     * train has diverged from its planned path.  Stop immediately and replan. */
    if (pos->route_state == TRAIN_STATE_ON_ROUTE && pos->target_sensor != NULL) {
        if (!traffic_is_reserved_by(hit, pos->train_num)) {
            pos->offroute_valid           = 1;
            pos->offroute_expected_sensor = pos->pred_next_sensor;
            pos->pred_next_sensor         = NULL;
            pos->pred_alt_sensor          = NULL;
            pos->pred_trigger_time        = 0;
            track_set_speed(pos->train_num, 0);
            pos->route_state       = TRAIN_STATE_RECOVERY_STOPPING;
            pos->stopping_since_us = time_us;
            ui_mark_position_dirty();
            return;
        }
    }

    if ((pos->route_state == TRAIN_STATE_UNKNOWN ||
         pos->route_state == TRAIN_STATE_LOOP_FIND_DIR) &&
        prev_sensor != NULL) {
        if (follow_dist(prev_sensor, hit, 20) >= 0) {
            pos->going_forward = 1;
        } else if (prev_sensor->reverse != NULL &&
                   follow_dist(prev_sensor->reverse, hit, 20) >= 0) {
            pos->going_forward = 0;
        } else {
            KASSERT(0 && "Two consecutive sensors with no path between them?");
        }
        pos->position_known = 1;
    }

    // Infer direction for trains running without goto commands.
    if (pos->route_state == TRAIN_STATE_UNKNOWN && prev_sensor != NULL) {
        /* UNKNOWN → KNOWN: position and direction known
         * Only transition if the train is running via a tr command (user_speed > 0).
        */
        pos->route_state = TRAIN_STATE_KNOWN;
        ui_mark_position_dirty();

    }

    /* Dead track recovery: a sensor fired — position now known. Replan. */
    if (pos->route_state == TRAIN_STATE_DEAD_TRACK) {
        pos->offroute_valid = 0;
        pos->effective_v = 0;
        traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);
        if (pos->pending_target == NULL && pos->orig_user_target != NULL) {
            pos->pending_target    = pos->orig_user_target;
            pos->pending_offset_mm = pos->orig_target_offset;
        }
        if (pos->pending_target == NULL) {
            /* find_dir_only or no target — direction now known; just stop. */
            pos->find_dir_only = 0;
            pos->route_state   = TRAIN_STATE_STOPPED;
            ui_mark_position_dirty();
            return;
        }
        int dead_ok = pos_try_direct_goto(pos);
        KASSERT(dead_ok);
        ui_mark_position_dirty();
        return;
    }

    /* LOOP_FIND_DIR: running to acquire position.
     * First sensor (prev_sensor==NULL): record cur_sensor, compute pred_next, keep running.
     * Second sensor (prev_sensor!=NULL): direction known, pred_next valid -> stop. */
    if (pos->route_state == TRAIN_STATE_LOOP_FIND_DIR && prev_sensor != NULL) {
        track_set_speed(pos->train_num, 0);
        pos->stopping_since_us = time_us;
        pos->route_state       = TRAIN_STATE_STOPPING_GOTO;
        ui_mark_position_dirty();
    }

    /* Predict next sensor.
     * Keep pred_next_sensor for correct
     * sensor attribution on the second hit, but suppress pred_trigger_time
     * and dead_track_deadline so timing-based logic is not misled. */
    uint64_t dt_pred = 0;
    pos->pred_next_sensor  = predict_next_sensor(pos, hit, &dt_pred);
    if (pos->route_state == TRAIN_STATE_LOOP_FIND_DIR) {
        pos->pred_trigger_time      = 0;
        pos->dead_track_deadline_us = 0;
    } else {
        pos->pred_trigger_time = (dt_pred > 0) ? time_us + dt_pred : 0;
        if (pos->pred_next_sensor != NULL && dt_pred > 0) {
            uint64_t T2 = 0;
            predict_next_sensor(pos, pos->pred_next_sensor, &T2);
            pos->dead_track_deadline_us =
                time_us + DEAD_TRACK_DEADLINE_MULTIPLIER * (dt_pred + T2);
        } else {
            pos->dead_track_deadline_us = 0;
        }
    }

    /* Stop-at logic */
    if (pos->target_sensor &&
        pos->route_state == TRAIN_STATE_ON_ROUTE) {

        int32_t rem = follow_dist(hit, pos->target_sensor, 150);
        if (rem >= 0) {
            rem += pos->target_offset_mm;
            if (rem < 0) rem = 0;
            pos->dist_to_target_mm = rem;

            int user_spd = pos->user_speed;
            int32_t a    = speed_table_get_decel(pos->train_ind, user_spd, pos->target_sensor);

            if (a > 0) {
                int32_t d_brake = (int32_t)((int64_t)pos->effective_v
                                            * pos->effective_v / (2LL * a));
                /* Issue stop early to compensate overshoot. */
                int32_t d_early = d_brake + (int32_t)(
                    (int64_t)pos->effective_v * (int64_t)STOP_EARLY_US[pos->train_ind] / 1000000LL);
                if (rem <= d_early) {
                    pos->route_state       = TRAIN_STATE_STOPPING;
                    pos->stopping_since_us = time_us;
                    track_set_speed(pos->train_num, 0);
                }
            }
        }
    }

    ui_mark_position_dirty();
}

static void start_queued_goto_if_any(train_pos_t *pos) {
    if (!pos || !pos->queued_valid || !pos->queued_target) return;
    track_node *qt = pos->queued_target;
    int32_t qo = pos->queued_offset_mm;
    pos->queued_target = NULL;
    pos->queued_offset_mm = 0;
    pos->queued_valid = 0;
    pos_goto(pos->train_num, qt, qo);
}

/* ===== Public event API ===== */

void pos_on_sensor_trigger(uint16_t sensor_id, uint64_t time_us) {
    int track_idx = (int)sensor_id - 1;
    if (track_idx < 0 || track_idx >= TRACK_MAX) return;

    track_node *hit = &g_track[track_idx];
    if (hit->type != NODE_SENSOR) return;

    train_pos_t *owner = traffic_attribute_sensor(hit, time_us);
    if (!owner) return;

    /* If train took the alt-direction branch, correct the stored switch state */
    if (owner->pred_alt_sensor == hit && owner->pred_branch_node != NULL) {
        int sw_idx = track_switch_to_index(owner->pred_branch_node->num);
        if (sw_idx >= 0) {
            char stored = track_get_switch_state()[sw_idx].state;
            char actual = (stored == 'S') ? 'C' : 'S';
            track_update_switch(owner->pred_branch_node->num, actual);
            ui_mark_switches_dirty();
        }
    }

    handle_sensor(owner, hit, time_us);
}

void pos_on_tick(uint64_t now_us) {
    static const int WAIT_REPLAN_ORDER[6] = {13, 14, 15, 17, 18, 55};
    for (int wi = 0; wi < 6; wi++) {
        train_pos_t *pos = pos_get(WAIT_REPLAN_ORDER[wi]);
        if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        if (pos->next_replan_us > 0 && now_us < pos->next_replan_us) continue;


        int backoff_exp = pos->replan_retry_count;
        if (backoff_exp > REPLAN_MAX_BACKOFF) backoff_exp = REPLAN_MAX_BACKOFF;
        uint64_t backoff_us = REPLAN_INTERVAL_US << backoff_exp;

        pos->replan_rand_state = pos->replan_rand_state * 1664525u + 1013904223u;
        uint64_t jitter_us = (pos->replan_rand_state >> 16) % (uint32_t)REPLAN_INTERVAL_US;

        pos->next_replan_us = now_us + backoff_us + jitter_us;
        pos->replan_retry_count++;

        if (pos->pending_target == NULL && pos->orig_user_target != NULL) {
            pos->pending_target = pos->orig_user_target;
            pos->pending_offset_mm = pos->orig_target_offset;
        }
        if (pos->pending_target != NULL) {
            int ok = pos_try_direct_goto(pos);
            KASSERT(ok);
        }
    }

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (pos->train_num < 0) continue;

        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) continue;

        /* train physically stopped; no estimation until sensor fires. */
        if (pos->route_state == TRAIN_STATE_DEAD_TRACK) continue;

        /* Once braking is complete, replan directly to original target. */
        if (pos->route_state == TRAIN_STATE_RECOVERY_STOPPING) {
            uint64_t brake_us = calc_brake_us(pos);
            if (now_us < pos->stopping_since_us + brake_us) continue;

            if (pos->user_speed > 0 && pos->user_speed <= 14)
                pos->cached_v[pos->user_speed] = pos->effective_v;
            pos->effective_v = 0;

            traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);

            /* Try replanning directly to original target before falling back
             * to the loop-stabilization recovery path. */
            if (pos->pending_target == NULL && pos->orig_user_target != NULL) {
                pos->pending_target    = pos->orig_user_target;
                pos->pending_offset_mm = pos->orig_target_offset;
            }
            KASSERT(pos->pending_target != NULL);
            int rec_ok = pos_try_direct_goto(pos);
            KASSERT(rec_ok);
            continue;
        }

        /* tr 0 command sent; wait for physical stop -> STOPPED.
         * Keep effective_v intact until confirmed stopped for accurate estimate. */
        if (pos->route_state == TRAIN_STATE_STOPPING_TR) {
            if (now_us >= pos->stopping_since_us + calc_brake_us(pos)) {
                pos->route_state = TRAIN_STATE_STOPPED;
                pos->effective_v = 0;
                traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);
                start_queued_goto_if_any(pos);
                ui_mark_position_dirty();
            }
            continue;
        }

        /* goto was issued while the train was running; stop
         * command has been sent.  Once physically stopped, drive to loop. */
        if (pos->route_state == TRAIN_STATE_STOPPING_GOTO) {
            if (now_us >= pos->stopping_since_us + calc_brake_us(pos)) {
                if (pos->user_speed > 0 && pos->user_speed <= 14)
                    pos->cached_v[pos->user_speed] = pos->effective_v;
                pos->effective_v = 0;
                if (pos->find_dir_only) {
                    pos->find_dir_only = 0;
                    pos->route_state   = TRAIN_STATE_STOPPED;
                    traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);
                    ui_mark_position_dirty();
                } else {
                    int goto_ok = pos_try_direct_goto(pos);
                    KASSERT(goto_ok);
                }
            }
            continue;
        }

        /* STOPPING -> STOPPED transition
         * While braking/stopped, skip the sensor-timeout logic below. */
        if (pos->route_state == TRAIN_STATE_STOPPING ||
            pos->route_state == TRAIN_STATE_STOPPED) {
            if (pos->route_state == TRAIN_STATE_STOPPING &&
                pos->stopping_since_us > 0) {
                if (now_us >= pos->stopping_since_us + calc_brake_us(pos)) {
                    if (pos->user_speed > 0 && pos->user_speed <= 14)
                        pos->cached_v[pos->user_speed] = pos->effective_v;
                    pos->effective_v = 0;

                    if (pos->midrev_active) {
                        pos->midrev_active = 0;

                        track_reverse(pos->train_num);
                        pos->going_forward = !pos->going_forward;
                        if (pos->cur_sensor && pos->cur_sensor->reverse)
                            pos->cur_sensor = pos->cur_sensor->reverse;

                        int switch_blocked = 0;
                        for (int j = pos->midrev_sw_count - 1; j >= 0; j--) {
                            int owner = traffic_can_set_switch(pos->midrev_sw_nums[j],
                                                               pos->train_num);
                            if (owner >= 0) {
                                switch_blocked = 1;
                                break;
                            }
                        }
                        if (switch_blocked) {
                            pos_enter_wait_resource(pos, now_us);
                            continue;
                        }

                        for (int j = pos->midrev_sw_count - 1; j >= 0; j--) {
                            track_set_switch(pos->midrev_sw_nums[j],
                                             pos->midrev_sw_dirs[j]);
                            track_update_switch(pos->midrev_sw_nums[j],
                                                pos->midrev_sw_dirs[j]);
                        }
                        resend_unreliable_switches(pos->midrev_sw_nums,
                                                    pos->midrev_sw_dirs,
                                                    pos->midrev_sw_count);
                        if (pos->midrev_sw_count > 0) ui_mark_switches_dirty();

                        pos->target_sensor    = pos->midrev_final_target;
                        pos->target_offset_mm = pos->midrev_final_offset;

                        {
                            int32_t d2 = follow_dist(pos->cur_sensor,
                                                      pos->midrev_final_target,
                                                      200);
                            if (d2 >= 0) {
                                int32_t rem2 = d2 + pos->midrev_final_offset;
                                pos->dist_to_target_mm = (rem2 > 0) ? rem2 : 0;
                            } else {
                                /* fallback: use pre-planned estimate */
                                int32_t rem2 = pos->midrev_dist_after
                                               + pos->midrev_final_offset;
                                pos->dist_to_target_mm = (rem2 > 0) ? rem2 : 0;
                            }
                        }

                        pos->user_speed = GOTO_USER_SPEED;
                        int can_spd = 1 + (GOTO_USER_SPEED - 1) * 77;
                        track_set_speed(pos->train_num, can_spd);
                        int32_t cv = pos->cached_v[GOTO_USER_SPEED];
                        pos->effective_v = (cv > 0) ? cv
                            : speed_table_get_v(pos->train_ind, GOTO_USER_SPEED);
                        pos->speed_warmup_mm = 400;
                        pos->cur_sensor_time = now_us;
                        pos->stopping_since_us = 0;

                        /* First sensor after reversal = the reversal sensor itself
                         * (cur_sensor was already updated to old_cur->reverse above).
                         * Use pred_trigger_time=0 to suppress the time gate so that
                         * a WAIT_RESOURCE delay between reversal and route-start
                         * does not cause the sensor to be time-gated out. */
                        pos->pred_next_sensor       = pos->cur_sensor;
                        pos->pred_alt_sensor        = NULL;
                        pos->pred_trigger_time      = 0;
                        pos->dead_track_deadline_us = 0;

                        pos->route_state = TRAIN_STATE_ON_ROUTE;
                    } else {
                        pos->route_state        = TRAIN_STATE_STOPPED;
                        pos->orig_user_target   = NULL;
                        pos->orig_target_offset = 0;
                        traffic_release_train_keep_position(pos->train_num, pos->cur_sensor);
                        start_queued_goto_if_any(pos);
                    }
                    ui_mark_position_dirty();
                }
            }
            continue;
        }

        /* Continuous stop-at check
         * Between sensor triggers, estimate remaining distance using speed
         * and time elapsed since the last known sensor position. */
        if (pos->route_state == TRAIN_STATE_ON_ROUTE &&
            pos->target_sensor != NULL &&
            pos->cur_sensor    != NULL &&
            pos->effective_v   >  0) {

            int32_t dist_from_cur = follow_dist(pos->cur_sensor,
                                                pos->target_sensor, 150);
            if (dist_from_cur >= 0) {
                uint64_t elapsed  = now_us - pos->cur_sensor_time;
                int32_t  traveled = (int32_t)((int64_t)pos->effective_v *
                                              (int64_t)elapsed / 1000000LL);

                int32_t rem = dist_from_cur + pos->target_offset_mm - traveled;
                if (rem < 0) rem = 0;
                pos->dist_to_target_mm = rem;

                int user_spd = pos->user_speed;
                int32_t a_tick = speed_table_get_decel(pos->train_ind, user_spd, pos->target_sensor);

                if (a_tick > 0) {
                    int32_t d_brake_tick = (int32_t)((int64_t)pos->effective_v
                                          * pos->effective_v / (2LL * a_tick));
                    /* Issue stop early to compensate overshoot. */
                    int32_t d_early_tick = d_brake_tick + (int32_t)(
                        (int64_t)pos->effective_v * (int64_t)STOP_EARLY_US[pos->train_ind] / 1000000LL);
                    if (rem <= d_early_tick) {
                        pos->route_state       = TRAIN_STATE_STOPPING;
                        pos->stopping_since_us = now_us;
                        track_set_speed(pos->train_num, 0);
                        ui_mark_position_dirty();
                        continue;
                    }
                }
                ui_mark_position_dirty();
            }
        }

        /* Dead-track detection */
        if ((pos->route_state == TRAIN_STATE_LOOP_FIND_DIR  ||
             pos->route_state == TRAIN_STATE_ON_ROUTE) &&
            pos->dead_track_deadline_us > 0 &&
            now_us > pos->dead_track_deadline_us) {

            pos->effective_v              = 0;
            pos->route_state              = TRAIN_STATE_DEAD_TRACK;
            pos->target_sensor            = NULL;
            pos->target_offset_mm         = 0;
            pos->offroute_valid           = 1;
            pos->offroute_expected_sensor = pos->pred_next_sensor;
            pos->pred_next_sensor         = NULL;
            pos->pred_alt_sensor          = NULL;
            pos->pred_trigger_time        = 0;
            pos->dead_track_deadline_us   = 0;
            traffic_release_train(pos->train_num);
            /* orig_user_target / orig_target_offset kept for recovery */
            ui_mark_position_dirty();
            continue;
        }

        if (pos->pred_trigger_time == 0) continue;
        if (pos->pred_next_sensor == NULL) continue;

        /* If more than 2x the expected interval has elapsed without a sensor,
         * advance the prediction to the next node (handles broken/missing
         * sensors without declaring dead track). */
        if (pos->cur_sensor_time > 0 &&
            pos->pred_trigger_time > pos->cur_sensor_time &&
            now_us > 2 * pos->pred_trigger_time - pos->cur_sensor_time) {

            track_node *skipped = pos->pred_next_sensor;
            uint64_t dt = 0;
            pos->pred_next_sensor  = predict_next_sensor(pos, skipped, &dt);
            pos->pred_trigger_time = now_us + dt;

            if (pos->target_sensor &&
                pos->route_state == TRAIN_STATE_ON_ROUTE) {
                int32_t skip_dist = follow_dist(skipped,
                    (pos->pred_next_sensor ? pos->pred_next_sensor
                                           : pos->target_sensor), 50);
                if (skip_dist > 0) {
                    pos->dist_to_target_mm -= skip_dist;
                    if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
                }
            }
        }
    }
}
