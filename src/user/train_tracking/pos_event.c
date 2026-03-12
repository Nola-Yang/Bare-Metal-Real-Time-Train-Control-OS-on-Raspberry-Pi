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
     * LOOP_STABILIZE: must update so predict_next_sensor stays accurate and
     *   last_time_err_us can converge.
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
                        (st == TRAIN_STATE_LOOP_STABILIZE ||
                         st == TRAIN_STATE_ON_ROUTE);
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
    track_node *expected_sensor = pos->pred_next_sensor;

    track_node *prev_sensor = pos->cur_sensor;
    pos->cur_sensor      = hit;
    pos->cur_sensor_time = time_us;

    if (prev_sensor && !b_was_predicted && !b_is_skip) {
        observe_path_and_correct_switches(prev_sensor, hit);
    }

    // Infer direction for trains running without goto commands.
    if (pos->route_state == TRAIN_STATE_UNKNOWN && prev_sensor != NULL) {
        if (follow_dist(prev_sensor, hit, 20) >= 0) {
            pos->going_forward = 1;
        } else if (prev_sensor->reverse != NULL &&
                   follow_dist(prev_sensor->reverse, hit, 20) >= 0) {
            pos->going_forward = 0;
        } else {
            KASSERT(0 && "Two consecutive sensors with no path between them?");
        }

        /* UNKNOWN → KNOWN: position and direction known
         * Only transition if the train is running via a tr command (user_speed > 0).
        */
        pos->route_state = TRAIN_STATE_KNOWN;
        ui_mark_position_dirty();

    }

    /* Dead track recovery: a sensor fired after the user manually pushed
     * the train to a powered section.  Restart the goto via the normal
     * recovery flow */
    if (pos->route_state == TRAIN_STATE_DEAD_TRACK) {
        pos->offroute_valid = 0;
        transition_to_enter_loop(pos, time_us);
        ui_mark_position_dirty();
        return;
    }

    /* ENTER_LOOP: first loop sensor confirms the train is back on the
     * loop.  Re-assert loop switches, update going_forward, and advance to
     * LOOP_STABILIZE.
     */
    if (pos->route_state == TRAIN_STATE_ENTER_LOOP &&
        (is_forward_loop_sensor(hit) || is_reverse_loop_sensor(hit))) {
        pos_apply_loop_switches();

        pos->going_forward    = is_forward_loop_sensor(hit) ? 1 : 0;
        pos->route_state      = TRAIN_STATE_LOOP_STABILIZE;
        pos->stable_sensor_count = 0;
        ui_mark_position_dirty();
    }


    // off-route check
    if (pos->skip_offroute_count > 0) {
        pos->skip_offroute_count--;
    } else if ((pos->route_state == TRAIN_STATE_ON_ROUTE          ||
         pos->route_state == TRAIN_STATE_STOPPING          ||
         pos->route_state == TRAIN_STATE_LOOP_FIND_DIR     ||
         pos->route_state == TRAIN_STATE_LOOP_STABILIZE    ||
         pos->route_state == TRAIN_STATE_ENTER_LOOP) &&
        pos->pred_next_sensor != NULL &&
        !b_was_predicted && !b_is_skip) {

        /* Unexpected sensor: check if we can still reach the target from here.
         * If yes, update prediction from hit and continue without off-route.
        */

        int still_reachable = 0;

        if (pos->route_state == TRAIN_STATE_ENTER_LOOP ) {
            if (follow_reaches_loop(hit, 50)) {
                still_reachable = 1;
            }
        }

        if (!still_reachable) {
            pos->offroute_valid           = 1;
            pos->offroute_expected_sensor = expected_sensor;
            pos->offroute_actual_sensor   = hit;
            pos->pred_next_sensor      = NULL;
            pos->pred_trigger_time     = 0;
            track_set_speed(pos->train_num, 0);
            pos->route_state       = TRAIN_STATE_RECOVERY_STOPPING;
            pos->stopping_since_us = time_us;
            ui_mark_position_dirty();
            return;
        }

        /* Fall through: prediction is refreshed below from hit */
    }

    /* Predict next sensor */
    uint64_t dt_pred = 0;
    pos->pred_next_sensor  = predict_next_sensor(pos, hit, &dt_pred);
    pos->pred_trigger_time = time_us + dt_pred;

    if (pos->pred_next_sensor != NULL && dt_pred > 0) {
        uint64_t T2 = 0;
        predict_next_sensor(pos, pos->pred_next_sensor, &T2);
        pos->dead_track_deadline_us =
            time_us + DEAD_TRACK_DEADLINE_MULTIPLIER * (dt_pred + T2);
    } else {
        pos->dead_track_deadline_us = 0;
    }

    /* Direction detection and speed-stabilisation */
    if (pos->pending_target) {

        if (pos->route_state == TRAIN_STATE_LOOP_FIND_DIR && prev_sensor) {
            int fwd_prev = is_forward_loop_sensor(prev_sensor);
            int fwd_hit  = is_forward_loop_sensor(hit);
            int rev_prev = is_reverse_loop_sensor(prev_sensor);
            int rev_hit  = is_reverse_loop_sensor(hit);

            if (fwd_prev && fwd_hit) {
                pos->going_forward       = 1;
                pos->route_state         = TRAIN_STATE_LOOP_STABILIZE;
                pos->stable_sensor_count = 0;
            } else if (rev_prev && rev_hit) {
                pos->going_forward       = 0;
                pos->route_state         = TRAIN_STATE_LOOP_STABILIZE;
                pos->stable_sensor_count = 0;
            }
        }

        else if (pos->route_state == TRAIN_STATE_LOOP_STABILIZE) {
            if (b_was_predicted) {
                int64_t abs_err = pos->last_time_err_us;
                if (abs_err < 0) abs_err = -abs_err;
                if (abs_err < STABLE_TIME_ERR_US) {
                    pos->stable_sensor_count++;
                } else {
                    pos->stable_sensor_count = 0;
                }
            }
            /* b_is_skip:
             * Leave stable_sensor_count unchanged */

            if (pos->stable_sensor_count >= STABLE_SENSOR_MIN) {
                execute_pending_route(pos);
            }
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

/* ===== Public event API ===== */

void pos_on_sensor_trigger(uint16_t sensor_id, uint64_t time_us) {
    int track_idx = (int)sensor_id - 1;
    if (track_idx < 0 || track_idx >= TRACK_MAX) return;

    track_node *hit = &g_track[track_idx];
    if (hit->type != NODE_SENSOR) return;

    /* assume single train: deliver every sensor to the first active slot. */
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        if (g_pos[i].train_num >= 0) {
            handle_sensor(&g_pos[i], hit, time_us);
            return;
        }
    }
}

void pos_on_tick(uint64_t now_us) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (pos->train_num < 0) continue;

        /* train physically stopped; no estimation until sensor fires. */
        if (pos->route_state == TRAIN_STATE_DEAD_TRACK) continue;

        /* Once braking is complete, state transition to ENTER_LOOP */
        if (pos->route_state == TRAIN_STATE_RECOVERY_STOPPING) {
            uint64_t brake_us = 1000000ULL;
            int32_t decel = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
            if (pos->effective_v > 0 && decel > 0) {
                brake_us = (uint64_t)pos->effective_v * 1500000ULL
                           / (uint64_t)decel;
            }
            if (now_us < pos->stopping_since_us + brake_us) continue;

            if (pos->user_speed > 0 && pos->user_speed <= 14)
                pos->cached_v[pos->user_speed] = pos->effective_v;
            pos->effective_v = 0;
            transition_to_enter_loop(pos, now_us);
            continue;
        }

        /* tr 0 command sent; wait for physical stop -> STOPPED.
         * Keep effective_v intact until confirmed stopped for accurate estimate. */
        if (pos->route_state == TRAIN_STATE_STOPPING_TR) {
            uint64_t brake_us = 1000000ULL;
            int32_t decel = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
            if (pos->effective_v > 0 && decel > 0) {
                brake_us = (uint64_t)pos->effective_v * 1500000ULL
                           / (uint64_t)decel;
            }
            if (now_us >= pos->stopping_since_us + brake_us) {
                pos->route_state = TRAIN_STATE_STOPPED;
                pos->effective_v = 0;
                ui_mark_position_dirty();
            }
            continue;
        }

        /* goto was issued while the train was running; stop
         * command has been sent.  Once physically stopped, drive to loop. */
        if (pos->route_state == TRAIN_STATE_STOPPING_GOTO) {
            uint64_t brake_us = 1000000ULL;
            int32_t decel = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
            if (pos->effective_v > 0 && decel > 0) {
                brake_us = (uint64_t)pos->effective_v * 1500000ULL
                           / (uint64_t)decel;
            }
            if (now_us >= pos->stopping_since_us + brake_us) {
                if (pos->user_speed > 0 && pos->user_speed <= 14)
                    pos->cached_v[pos->user_speed] = pos->effective_v;
                pos->effective_v = 0;
                if (!pos_try_direct_goto(pos)) {
                    transition_to_enter_loop(pos, now_us);
                }
            }
            continue;
        }

        /* STOPPING -> STOPPED transition.
         * Estimate braking time = stop_distance / effective_v (with 50% margin).
         * While braking/stopped, skip the sensor-timeout logic below. */
        if (pos->route_state == TRAIN_STATE_STOPPING ||
            pos->route_state == TRAIN_STATE_STOPPED) {
            if (pos->route_state == TRAIN_STATE_STOPPING &&
                pos->stopping_since_us > 0) {
                uint64_t brake_us = 1000000ULL;  /* 1 s default */
                int32_t decel = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
                if (pos->effective_v > 0 && decel > 0) {
                    brake_us = (uint64_t)pos->effective_v * 1500000ULL
                               / (uint64_t)decel;
                }
                if (now_us >= pos->stopping_since_us + brake_us) {
                    if (pos->user_speed > 0 && pos->user_speed <= 14)
                        pos->cached_v[pos->user_speed] = pos->effective_v;
                    pos->route_state       = TRAIN_STATE_STOPPED;
                    pos->effective_v       = 0;
                    pos->orig_user_target  = NULL;
                    pos->orig_target_offset = 0;
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
             pos->route_state == TRAIN_STATE_LOOP_STABILIZE ||
             pos->route_state == TRAIN_STATE_ENTER_LOOP     ||
             pos->route_state == TRAIN_STATE_ON_ROUTE) &&
            pos->dead_track_deadline_us > 0 &&
            now_us > pos->dead_track_deadline_us) {

            pos->effective_v              = 0;
            pos->route_state              = TRAIN_STATE_DEAD_TRACK;
            pos->target_sensor            = NULL;
            pos->target_offset_mm         = 0;
            pos->offroute_valid           = 1;
            pos->offroute_expected_sensor = pos->pred_next_sensor;
            pos->offroute_actual_sensor   = NULL;
            pos->pred_next_sensor         = NULL;
            pos->pred_trigger_time        = 0;
            pos->dead_track_deadline_us   = 0;
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
                    if (skip_dist >= pos->dist_to_target_mm) {
                        /* Skipped past the target — off-route */
                        pos->offroute_valid           = 1;
                        pos->offroute_expected_sensor = skipped;
                        pos->offroute_actual_sensor   = NULL;
                        pos->pred_next_sensor         = NULL;
                        pos->pred_trigger_time        = 0;
                        track_set_speed(pos->train_num, 0);
                        pos->route_state       = TRAIN_STATE_RECOVERY_STOPPING;
                        pos->stopping_since_us = now_us;
                        ui_mark_position_dirty();
                        continue;
                    }
                    pos->dist_to_target_mm -= skip_dist;
                }
            }
        }
    }
}
