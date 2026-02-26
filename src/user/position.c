#include "position.h"
#include "route_priv.h"
#include "track.h"
#include "track_data.h"
#include "speed_table.h"
#include "timer.h"
#include "util.h"
#include "kassert.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>


static train_pos_t g_pos[MAX_POS_TRAINS];

/* ===== Fixed loop switch settings ===== */

#define LOOP_SW_COUNT 4
static const int  LOOP_SW_NUMS[LOOP_SW_COUNT] = { 7,   8,   14,  11  };
static const char LOOP_SW_DIRS[LOOP_SW_COUNT] = { 'S', 'S', 'S', 'C' };
#define OFF_ROUTE_PATH_MAX_HOPS 120

/* ===== Position slot management ===== */

static train_pos_t *find_pos(int train_num) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        if (g_pos[i].train_num == train_num) return &g_pos[i];
    }
    return NULL;
}

static train_pos_t *find_or_create_pos(int train_num) {
    train_pos_t *p = find_pos(train_num);
    if (p) return p;
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        if (g_pos[i].train_num < 0) {
            train_pos_t *slot = &g_pos[i];
            slot->train_num             = train_num;
            slot->cur_sensor            = NULL;
            slot->cur_sensor_time       = 0;
            slot->effective_v           = 0;
            slot->user_speed            = 0;
            slot->pred_next_sensor      = NULL;
            slot->pred_trigger_time     = 0;
            slot->last_time_err_us      = 0;
            slot->last_dist_err_mm      = 0;
            slot->route_state           = TRAIN_STATE_UNKNOWN;
            slot->target_sensor         = NULL;
            slot->target_offset_mm      = 0;
            slot->dist_to_target_mm     = 0;
            slot->consec_missed         = 0;
            slot->pending_target        = NULL;
            slot->pending_offset_mm     = 0;
            slot->stable_sensor_count   = 0;
            slot->going_forward         = 1;
            slot->orig_user_target      = NULL;
            slot->orig_target_offset    = 0;
            slot->stopping_since_us     = 0;
            return slot;
        }
    }
    return NULL;
}

/* ===== Loop-entry setup helper ===== */

/*
 * transition_to_enter_loop — drive a stationary train back onto the fixed
 * loop and enter ENTER_LOOP state so handle_sensor can confirm arrival.
 *
 * Steps:
 *   1. Re-apply loop switch defaults.
 *   2. If the last known sensor is off-loop, BFS to the nearest loop entry
 *      and set the required intermediate switches.  If no forward path
 *      exists, reverse the train and retry from the reverse node.
 *   3. Restart the train at speed 8.
 *   4. Refresh effective_v and next-sensor prediction.
 *   5. Set route_state = ENTER_LOOP.
 */
static void transition_to_enter_loop(train_pos_t *pos, uint64_t now_us) {
    pos_apply_loop_switches();

    /* BFS to loop entry if not already on the loop */
    if (pos->cur_sensor != NULL &&
        !(is_forward_loop_sensor(pos->cur_sensor) ||
          is_reverse_loop_sensor(pos->cur_sensor))) {
        route_plan_t rp;
        if (bfs_find_route_to_loop(pos->cur_sensor, &rp)) {
            for (int j = 0; j < rp.sw_count; j++) {
                track_set_switch(rp.sw_nums[j], rp.sw_dirs[j]);
                track_update_switch(rp.sw_nums[j], rp.sw_dirs[j]);
            }
            if (rp.sw_count > 0) ui_mark_switches_dirty();
        } else {
            // try reverse
            KASSERT(pos->cur_sensor->reverse != NULL &&
                    bfs_find_route_to_loop(pos->cur_sensor->reverse, &rp));
            track_reverse(pos->train_num);
            pos->cur_sensor = pos->cur_sensor->reverse;
            pos->going_forward = !pos->going_forward;
            for (int j = 0; j < rp.sw_count; j++) {
                track_set_switch(rp.sw_nums[j], rp.sw_dirs[j]);
                track_update_switch(rp.sw_nums[j], rp.sw_dirs[j]);
            }
            if (rp.sw_count > 0) ui_mark_switches_dirty();
        }
    }

    /* Restart the train */
    if (pos->user_speed == 0) pos->user_speed = 8;
    int can_spd = 1 + (pos->user_speed - 1) * 77;
    track_set_speed(pos->train_num, can_spd);
    pos->effective_v     = SPEED_V_MM_S[pos->user_speed];
    pos->cur_sensor_time = now_us;

    /* Prediction */
    if (pos->cur_sensor != NULL) {
        uint64_t dt = 0;
        pos->pred_next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
        pos->pred_trigger_time = now_us + dt;
    }

    /* Restore pending_target from orig_user_target so that a second call
     * to execute_pending_route() after recovery still has a target to work
     * with. */
    if (pos->orig_user_target != NULL && pos->pending_target == NULL) {
        pos->pending_target    = pos->orig_user_target;
        pos->pending_offset_mm = pos->orig_target_offset;
    }

    pos->stable_sensor_count = 0;
    pos->route_state = TRAIN_STATE_ENTER_LOOP;
    ui_mark_position_dirty();
}

/* ===== Main sensor handler ===== */

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
            pos->consec_missed++;
            *out_is_skip = 1;
        } else {
            pos->consec_missed = 0;
        }
    } else {
        pos->consec_missed = 0;
    }

    /* Prediction error accounting */
    if (*out_was_predicted && pos->pred_trigger_time > 0) {
        pos->last_time_err_us =
            (int64_t)time_us - (int64_t)pos->pred_trigger_time;
        pos->last_dist_err_mm = (int32_t)(
            pos->effective_v * pos->last_time_err_us / 1000000LL);
        ui_mark_prediction_dirty();
    }

    /* EMA speed update */
    if (pos->cur_sensor && pos->effective_v > 0) {
        int32_t meas_dist = follow_dist(pos->cur_sensor, hit, 100);
        uint64_t dt = time_us - pos->cur_sensor_time;
        if (dt > 0 && meas_dist > 0) {
            int32_t meas_v =
                (int32_t)((int64_t)meas_dist * 1000000LL / (int64_t)dt);
            pos->effective_v = (7 * pos->effective_v + meas_v) / 8;
        }
    }
}

static void handle_sensor(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    int b_was_predicted, b_is_skip;
    update_sensor_stats(pos, hit, time_us, &b_was_predicted, &b_is_skip);

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

    /* ENTER_LOOP: first loop sensor confirms the train is back on the
     * loop.  Re-assert loop switches, update going_forward, and advance to
     * LOOP_STABILIZE.  
     */
    if (pos->route_state == TRAIN_STATE_ENTER_LOOP &&
        (is_forward_loop_sensor(hit) || is_reverse_loop_sensor(hit))) {
        pos_apply_loop_switches();
        
        pos->going_forward    = is_forward_loop_sensor(hit) ? 1 : 0;
        pos->consec_missed    = 0;
        pos->route_state      = TRAIN_STATE_LOOP_STABILIZE;
        pos->stable_sensor_count = 0;
        ui_mark_position_dirty();
    }

    
    // off-route check
    if ((pos->route_state == TRAIN_STATE_ON_ROUTE          ||
         pos->route_state == TRAIN_STATE_STOPPING          ||
         pos->route_state == TRAIN_STATE_LOOP_FIND_DIR     ||
         pos->route_state == TRAIN_STATE_LOOP_STABILIZE    ||
         pos->route_state == TRAIN_STATE_ENTER_LOOP) &&
        pos->pred_next_sensor != NULL &&
        !b_was_predicted && !b_is_skip) {
        pos->consec_missed         = 0;
        pos->pred_next_sensor      = NULL;
        pos->pred_trigger_time     = 0;
        track_set_speed(pos->train_num, 0);
        pos->route_state       = TRAIN_STATE_RECOVERY_STOPPING;
        pos->stopping_since_us = time_us;
        ui_mark_position_dirty();
        return;
    }

    /* Predict next sensor */
    uint64_t dt_pred = 0;
    pos->pred_next_sensor  = predict_next_sensor(pos, hit, &dt_pred);
    pos->pred_trigger_time = time_us + dt_pred;

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
            int64_t abs_err = pos->last_time_err_us;
            if (abs_err < 0) abs_err = -abs_err;

            if (prev_sensor && abs_err < STABLE_TIME_ERR_US) {
                pos->stable_sensor_count++;
            } else {
                pos->stable_sensor_count = 0;
            }

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

            if (user_spd > 0 && rem <= SPEED_STOP_DIST_MM[user_spd]) {
                pos->route_state       = TRAIN_STATE_STOPPING;
                pos->stopping_since_us = time_us;
                track_set_speed(pos->train_num, 0);
            }
        }
    }

    ui_mark_position_dirty();
}

/* ===== Public API ===== */

/*
 * Fill SPEED_V_MM_S (mm/s) from:
 *   f(x) = -0.3358x^5 + 17.71x^4 - 375.3x^3 + 4053x^2 - 22980x + 58520
 * Speed in mm/s = 1,000,000 / f(x).
 *
 * Integer arithmetic: coefficients scaled by 10000, so val = f(x)*10000.
 * Then speed = 10^10 / val.
 */
static void init_speed_table(void) {
    SPEED_V_MM_S[0] = 0;
    for (int x = 1; x <= 14; x++) {
        int64_t x2 = (int64_t)x * x;
        int64_t x3 = x2 * x;
        int64_t x4 = x3 * x;
        int64_t x5 = x4 * x;
        int64_t val = -3358LL    * x5
                    + 177100LL   * x4
                    - 3753000LL  * x3
                    + 40530000LL * x2
                    - 229800000LL* x
                    + 585200000LL;
        /* val = f(x)*10000 us/mm; speed = 1e6/(val/10000) = 1e10/val */
        SPEED_V_MM_S[x] = (val > 0) ? (int32_t)(10000000000LL / val) : 0;
    }
}

/*
 * Fill SPEED_STOP_DIST_MM using the cubic formula calibrated from measurements:
 *   dist(x) = 1.463*x^3 - 21.19*x^2 + 148.8*x - 216  (mm)
 *
 * Implemented in integer arithmetic (scaled by 1000) to avoid floating point.
 */
static void init_braking_table(void) {
    SPEED_STOP_DIST_MM[0] = 0;
    for (int x = 1; x <= 14; x++) {
        int64_t val = (int64_t)1463 * x * x * x
                    - (int64_t)21190 * x * x
                    + (int64_t)148800 * x
                    - (int64_t)216000;
        val /= 1000;
        SPEED_STOP_DIST_MM[x] = (val > 0) ? (int32_t)val : 0;
    }
}

void pos_init(void) {
    init_speed_table();
    init_braking_table();
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        g_pos[i].train_num = -1;
    }
}

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

        /* Once braking is complete, state transition to ENTER_LOOP */
        if (pos->route_state == TRAIN_STATE_RECOVERY_STOPPING) {
            uint64_t brake_us = 1000000ULL;
            if (pos->effective_v > 0 &&
                pos->user_speed > 0 && pos->user_speed <= 14) {
                int32_t d = SPEED_STOP_DIST_MM[pos->user_speed];
                brake_us  = (uint64_t)d * 1000000ULL / (uint64_t)pos->effective_v;
                brake_us  = brake_us * 3 / 2;
            }
            if (now_us < pos->stopping_since_us + brake_us) continue;

            pos->effective_v = 0;
            transition_to_enter_loop(pos, now_us);
            continue;
        }

        /* tr 0 command sent; wait for physical stop -> STOPPED.
         * Keep effective_v intact until confirmed stopped for accurate estimate. */
        if (pos->route_state == TRAIN_STATE_STOPPING_TR) {
            uint64_t brake_us = 1000000ULL;
            if (pos->effective_v > 0 &&
                pos->user_speed > 0 && pos->user_speed <= 14) {
                int32_t d = SPEED_STOP_DIST_MM[pos->user_speed];
                brake_us  = (uint64_t)d * 1000000ULL / (uint64_t)pos->effective_v;
                brake_us  = brake_us * 3 / 2;
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
            if (pos->effective_v > 0 &&
                pos->user_speed > 0 && pos->user_speed <= 14) {
                int32_t d = SPEED_STOP_DIST_MM[pos->user_speed];
                brake_us  = (uint64_t)d * 1000000ULL / (uint64_t)pos->effective_v;
                brake_us  = brake_us * 3 / 2;
            }
            if (now_us >= pos->stopping_since_us + brake_us) {
                pos->effective_v = 0;
                transition_to_enter_loop(pos, now_us);
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
                if (pos->effective_v > 0 &&
                    pos->user_speed > 0 && pos->user_speed <= 14) {
                    int32_t d = SPEED_STOP_DIST_MM[pos->user_speed];
                    brake_us = (uint64_t)d * 1000000ULL /
                               (uint64_t)pos->effective_v;
                    brake_us = brake_us * 3 / 2;
                }
                if (now_us >= pos->stopping_since_us + brake_us) {
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
                
                if (user_spd > 0 && rem <= SPEED_STOP_DIST_MM[user_spd]) {
                    pos->route_state       = TRAIN_STATE_STOPPING;
                    pos->stopping_since_us = now_us;
                    track_set_speed(pos->train_num, 0);
                    ui_mark_position_dirty();
                    continue;
                }
                ui_mark_position_dirty();
            }
        }

        if (pos->pred_trigger_time == 0) continue;
        if (pos->pred_next_sensor == NULL) continue;

        /* Todo：need calibration If twice the predicted time has elapsed, sensor may be missing */
        if (now_us > pos->pred_trigger_time * 2 &&
            pos->pred_trigger_time > 0) {

            pos->consec_missed++;

            track_node *skipped = pos->pred_next_sensor;
            uint64_t dt = 0;
            pos->pred_next_sensor  = predict_next_sensor(pos, skipped, &dt);
            pos->pred_trigger_time = now_us + dt;

            if (pos->target_sensor) {
                int32_t skip_dist = follow_dist(skipped,
                    (pos->pred_next_sensor ? pos->pred_next_sensor
                                           : pos->target_sensor), 50);
                if (skip_dist > 0)
                    pos->dist_to_target_mm -= skip_dist;
            }
        }
    }
}

void pos_on_speed_change(int train_num, int user_speed) {
    train_pos_t *pos = find_or_create_pos(train_num);
    if (!pos) return;

    pos->user_speed = user_speed;

    if (user_speed > 0 && user_speed <= 14) {
        pos->effective_v = SPEED_V_MM_S[user_speed];
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
    for (int i = 0; i < LOOP_SW_COUNT; i++) {
        track_set_switch(LOOP_SW_NUMS[i], LOOP_SW_DIRS[i]);
        track_update_switch(LOOP_SW_NUMS[i], LOOP_SW_DIRS[i]);
    }
    ui_mark_switches_dirty();
}

int pos_goto(int train_num, track_node *target, int32_t offset_mm) {
    KASSERT(target != NULL);
    if (!target) return 0;

    train_pos_t *pos = find_or_create_pos(train_num);
    KASSERT(pos != NULL);
    if (!pos) return 0;

    pos->pending_target     = target;
    pos->pending_offset_mm  = offset_mm;
    pos->orig_user_target   = target;
    pos->orig_target_offset = offset_mm;

    if (pos->route_state == TRAIN_STATE_UNKNOWN) {
        /* assume train is on loop.
         * If the user type tr but the sys didn't recieve the 
         * pos and dir, assumption still hold. just overwrite the speed.
         * auto-start at speed 8 to
         * acquire position and direction 
        */

        pos_apply_loop_switches();
        pos->user_speed = 8;
        int can_spd = 1 + (pos->user_speed - 1) * 77;
        track_set_speed(train_num, can_spd);

        pos->effective_v     = SPEED_V_MM_S[pos->user_speed];
        pos->cur_sensor_time = read_timer();
        pos->going_forward   = 1;
        pos->stable_sensor_count = 0;
        pos->route_state = TRAIN_STATE_LOOP_FIND_DIR;
        uint64_t dt = 0;
        pos->pred_next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
        pos->pred_trigger_time = pos->cur_sensor_time + dt;

    } else if (pos->route_state == TRAIN_STATE_KNOWN) {
        /* pos Known, running via tr.
         * Issue stop command now; pos_on_tick will call
         * transition_to_enter_loop() once physically stopped. */
        track_set_speed(train_num, 0);
        pos->stopping_since_us = read_timer();
        pos->route_state = TRAIN_STATE_STOPPING_GOTO;

    } else if (pos->route_state == TRAIN_STATE_STOPPING_TR) {
        /* Train is already decelerating from a tr 0 command.
         * Redirect the post-stop action from STOPPED to ENTER_LOOP. */
        if (pos->user_speed == 0) pos->user_speed = 8;
        pos->route_state = TRAIN_STATE_STOPPING_GOTO;

    } else if (pos->route_state == TRAIN_STATE_STOPPED) {
        transition_to_enter_loop(pos, read_timer());
    }

    return 1;
}

int pos_is_train_goto_active(int train_num) {
    train_pos_t *pos = find_pos(train_num);
    if (!pos) return 0;
    train_route_state_t st = pos->route_state;
    return (st == TRAIN_STATE_STOPPING_GOTO    ||
            st == TRAIN_STATE_ENTER_LOOP       ||
            st == TRAIN_STATE_LOOP_FIND_DIR    ||
            st == TRAIN_STATE_LOOP_STABILIZE   ||
            st == TRAIN_STATE_ON_ROUTE         ||
            st == TRAIN_STATE_STOPPING         ||
            st == TRAIN_STATE_RECOVERY_STOPPING) ? 1 : 0;
}

train_pos_t *pos_get(int train_num) {
    return find_pos(train_num);
}

train_pos_t *pos_get_by_index(int i) {
    if (i < 0 || i >= MAX_POS_TRAINS) return NULL;
    return &g_pos[i];
}

track_node *pos_find_sensor(const char *name) {
    if (!name) return NULL;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type == NODE_SENSOR && g_track[i].name != NULL) {
            const char *a = g_track[i].name;
            const char *b = name;
            while (*a && *b && *a == *b) { a++; b++; }
            if (*a == '\0' && *b == '\0') return &g_track[i];
        }
    }
    return NULL;
}
