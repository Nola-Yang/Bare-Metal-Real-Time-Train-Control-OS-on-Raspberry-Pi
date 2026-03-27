#include "train_tracking/position.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/speed_table.h"
#include "server/clock_server.h"
#include "server/nameserver.h"
#include "timer.h"
#include "kassert.h"
#include "ui.h"
#include "util.h"
#include <stddef.h>
#include <stdint.h>

static uint32_t Speed_Warmup_Distance = 1000;

/* Time from sending the command to the train actually starting
 * to move */
#define GO_LATENCY_US 1000000ULL


static int get_can_speed(int speed_level) {
    return 1 + (speed_level - 1) * 77;
}

/* Start train moving at constant speed to acquire position (FIND_POS).
 * Used by pos_goto (UNKNOWN state) and pos_start_find_pos. */
static void pos_begin_pos_find(train_pos_t *pos, uint64_t now_us) {
    pos->user_speed      = DEFAULT_SPEED_LEVEL;
    pos->accel_a_eff = speed_table_get_accel(pos->train_ind, DEFAULT_SPEED_LEVEL);

    int can_spd          = get_can_speed(DEFAULT_SPEED_LEVEL);
    track_set_speed(pos->train_num, can_spd);

    pos->effective_v     = 0;               /* will be ramped by tick */
    pos->speed_warmup_mm = Speed_Warmup_Distance;
    pos->cur_sensor_time = now_us;
    pos->is_accelerating = 1;
    pos->accel_start_us  = now_us + GO_LATENCY_US;
    pos->awaiting_post_launch_sensor = 1;
    pos->force_offroute_on_next_sensor = 0;
    pos->dead_track_rescue_pending = 0;
    pos->dead_track_recover.valid = 0;
    pos->dead_track_recover.orig_target = NULL;
    pos->dead_track_recover.orig_offset_mm = 0;
    pos->dead_track_bootstrap_due_us = 0;
    pos->stopped_on_target_hit = 0;
    pos_clear_deadlock_recover(pos);
    pos->route_state     = TRAIN_STATE_FIND_POS;
}

/* ===== Position slot management ===== */

static track_node *predict_next_sensor_preserve_pred(train_pos_t *pos,
                                                     track_node *cur,
                                                     uint64_t *out_dt_us) {
    track_node *saved_alt = NULL;
    track_node *saved_branch = NULL;
    if (pos) {
        saved_alt = pos->pred.alt_sensor;
        saved_branch = pos->pred.branch_node;
    }

    track_node *next = predict_next_sensor(pos, cur, out_dt_us);

    if (pos) {
        pos->pred.alt_sensor = saved_alt;
        pos->pred.branch_node = saved_branch;
    }
    return next;
}

uint64_t pos_dead_track_deadline_from_interval(uint64_t now_us, uint64_t interval_us) {
    uint64_t timeout_us;

    if (interval_us == 0) return 0;

    if (interval_us > UINT64_MAX / DEAD_TRACK_TIMEOUT_MULTIPLIER) {
        timeout_us = UINT64_MAX;
    } else {
        timeout_us = interval_us * DEAD_TRACK_TIMEOUT_MULTIPLIER;
    }

    if (timeout_us < DEAD_TRACK_TIMEOUT_MIN_US) {
        timeout_us = DEAD_TRACK_TIMEOUT_MIN_US;
    }

    if (timeout_us > UINT64_MAX - now_us) return UINT64_MAX;
    return now_us + timeout_us;
}

void pos_refresh_dead_track_deadline(train_pos_t *pos, uint64_t now_us) {
    if (!pos) return;

    uint64_t t1 = 0;

    if (pos->pred.next_sensor != NULL && pos->pred.trigger_time > now_us) {
        t1 = pos->pred.trigger_time - now_us;
    } else if (pos->cur_sensor != NULL) {
        (void)predict_next_sensor_preserve_pred(pos, pos->cur_sensor, &t1);
    }

    pos->dead_track_deadline_us = pos_dead_track_deadline_from_interval(now_us, t1);
}

void pos_launch_at_goto_speed(train_pos_t *pos, uint64_t now_us) {
    pos->user_speed      = pos->goto_speed;
    int can_spd          = get_can_speed(pos->goto_speed);
    track_set_speed(pos->train_num, can_spd);
    pos->effective_v     = 0;               
    pos->speed_warmup_mm = Speed_Warmup_Distance;
    pos->cur_sensor_time = now_us;
    pos->is_accelerating = 1;
    pos->accel_start_us  = now_us + GO_LATENCY_US;
    pos->awaiting_post_launch_sensor = 1;
    pos->force_offroute_on_next_sensor = 0;
    pos->dead_track_rescue_pending = 0;
    pos->dead_track_recover.valid = 0;
    pos->dead_track_bootstrap_due_us = 0;
    pos->stopped_on_target_hit = 0;
}

static int state_is_goto_active(train_route_state_t st) {
    return (st == TRAIN_STATE_STOPPING_GOTO     ||
            st == TRAIN_STATE_FIND_POS     ||
            st == TRAIN_STATE_ON_ROUTE          ||
            st == TRAIN_STATE_STOPPING          ||
            st == TRAIN_STATE_RECOVERY_STOPPING ||
            st == TRAIN_STATE_DEAD_TRACK        ||
            st == TRAIN_STATE_WAIT_SWITCH_SETTLE ||
            st == TRAIN_STATE_WAIT_RESOURCE);
}

track_node *pos_release_keep_end(track_node *last_hit, track_node *hint) {
    if (hint) return hint;
    return predict_next_sensor(NULL, last_hit, NULL);
}

/* ===== Public API ===== */

void pos_init(void) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        g_pos[i].train_num = -1;
        g_pos[i].replan.seen_generation = 0;
    }
    pos_clear_deadlock_notice();
    pos_reset_game_events();
    traffic_init();
    route_init();
}

void pos_on_reverse(int train_num) {
    train_pos_t *pos = pos_find_slot(train_num);
    if (!pos) return;

    pos->prev_going_forward = pos->going_forward;
    pos->going_forward = !pos->going_forward;

    if (pos->cur_sensor && pos->cur_sensor->reverse)
        pos->cur_sensor = pos->cur_sensor->reverse;

    track_node *keep_end = pos_release_keep_end(pos->cur_sensor, NULL);
    pos_clear_prediction(pos);
    traffic_release_train_keep_body(train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM, keep_end);
    pos->replan.blocker_mask = 0;

    ui_mark_position_dirty();
}

void pos_on_speed_change(int train_num, int user_speed) {
    train_pos_t *pos = pos_find_or_create_slot(train_num, user_speed);
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
        pos->speed_warmup_mm = Speed_Warmup_Distance;
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
int pos_goto(int train_num, track_node *target, int speed_level, int32_t offset_mm) {
    KASSERT(target != NULL);
    if (!target) return 0;

    train_pos_t *pos = pos_find_or_create_slot(train_num, speed_level);
    KASSERT(pos != NULL);
    if (!pos) return 0;
    if (pos->route_state == TRAIN_STATE_DEAD_TRACK) return 0;

    if (state_is_goto_active(pos->route_state)) {
        pos->queued_target = target;
        pos->queued_offset_mm = offset_mm;
        pos->queued_valid = 1;
        ui_mark_position_dirty();
        return 1;
    }

    if (pos->route_state != TRAIN_STATE_STOPPED) {
        if (pos->cur_sensor) {
            traffic_release_train_keep_body(train_num, pos->cur_sensor,
                                            TRAIN_BODY_MM,
                                            pos_release_keep_end(pos->cur_sensor,
                                                                 pos->pred.next_sensor));
        } else {
            traffic_release_train(train_num);
        }
    }

    pos_prepare_goto_request(pos, target, speed_level, offset_mm);
    ui_mark_position_dirty();

    if (pos->route_state == TRAIN_STATE_UNKNOWN) {
        /* Position unknown — start moving to trigger a sensor and acquire position.
         * handle_sensor will stop the train and transition to STOPPING_GOTO. */
        pos_begin_pos_find(pos, read_timer());

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
        //Todo
        KASSERT(ok);
    }

    return 1;
}

int pos_start_find_pos(int train_num, int speed_level) {
    train_pos_t *pos = pos_find_or_create_slot(train_num, speed_level);
    if (!pos) return 0;
    if (pos->route_state != TRAIN_STATE_UNKNOWN &&
        pos->route_state != TRAIN_STATE_DEAD_TRACK) return 0;

    pos_enter_find_pos(pos, read_timer());

    return 1;
}

void pos_enter_find_pos(train_pos_t *pos, uint64_t now_us) {
    track_node *keep_pred = NULL;

    if (!pos) return;

    if (pos->cur_sensor) {
        track_node *keep_hint = pos->offroute_expected_sensor
                                ? pos->offroute_expected_sensor
                                : pos->pred.next_sensor;
        keep_pred = pos_release_keep_end(pos->cur_sensor, keep_hint);
    }

    pos_prepare_find_pos_request(pos);
    if (pos->cur_sensor) {
        traffic_refresh_sensor_prediction_reservation(pos->train_num,
                                                      pos->cur_sensor,
                                                      keep_pred,
                                                      TRAIN_BODY_MM);
    } else {
        traffic_release_train(pos->train_num);
    }
    pos_begin_pos_find(pos, now_us);
    ui_mark_position_dirty();
}

int pos_is_train_goto_active(int train_num) {
    train_pos_t *pos = pos_find_slot(train_num);
    if (!pos) return 0;
    return state_is_goto_active(pos->route_state);
}

int pos_is_train_position_known(int train_num) {
    train_pos_t *pos = pos_find_slot(train_num);
    if (!pos) return 0;
    return pos->cur_sensor != NULL && pos->position_known;
}

int pos_queue_goto(int train_num, track_node *target, int speed_level, int32_t offset_mm) {
    train_pos_t *pos = pos_find_or_create_slot(train_num, speed_level);
    if (!pos || !target) return 0;
    if (pos->route_state == TRAIN_STATE_DEAD_TRACK) return 0;
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
