#include "train_tracking/position.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "train_tracking/speed_table.h"
#include "server/clock_server.h"
#include "server/nameserver.h"
#include "timer.h"
#include "kassert.h"
#include "ui.h"
#include "util.h"
#include <stddef.h>
#include <stdint.h>


train_pos_t g_pos[MAX_POS_TRAINS];

static uint32_t Speed_Warmup_Distance = 1000;
static int g_pos_clock_tid = -1;

/* Time from sending the command to the train actually starting
 * to move */
#define GO_LATENCY_US 100000ULL

#ifdef TRACK_D
    static const int32_t GOTO_ACCEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {101, 101, 101, 101, 101};
#else
    static const int32_t GOTO_ACCEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {105, 109, 108, 109, 110};
#endif

/* Start train moving at GOTO_USER_SPEED to acquire position (FIND_POS).
 * Used by pos_goto (UNKNOWN state) and pos_start_direction_find. */
static void pos_begin_pos_find(train_pos_t *pos) {
    pos->user_speed      = GOTO_USER_SPEED;
    int can_spd          = 1 + (GOTO_USER_SPEED - 1) * 77;
    track_set_speed(pos->train_num, can_spd);
    pos->effective_v     = 0;               /* will be ramped by tick */
    pos->speed_warmup_mm = Speed_Warmup_Distance;
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
            slot->pred.skipped_sensor_count = 0;
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
            slot->replan.seen_generation   = traffic_get_change_generation();
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

void pos_reset_dead_train(int train_num) {
    train_pos_t *p = find_pos(train_num);
    if (!p) return;

    p->route_state = TRAIN_STATE_STOPPED;
}

void pos_clear_prediction(train_pos_t *pos) {
    pos->pred.next_sensor  = NULL;
    pos->pred.alt_sensor   = NULL;
    pos->pred.branch_node  = NULL;
    pos->pred.trigger_time = 0;
    pos->pred.skipped_sensor_count = 0;
    pos->dead_track_deadline_us = 0;
}

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

void pos_refresh_dead_track_deadline(train_pos_t *pos, uint64_t now_us) {
    if (!pos) return;

    uint64_t t1 = 0;

    if (pos->pred.next_sensor != NULL && pos->pred.trigger_time > now_us) {
        t1 = pos->pred.trigger_time - now_us;
    } else if (pos->cur_sensor != NULL) {
        (void)predict_next_sensor_preserve_pred(pos, pos->cur_sensor, &t1);
    }

    if (t1 > 0) {
        pos->dead_track_deadline_us = now_us + DEAD_TRACK_TIMEOUT;
    } else {
        pos->dead_track_deadline_us = 0;
    }
}

void pos_launch_at_goto_speed(train_pos_t *pos, uint64_t now_us) {
    pos->user_speed      = GOTO_USER_SPEED;
    int can_spd          = 1 + (GOTO_USER_SPEED - 1) * 77;
    track_set_speed(pos->train_num, can_spd);
    pos->effective_v     = 0;               
    pos->speed_warmup_mm = Speed_Warmup_Distance;
    pos->cur_sensor_time = now_us;
    pos->is_accelerating = 1;
    pos->accel_start_us  = now_us + GO_LATENCY_US;
}

void pos_wait_switch_settle(int sw_count) {
    if (sw_count <= 0) return;
    if (g_pos_clock_tid < 0) {
        g_pos_clock_tid = WhoIs(CLOCK_SERVER_NAME);
        KASSERT(g_pos_clock_tid >= 0);
    }
    KASSERT(Delay(g_pos_clock_tid, SWITCH_SETTLE_TICKS) >= 0);
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
    traffic_init();
    route_init();
}

void pos_on_reverse(int train_num) {
    train_pos_t *pos = find_pos(train_num);
    if (!pos) return;

    pos->going_forward = !pos->going_forward;
    if (pos->cur_sensor && pos->cur_sensor->reverse)
        pos->cur_sensor = pos->cur_sensor->reverse;

    track_node *keep_end = pos_release_keep_end(pos->cur_sensor, NULL);
    pos_clear_prediction(pos);
    traffic_release_train_keep_body(train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM, keep_end);

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
int pos_goto(int train_num, track_node *target, int32_t offset_mm) {
    KASSERT(target != NULL);
    if (!target) return 0;

    train_pos_t *pos = find_or_create_pos(train_num);
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

    if (pos->cur_sensor) {
        traffic_release_train_keep_body(train_num, pos->cur_sensor,
                                        TRAIN_BODY_MM,
                                        pos_release_keep_end(pos->cur_sensor,
                                                             pos->pred.next_sensor));
    } else {
        traffic_release_train(train_num);
    }

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
        //Todo
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
