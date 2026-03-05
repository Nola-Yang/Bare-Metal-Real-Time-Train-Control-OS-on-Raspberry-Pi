#include "train_tracking/position.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "train_tracking/speed_table.h"
#include "timer.h"
#include "kassert.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>


train_pos_t g_pos[MAX_POS_TRAINS];

#define GOTO_USER_SPEED 8

#ifdef TRACK_D
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {227, 232, 242, 229, 230};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {147, 158, 157, 153, 168};
#else
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {226, 224, 226, 222, 236};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {174, 170, 183, 180, 167};
#endif

int32_t speed_table_get_v(int32_t train_ind, int user_speed) {
    if (user_speed != GOTO_USER_SPEED) return 0;
    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;
    return GOTO_SPEED_MM_S[train_ind];
}

int32_t speed_table_get_decel(int32_t train_ind, int user_speed) {
    if (user_speed != GOTO_USER_SPEED) return 0;
    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;
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
    }

    // default index
    return 0;
}

static train_pos_t *find_or_create_pos(int train_num) {
    train_pos_t *p = find_pos(train_num);
    if (p) return p;
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        if (g_pos[i].train_num < 0) {
            int32_t train_ind = train_num_to_ind(train_num);

            train_pos_t *slot = &g_pos[i];
            slot->train_num             = train_num;
            slot->train_ind             = train_ind;
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
            slot->pending_target        = NULL;
            slot->pending_offset_mm     = 0;
            slot->stable_sensor_count   = 0;
            slot->going_forward         = 1;
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
            slot->dead_track_deadline_us  = 0;
            for (int s = 0; s < 15; s++) slot->cached_v[s] = 0;
            slot->speed_warmup_mm = 0;
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
void transition_to_enter_loop(train_pos_t *pos, uint64_t now_us) {
    int just_reversed         = 0;
    int physical_anchor_pending = 0;


    track_node *physical_anchor = NULL;
    int32_t     anchor_dist_mm  = 0;
    if (pos->cur_sensor != NULL) {
        int32_t saved_ev = pos->effective_v;
        if (saved_ev == 0) {
            int32_t tv = speed_table_get_v(pos->train_ind, GOTO_USER_SPEED);
            pos->effective_v = (tv > 0) ? tv : 100;
        }
        uint64_t dummy_dt = 0;
        physical_anchor = predict_next_sensor(pos, pos->cur_sensor, &dummy_dt);
        pos->effective_v = saved_ev;

        if (physical_anchor != NULL) {
            anchor_dist_mm = follow_dist(pos->cur_sensor, physical_anchor, 80);
            if (anchor_dist_mm < 0) anchor_dist_mm = 0;
        }
    }

    KASSERT(pos->cur_sensor != NULL);

    if (physical_anchor != NULL && (!is_forward_loop_sensor(physical_anchor) && !is_reverse_loop_sensor(physical_anchor))) {
        KASSERT(physical_anchor != NULL);
        route_plan_t rp;
        track_node *rev = physical_anchor->reverse;

        /* Priority 1: reverse direction reaches loop with current switches unchanged. */
        if (rev != NULL && follow_reaches_loop(rev, 80)) {
            track_reverse(pos->train_num);
            pos->cur_sensor    = rev;
            pos->going_forward = !pos->going_forward;
            just_reversed      = 1;

        } else {
            /* Priority 2:BFS */
            if (!bfs_find_route_to_loop(physical_anchor, &rp)) {
                KASSERT(rev != NULL && bfs_find_route_to_loop(rev, &rp));
                track_reverse(pos->train_num);
                pos->cur_sensor    = rev;
                pos->going_forward = !pos->going_forward;
                just_reversed      = 1;
                for (int j = 0; j < rp.sw_count; j++)
                    track_set_switch(rp.sw_nums[j], rp.sw_dirs[j]);
                resend_sw153_sw155(rp.sw_nums, rp.sw_dirs, rp.sw_count);

            } else {
                pos->cur_sensor = physical_anchor;
                for (int j = 0; j < rp.sw_count; j++)
                    track_set_switch(rp.sw_nums[j], rp.sw_dirs[j]);
                resend_sw153_sw155(rp.sw_nums, rp.sw_dirs, rp.sw_count);
                physical_anchor_pending = 1;
            }
        }
    } else if (physical_anchor != NULL) {
        pos_apply_loop_switches();
    } else {
        // ahead is exit, must reverse
        KASSERT(physical_anchor == NULL);
        track_reverse(pos->train_num);
        pos->cur_sensor    = pos->cur_sensor->reverse;
        pos->going_forward = !pos->going_forward;
        just_reversed      = 1;
        if (!follow_reaches_loop(pos->cur_sensor, 80)) {
            route_plan_t rp;
            KASSERT(bfs_find_route_to_loop(pos->cur_sensor, &rp));
            for (int j = 0; j < rp.sw_count; j++)
                track_set_switch(rp.sw_nums[j], rp.sw_dirs[j]);
            resend_sw153_sw155(rp.sw_nums, rp.sw_dirs, rp.sw_count);
        }
        
    }

    /* Restart the train */
    pos->user_speed = GOTO_USER_SPEED;
    int can_spd = 1 + (pos->user_speed - 1) * 77;
    track_set_speed(pos->train_num, can_spd);
    int32_t cv_loop  = pos->cached_v[pos->user_speed];
    pos->effective_v = (cv_loop > 0) ? cv_loop
                                     : speed_table_get_v(pos->train_ind, pos->user_speed);
    pos->speed_warmup_mm = 400;
    pos->cur_sensor_time = now_us;

    /* Prediction */
    if (pos->cur_sensor != NULL) {
        if (just_reversed) {
            pos->pred_next_sensor  = pos->cur_sensor;
            pos->pred_trigger_time = now_us;

        } else if (physical_anchor_pending) {
            pos->pred_next_sensor = pos->cur_sensor;  
            uint64_t dt = (anchor_dist_mm > 0 && pos->effective_v > 0)
                          ? (uint64_t)(uint32_t)anchor_dist_mm * 1000000ULL
                            / (uint64_t)(uint32_t)pos->effective_v
                          : 10000000ULL;              
            pos->pred_trigger_time = now_us + dt;

        } else {
            uint64_t dt = 0;
            pos->pred_next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
            pos->pred_trigger_time = now_us + dt;
        }

        /* Set dead-track deadline = now + 3*(T1+T2) */
        if (pos->pred_next_sensor != NULL && pos->pred_trigger_time > now_us) {
            uint64_t T1 = pos->pred_trigger_time - now_us;
            uint64_t T2 = 0;
            predict_next_sensor(pos, pos->pred_next_sensor, &T2);
            pos->dead_track_deadline_us =
                now_us + DEAD_TRACK_DEADLINE_MULTIPLIER * (T1 + T2);
        } else {
            pos->dead_track_deadline_us = 0;
        }
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

/* ===== Public API ===== */

void pos_init(void) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        g_pos[i].train_num = -1;
    }
}

void pos_on_reverse(int train_num) {
    train_pos_t *pos = find_pos(train_num);
    if (!pos) return;

    pos->going_forward = !pos->going_forward;
    if (pos->cur_sensor && pos->cur_sensor->reverse)
        pos->cur_sensor = pos->cur_sensor->reverse;

    pos->pred_next_sensor  = NULL;
    pos->pred_trigger_time = 0;

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
    ui_mark_position_dirty();

    if (pos->route_state == TRAIN_STATE_UNKNOWN) {
        /* assume train is on loop.
         * If the user type tr but the sys didn't recieve the
         * pos and dir, assumption still hold. just overwrite the speed.
         * auto-start at fixed speed 8 to acquire position and direction */

        pos_apply_loop_switches();
        pos->user_speed = GOTO_USER_SPEED;
        int can_spd = 1 + (pos->user_speed - 1) * 77;
        track_set_speed(train_num, can_spd);

        pos->effective_v     = speed_table_get_v(pos->train_ind, pos->user_speed);
        pos->cur_sensor_time = read_timer();
        pos->going_forward   = 1;
        pos->stable_sensor_count = 0;
        pos->route_state = TRAIN_STATE_LOOP_FIND_DIR;
        uint64_t dt = 0;
        pos->pred_next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
        pos->pred_trigger_time = pos->cur_sensor_time + dt;
        if (pos->pred_next_sensor != NULL && dt > 0) {
            uint64_t T2 = 0;
            predict_next_sensor(pos, pos->pred_next_sensor, &T2);
            pos->dead_track_deadline_us =
                pos->cur_sensor_time + DEAD_TRACK_DEADLINE_MULTIPLIER * (dt + T2);
        } else {
            pos->dead_track_deadline_us = 0;
        }

    } else if (pos->route_state == TRAIN_STATE_KNOWN) {
        int pred_on_loop = (pos->pred_next_sensor != NULL &&
                            (is_forward_loop_sensor(pos->pred_next_sensor) ||
                             is_reverse_loop_sensor(pos->pred_next_sensor)));
        if (pred_on_loop) {
            /* Train already running on the loop — skip stop/re-enter and
             * go straight to LOOP_STABILIZE. */
            pos_apply_loop_switches();
            pos->going_forward       = is_forward_loop_sensor(pos->pred_next_sensor) ? 1 : 0;
            pos->stable_sensor_count = 0;
            pos->route_state         = TRAIN_STATE_LOOP_STABILIZE;
            /* Adjust speed to fixed goto speed. */
            pos->user_speed  = GOTO_USER_SPEED;
            int can_spd = 1 + (pos->user_speed - 1) * 77;
            track_set_speed(pos->train_num, can_spd);
            int32_t cv = pos->cached_v[pos->user_speed];
            pos->effective_v = (cv > 0) ? cv
                                        : speed_table_get_v(pos->train_ind, pos->user_speed);
            pos->speed_warmup_mm = 400;
            /* Refresh prediction with new speed */
            uint64_t dt = 0;
            pos->pred_next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
            uint64_t now_goto      = read_timer();
            pos->pred_trigger_time = now_goto + dt;
            if (pos->pred_next_sensor != NULL && dt > 0) {
                uint64_t T2 = 0;
                predict_next_sensor(pos, pos->pred_next_sensor, &T2);
                pos->dead_track_deadline_us =
                    now_goto + DEAD_TRACK_DEADLINE_MULTIPLIER * (dt + T2);
            } else {
                pos->dead_track_deadline_us = 0;
            }
        } else {
            /* pos Known, running via tr.
             * Issue stop command now; pos_on_tick will call
             * transition_to_enter_loop() once physically stopped. */
            track_set_speed(train_num, 0);
            pos->stopping_since_us = read_timer();
            pos->route_state = TRAIN_STATE_STOPPING_GOTO;
        }

    } else if (pos->route_state == TRAIN_STATE_STOPPING_TR) {
        /* Train is already decelerating from a tr 0 command.
         * Redirect the post-stop action from STOPPED to ENTER_LOOP. */
        if (pos->user_speed == 0) pos->user_speed = GOTO_USER_SPEED;
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
            st == TRAIN_STATE_RECOVERY_STOPPING||
            st == TRAIN_STATE_DEAD_TRACK) ? 1 : 0;
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
