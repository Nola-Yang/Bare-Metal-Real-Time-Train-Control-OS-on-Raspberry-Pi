#include "position.h"
#include "position_priv.h"
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


train_pos_t g_pos[MAX_POS_TRAINS];

int32_t SPEED_V_MM_S[15];
int32_t SPEED_STOP_DIST_MM[15];

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
            slot->stopping_since_us     = 0;
            slot->goto_speed            = 8;
            for (int s = 0; s < 15; s++) slot->cached_v[s] = 0;
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
 *   3. Restart the train at goto_speed.
 *   4. Refresh effective_v and next-sensor prediction.
 *   5. Set route_state = ENTER_LOOP.
 */
void transition_to_enter_loop(train_pos_t *pos, uint64_t now_us) {
    pos_apply_loop_switches();
    int just_reversed = 0;

    /* BFS to loop entry if not already on the loop */
    if (pos->cur_sensor != NULL &&
        !(is_forward_loop_sensor(pos->cur_sensor) ||
          is_reverse_loop_sensor(pos->cur_sensor))) {
        route_plan_t rp;
        if (bfs_find_route_to_loop(pos->cur_sensor, &rp)) {
            for (int j = 0; j < rp.sw_count; j++) {
                track_set_switch(rp.sw_nums[j], rp.sw_dirs[j]);
            }
            if (rp.sw_count > 0) {
                ui_mark_switches_dirty();
            }
        } else {
            // try reverse
            KASSERT(pos->cur_sensor->reverse != NULL &&
                    bfs_find_route_to_loop(pos->cur_sensor->reverse, &rp));
            track_reverse(pos->train_num);
            pos->cur_sensor = pos->cur_sensor->reverse;
            pos->going_forward = !pos->going_forward;
            just_reversed = 1;
            for (int j = 0; j < rp.sw_count; j++) {
                track_set_switch(rp.sw_nums[j], rp.sw_dirs[j]);
            }
            if (rp.sw_count > 0) {
                ui_mark_switches_dirty();
            }
        }
    }

    /* Restart the train */
    pos->user_speed = pos->goto_speed;
    int can_spd = 1 + (pos->user_speed - 1) * 77;
    track_set_speed(pos->train_num, can_spd);
    int32_t cv_loop      = pos->cached_v[pos->user_speed];
    pos->effective_v     = (cv_loop > 0) ? cv_loop : SPEED_V_MM_S[pos->user_speed];
    pos->cur_sensor_time = now_us;

    /* Prediction */
    if (pos->cur_sensor != NULL) {
        if (just_reversed) {
            /* The train is physically at cur_sensor's location after reversing. */
            pos->pred_next_sensor  = pos->cur_sensor;
            pos->pred_trigger_time = now_us;
        } else {
            uint64_t dt = 0;
            pos->pred_next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
            pos->pred_trigger_time = now_us + dt;
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
        pos->effective_v = (cv > 0) ? cv : SPEED_V_MM_S[user_speed];
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
#ifdef TRACK_A
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

int pos_goto(int train_num, track_node *target, int32_t offset_mm, int goto_speed) {
    KASSERT(target != NULL);
    if (!target) return 0;

    train_pos_t *pos = find_or_create_pos(train_num);
    KASSERT(pos != NULL);
    if (!pos) return 0;

    pos->goto_speed         = goto_speed;
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
         * auto-start at goto_speed to acquire position and direction */

        pos_apply_loop_switches();
        pos->user_speed = pos->goto_speed;
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
        if (pos->user_speed == 0) pos->user_speed = pos->goto_speed;
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