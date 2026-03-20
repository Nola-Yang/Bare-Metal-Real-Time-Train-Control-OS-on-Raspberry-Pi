#ifndef _position_priv_h_
#define _position_priv_h_

#include <stdint.h>
#include "position.h"
#include "speed_table.h"

/* ===== Shared internal state ===== */

/* Defined in position.c; all position-tracking modules share this array. */
extern train_pos_t g_pos[MAX_POS_TRAINS];

/* stop-command lead time per train (microseconds). */
extern uint64_t STOP_EARLY_US[MAX_PHYSICAL_TRAINS];

/* ===== Shared constants ===== */

/* Maximum path hops allowed when checking whether a sensor is reachable
 * from the predicted next sensor (off-route / skip detection). */
#define OFF_ROUTE_PATH_MAX_HOPS 120

/* Dead-track timeout multiplier relative to the next predicted sensor interval. */
#define DEAD_TRACK_TIMEOUT 10000000

/* Fixed user speed used for all goto operations. */
#define GOTO_USER_SPEED 8

/* Physical train body length (mm). */
#define TRAIN_BODY_MM 200

/* WAIT_RESOURCE exponential backoff parameters.
 * wait = BASE * 2^min(retry, MAX_BACKOFF_STEPS) + jitter
 * where jitter is in [0, BASE). Max wait ~3.2s + jitter. */
#define REPLAN_INTERVAL_US    200000ULL   /* base interval (us) */
#define REPLAN_MAX_BACKOFF      4         /* cap exponent at 2^4 = 16x */

/* Keep resolved deadlock notices visible in the UI briefly after reroute. */
#define DEADLOCK_NOTICE_RESOLVED_US 8000000ULL

/* Give turnout commands a short settle window before launching the train. */
#define SWITCH_SETTLE_TICKS     5

/* Train order used for deadlock blocker masks. */
static inline int pos_deadlock_train_to_index(int train_num) {
    switch (train_num) {
    case 13: return 0;
    case 14: return 1;
    case 15: return 2;
    case 17: return 3;
    case 18: return 4;
    case 55: return 5;
    default: return -1;
    }
}

static inline int pos_deadlock_index_to_train(int index) {
    switch (index) {
    case 0: return 13;
    case 1: return 14;
    case 2: return 15;
    case 3: return 17;
    case 4: return 18;
    case 5: return 55;
    default: return -1;
    }
}

static inline uint8_t pos_deadlock_train_bit(int train_num) {
    int idx = pos_deadlock_train_to_index(train_num);
    return (idx >= 0) ? (uint8_t)(1u << idx) : 0;
}

/* Attempt a direct on-route plan from the current stopped position to
 * pos->pending_target. Returns 1 and sets ON_ROUTE if route found; 0 otherwise. */
int pos_try_direct_goto(train_pos_t *pos);

/* Pick the nearest safe deadlock-yield sensor target for the current stopped train.
 * Returns 1 and stores the chosen sensor in *out_target, 0 if none is available. */
int pos_pick_deadlock_yield_target(train_pos_t *pos, uint8_t cycle_mask,
                                   track_node **out_target, uint8_t *out_unblocked_mask);

/* Stop and wait for resources; pending_target remains unchanged for retries. */
void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us, uint8_t blocker_mask);

/* Zero all prediction fields (pred.* + dead_track_deadline_us). */
void pos_clear_prediction(train_pos_t *pos);

/* Refresh dead-track timeout using the current prediction when available,
 * otherwise fall back to the current sensor anchor (used after reversals). */
void pos_refresh_dead_track_deadline(train_pos_t *pos, uint64_t now_us);

/* If pending_target is NULL but orig_user_target is set, restore it so the
 * next replan attempt can reach the original destination. */
void pos_restore_pending_target(train_pos_t *pos);

/* Pick the sensor-window end used by reservation release:
 * prefer `hint`, otherwise predict the next sensor after `last_hit`. */
track_node *pos_release_keep_end(track_node *last_hit, track_node *hint);

/* Apply a route's switch commands only if every touched switch envelope is free. */
int pos_route_switch_blocker(const int *sw_nums, const char *sw_dirs,
                             int sw_count, int requester_train);

/* Apply a route's switch commands only if every touched switch envelope is free. */
int pos_apply_route_switches_safe(const int *sw_nums, const char *sw_dirs,
                                  int sw_count, int requester_train);

/* Set GOTO_USER_SPEED, send CAN speed command, restore effective_v from
 * cached_v (or speed table), set 400 mm warmup, and anchor cur_sensor_time. */
void pos_launch_at_goto_speed(train_pos_t *pos, uint64_t now_us);

/* Sleep briefly after issuing switch commands so the turnout can settle. */
void pos_wait_switch_settle(int sw_count);

/* If user_speed is in [1,14], save effective_v into cached_v, then zero
 * effective_v.  Call this whenever the train has fully stopped. */
void pos_save_ema_and_stop(train_pos_t *pos);

/* Clear any pending deadlock-resume chain on the train. */
void pos_clear_deadlock_recover(train_pos_t *pos);

/* Internal deadlock notice state shared with the UI. */
void pos_set_deadlock_notice(const pos_deadlock_notice_t *notice);
void pos_clear_deadlock_notice(void);

#endif /* _position_priv_h_ */
