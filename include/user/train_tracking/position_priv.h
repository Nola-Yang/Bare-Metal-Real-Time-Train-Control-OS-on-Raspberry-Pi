#ifndef _position_priv_h_
#define _position_priv_h_

#include <stdint.h>
#include "position.h"
#include "pos_route_internal.h"
#include "speed_table.h"

/* ===== Shared internal state ===== */

/* Defined in position.c; all position-tracking modules share this array. */
extern train_pos_t g_pos[MAX_POS_TRAINS];

/* Find an existing train-position slot, or NULL if none exists. */
train_pos_t *pos_find_slot(int train_num);

/* Find or allocate a train-position slot for a valid train number. */
train_pos_t *pos_find_or_create_slot(int train_num, int speed_level);

/* ===== Shared constants ===== */

/* Maximum path hops allowed when checking whether a sensor is reachable
 * from the predicted next sensor (off-route / skip detection). */
#define OFF_ROUTE_PATH_MAX_HOPS 120

/* Dead-track timeout is prediction-relative, but never shorter than this floor. */
#define DEAD_TRACK_TIMEOUT_MIN_US 7000000ULL
#define DEAD_TRACK_TIMEOUT_MULTIPLIER 3ULL
#define DEAD_TRACK_RETRY_DELAY_US 10000000ULL
/* Fixed user speed used for all goto operations. */
#define GOTO_USER_SPEED 8

/* WAIT_RESOURCE exponential backoff parameters.
 * wait = BASE * 2^min(retry, MAX_BACKOFF_STEPS) + jitter
 * where jitter is in [0, BASE). Max wait is under 1s. */
#define REPLAN_INTERVAL_US    200000ULL   /* base interval (us) */
#define REPLAN_MAX_BACKOFF      2         /* cap exponent at 2^2 = 4x */

/* Keep resolved deadlock notices visible in the UI briefly after reroute. */
#define DEADLOCK_NOTICE_RESOLVED_US 8000000ULL
/* Wait up to this long for a strict deadlock reroute before forcing fallback. */
#define DEADLOCK_FALLBACK_TIMEOUT_US 7000000ULL
/* After a deadlock-yield stop, wait briefly before resuming the original target. */
#define DEADLOCK_RESUME_DELAY_US 1000000ULL

/* Give turnout commands a short settle window before launching the train. */
#define SWITCH_SETTLE_TICKS     5

/* Rolling reservation window tuning.
 * threshold    = GOTO_MIN_DIST_FACTOR * brake_dist + early_stop_dist
 * auth target  = first reachable sensor on the committed route whose
 *                distance is >= threshold
 * extend point = actual stop distance (brake + early stop) */
#define AUTH_TARGET_EXTRA_EARLY_STOP_US 500000ULL

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
 * pos->pending_target. Returns 1 when the request is accepted, either by
 * launching immediately or by entering WAIT_RESOURCE for retry. */
int pos_try_direct_goto(train_pos_t *pos);

/* Strict variant: planner-unreachable results fail without forcing the train
 * into WAIT_RESOURCE. */
int pos_try_direct_goto_strict(train_pos_t *pos);

/* Deadlock-timeout variant: only requires planner reachability and launches
 * immediately without entering WAIT_RESOURCE on reservation/switch conflicts. */
int pos_try_direct_goto_force_reachable(train_pos_t *pos);

/* Retry launching the already committed route for WAIT_RESOURCE resumes. */
int pos_try_resume_committed_route(train_pos_t *pos, uint64_t now_us);

/* Resume a WAIT_RESOURCE train according to its saved wait mode after
 * deadlock handling decides no reroute is needed. */
int pos_try_resume_wait_resource(train_pos_t *pos, uint64_t now_us);

/* Return 1 when `hit` lies on the alternate leg of the next predicted branch. */
int pos_hit_matches_alt_branch(const train_pos_t *pos, track_node *hit);

/* Advance the per-train sensor FSM after attribution selects an owner. */
void pos_handle_sensor_hit(train_pos_t *pos, track_node *hit, uint64_t time_us);

/* Restore a dead-track train into recovery if the current hit revives it. */
void pos_revive_dead_track_for_current_hit(train_pos_t *pos);

/* Update effective_v while the train is still in the acceleration ramp. */
void pos_update_accel_velocity(train_pos_t *pos, uint64_t now_us);

/* Pick the nearest safe deadlock-yield sensor target for the current stopped train.
 * Returns 1 and stores the chosen sensor in *out_target, 0 if none is available. */
int pos_pick_deadlock_yield_target(train_pos_t *pos, uint8_t cycle_mask,
                                   track_node **out_target,
                                   uint8_t *out_unblocked_mask,
                                   pos_deadlock_pick_kind_t *out_kind);

/* Pick a timeout-fallback target that is planner-reachable, preferring farther
 * deadlock sensor candidates over nearer ones. */
int pos_pick_deadlock_timeout_fallback_target(train_pos_t *pos,
                                              track_node **out_target);

/* Compute authority-window parameters from the train's braking model. */
int32_t pos_route_authority_stop_dist_mm(const train_pos_t *pos);
int32_t pos_route_authority_min_mm(const train_pos_t *pos);
int32_t pos_route_authority_target_mm(const train_pos_t *pos);
int32_t pos_route_authority_extend_trigger_mm(const train_pos_t *pos);
int32_t pos_route_authority_remaining_mm(const train_pos_t *pos);
int pos_route_authority_is_leg_goal_stop(const train_pos_t *pos);
static inline uint64_t pos_target_early_stop_us(const train_pos_t *pos) {
    uint64_t early_stop_us;

    if (!pos) return 0;
    early_stop_us = speed_table_get_early_stop(pos->train_ind, pos->goto_speed);
    if (!pos_route_authority_is_leg_goal_stop(pos)) {
        early_stop_us += AUTH_TARGET_EXTRA_EARLY_STOP_US;
    }
    return early_stop_us;
}
void pos_route_authority_reset(train_pos_t *pos);
void pos_route_authority_sync_target(train_pos_t *pos);
int pos_route_authority_prepare_launch(train_pos_t *pos, const route_plan_t *full_plan,
                                       route_plan_t *out_prefix,
                                       int *out_reserved_end_cursor,
                                       int *out_switch_blocker_owner,
                                       uint8_t *out_blocker_mask);
int pos_route_authority_try_top_up(train_pos_t *pos, uint64_t now_us, int force);

/* Stop and wait for resources; pending_target remains unchanged for retries. */
void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us, uint8_t blocker_mask,
                             pos_wait_mode_t wait_mode);

/* Clear or commit the active route path used by rolling reservations. */
void pos_clear_committed_route(train_pos_t *pos);
void pos_commit_route_plan(train_pos_t *pos, const route_plan_t *plan,
                           track_node *launch_origin, int need_initial_reverse,
                           int32_t final_offset_mm);

/* Seed a new goto request onto an existing train slot. */
void pos_prepare_goto_request(train_pos_t *pos, track_node *target, int speed_level, int32_t offset_mm);

/* Clear any active destination so FIND_POS can run without a planned target. */
void pos_prepare_find_pos_request(train_pos_t *pos);

/* Launch the train into FIND_POS/bootstrap from its current slot state. */
void pos_enter_find_pos(train_pos_t *pos, uint64_t now_us);

/* Zero all prediction fields (pred.* + dead_track_deadline_us). */
void pos_clear_prediction(train_pos_t *pos);

/* Refresh dead-track timeout using the current prediction when available,
 * otherwise fall back to the current sensor anchor (used after reversals). */
void pos_refresh_dead_track_deadline(train_pos_t *pos, uint64_t now_us);

/* Convert a predicted next-sensor interval into an absolute dead-track deadline. */
uint64_t pos_dead_track_deadline_from_interval(uint64_t now_us, uint64_t interval_us);

/* If pending_target is NULL but orig_user_target is set, restore it so the
 * next replan attempt can reach the original destination. */
void pos_restore_pending_target(train_pos_t *pos);

/* Pick the sensor-window end used by reservation release:
 * prefer `hint`, otherwise predict the next sensor after `last_hit`.
 * The kept window is the full reachable segment from `last_hit` to this
 * sensor, not just the two sensor endpoints. */
track_node *pos_release_keep_end(track_node *last_hit, track_node *hint);

/* Refresh reservations after a planned stop:
 * if the stop target was physically hit, keep the full contiguous segment
 * from that hit to the next reachable sensor in the original travel
 * direction; otherwise keep the remaining reserved route tail to that
 * target. */
void pos_refresh_stop_reservation(train_pos_t *pos);

/* Apply a route's switch commands only if every touched switch envelope is free. */
int pos_route_switch_blocker(const int *sw_nums, const char *sw_dirs,
                             int sw_count, int requester_train);

/* Apply a route's switch commands only if every touched switch envelope is free. */
int pos_apply_route_switches_safe(const int *sw_nums, const char *sw_dirs,
                                  int sw_count, int requester_train);

/* Set GOTO_USER_SPEED, send CAN speed command, restore effective_v from
 * cached_v (or speed table), set 400 mm warmup, and anchor cur_sensor_time. */
void pos_launch_at_goto_speed(train_pos_t *pos, uint64_t now_us);

/* Arm a deferred launch after issuing switch commands. */
void pos_arm_switch_settle(train_pos_t *pos, int sw_count,
                           pos_switch_settle_mode_t mode, uint64_t now_us);

/* Finish a deferred launch immediately. */
void pos_complete_switch_settle(train_pos_t *pos, uint64_t now_us);

/* If user_speed is in [1,14], save effective_v into cached_v, then zero
 * effective_v.  Call this whenever the train has fully stopped. */
void pos_save_ema_and_stop(train_pos_t *pos);

/* Clear any pending deadlock-resume chain on the train. */
void pos_clear_deadlock_recover(train_pos_t *pos);

/* Try to resume a yielded deadlock victim once the blocked peers have moved. */
int pos_deadlock_maybe_resume_after_yield(train_pos_t *pos, uint64_t now_us);
void pos_deadlock_on_tick(uint64_t now_us);
void pos_deadlock_refresh_notice_state(uint64_t now_us);
int pos_deadlock_maybe_reroute_waiter(train_pos_t *pos, uint64_t now_us);

/* Resume the stored second leg of a mid-route reversal after the stop completes. */
int pos_handle_midrev_resume(train_pos_t *pos, uint64_t now_us);

/* Internal deadlock notice state shared with the UI. */
void pos_set_deadlock_notice(const pos_deadlock_notice_t *notice);
void pos_clear_deadlock_notice(void);

void pos_reset_game_events(void);
void pos_publish_game_sensor_hit(train_pos_t *pos, track_node *hit, uint64_t time_us);
void pos_publish_game_goal_stop(train_pos_t *pos, track_node *target, uint64_t time_us);

#endif /* _position_priv_h_ */
