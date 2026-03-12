#ifndef _position_priv_h_
#define _position_priv_h_

#include <stdint.h>
#include "position.h"

/* ===== Shared internal state ===== */

/* Defined in position.c; all position-tracking modules share this array. */
extern train_pos_t g_pos[MAX_POS_TRAINS];

/* ===== Shared constants ===== */

/* Maximum path hops allowed when checking whether a sensor is reachable
 * from the predicted next sensor (off-route / skip detection). */
#define OFF_ROUTE_PATH_MAX_HOPS 120

/* Dead-track timeout multiplier relative to predicted sensor intervals. */
#define DEAD_TRACK_DEADLINE_MULTIPLIER 2

/* Fixed user speed used for all goto operations. */
#define GOTO_USER_SPEED 8

/* WAIT_RESOURCE periodic replan interval (us). */
#define REPLAN_INTERVAL_US 200000ULL



/* Drive a stationary train back onto the fixed loop and set route_state to
 * ENTER_LOOP. */
void transition_to_enter_loop(train_pos_t *pos, uint64_t now_us);

/* Attempt a direct on-route plan from the current stopped position to
 * pos->pending_target, skipping the loop-enter/stabilise sequence.
 * Returns 1 and sets ON_ROUTE if distance > 3x braking distance; 0 otherwise. */
int pos_try_direct_goto(train_pos_t *pos);

/* Stop and wait for resources; pending_target remains unchanged for retries. */
void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us);

#endif /* _position_priv_h_ */
