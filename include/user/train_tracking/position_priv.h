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

/* WAIT_RESOURCE exponential backoff parameters.
 * wait = BASE * 2^min(retry, MAX_BACKOFF_STEPS) + jitter
 * where jitter is in [0, BASE). Max wait ~3.2s + jitter. */
#define REPLAN_INTERVAL_US    200000ULL   /* base interval (us) */
#define REPLAN_MAX_BACKOFF      4         /* cap exponent at 2^4 = 16x */



/* Attempt a direct on-route plan from the current stopped position to
 * pos->pending_target. Returns 1 and sets ON_ROUTE if route found; 0 otherwise. */
int pos_try_direct_goto(train_pos_t *pos);

/* Stop and wait for resources; pending_target remains unchanged for retries. */
void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us);

#endif /* _position_priv_h_ */
