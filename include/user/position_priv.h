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



/* Drive a stationary train back onto the fixed loop and set route_state to
 * ENTER_LOOP. */
void transition_to_enter_loop(train_pos_t *pos, uint64_t now_us);

#endif /* _position_priv_h_ */