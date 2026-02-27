#ifndef _route_priv_h_
#define _route_priv_h_

#include <stdint.h>
#include "track_node.h"
#include "position.h"

/* ===== Loop sensor membership ===== */

/* True if track-array index belongs to the fixed loop. */
int  is_loop_sensor(int track_idx);

/* True if n is a forward-direction loop sensor. */
int  is_forward_loop_sensor(track_node *n);

/* True if n is the reverse-direction counterpart of a loop sensor. */
int  is_reverse_loop_sensor(track_node *n);

/* ===== Distance / prediction ===== */

/* Follow the track from cur forward until reaching to.
 * Returns accumulated distance (mm), or -1 if not reached within max_hops. */
int32_t follow_dist(track_node *cur, track_node *to, int max_hops);

/* Returns 1 if a fixed-loop sensor is reachable from start by following
 * current switch states within max_hops, 0 otherwise. */
int follow_reaches_loop(track_node *start, int max_hops);

/* Predict the next sensor after cur.  Writes dt (us) to *out_dt_us. */
track_node *predict_next_sensor(train_pos_t *pos, track_node *cur,
                                uint64_t *out_dt_us);

/* ===== Switch path analysis ===== */

/* Trace path from to; observe actual switch directions; correct stored state. */
void observe_path_and_correct_switches(track_node *from, track_node *to);

/* Walk the path from `from` to `to` and
 * update each edge's time_factor_q8 via EMA using ratio_q8 (Q8 fixed-point
 * ratio = actual_dt * 256 / pred_dt).  Clamps factor to [128, 512]. */
void update_edge_factors(track_node *from, track_node *to, int32_t ratio_q8);

/* ===== BFS route planning ===== */

/* BFS from start to target; fills plan.  Returns 1 on success, 0 if no path. */
int  bfs_find_route(track_node *start, track_node *target, route_plan_t *plan);

/* BFS from start to the nearest fixed-loop sensor.
 * Returns 1 on success, 0 if no loop entry is reachable. */
int  bfs_find_route_to_loop(track_node *start, route_plan_t *plan);

/* ===== Deferred route execution ===== */

/* Execute the pending goto once the train has stabilised on the loop. */
void execute_pending_route(train_pos_t *pos);

#endif /* _route_priv_h_ */
