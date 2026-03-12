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

/* Re-send switch commands for known unreliable switches that appear in plan.
 * Current retry list: SW1, SW153, SW155, SW15. */
void resend_unreliable_switches(const int *sw_nums, const char *sw_dirs, int sw_count);

/* ===== Route planning constants ===== */

#define GOTO_MIN_DIST_FACTOR 6

/* ===== Route planning ===== */

/* Dijkstra shortest-distance route from start to target; fills plan.
 * Returns 1 on success, 0 if no path.  plan->has_reversal is always 0. */
int  bfs_find_route(track_node *start, track_node *target, route_plan_t *plan);

/* Optimal route from start to target (or target->reverse), trying both
 * direct and single mid-route reversal.  d_brake is the minimum leg
 * length required for braking.  Returns 1 on success, 0 if unreachable.
 * plan->has_reversal is set to 1 when a reversal route is chosen. */
int  bfs_find_route_optimal(track_node *start, track_node *target,
                             int32_t d_brake, route_plan_t *plan);

/* Dijkstra from start to the nearest fixed-loop sensor.
 * Returns 1 on success, 0 if no loop entry is reachable. */
int  bfs_find_route_to_loop(track_node *start, route_plan_t *plan);

/* ===== Deferred route execution ===== */

/* Execute the pending goto once the train has stabilised on the loop. */
void execute_pending_route(train_pos_t *pos);

#endif /* _route_priv_h_ */
