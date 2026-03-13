#ifndef _route_priv_h_
#define _route_priv_h_

#include <stdint.h>
#include "track_node.h"
#include "position.h"

/* ===== Distance / prediction ===== */

/* Follow the track from cur forward until reaching to.
 * Returns accumulated distance (mm), or -1 if not reached within max_hops. */
int32_t follow_dist(track_node *cur, track_node *to, int max_hops);

/* Predict the next sensor after cur.  Writes dt (us) to *out_dt_us. */
track_node *predict_next_sensor(train_pos_t *pos, track_node *cur,
                                uint64_t *out_dt_us);

/* Re-send switch commands for known unreliable switches that appear in plan.
 * Current retry list: SW1, SW153, SW155, SW15. */
void resend_unreliable_switches(const int *sw_nums, const char *sw_dirs, int sw_count);

/* ===== Route planning constants ===== */

#define GOTO_MIN_DIST_FACTOR 6

/* ===== Route planning ===== */

/* Dijkstra shortest-distance route from start to target; fills plan.
 * Returns 1 on success, 0 if no path.  plan->has_reversal is always 0. */
int  bfs_find_route(track_node *start, track_node *target, route_plan_t *plan);

/* Constrained shortest-distance route from start to target; blocked[i]=1
 * forbids entering node i (except the start node itself). */
int  bfs_find_route_constrained(track_node *start, track_node *target,
                                const uint8_t *blocked, route_plan_t *plan);

/* Optimal route from start to target (or target->reverse), trying both
 * direct and single mid-route reversal.  d_brake is the minimum leg
 * length required for braking.  Returns 1 on success, 0 if unreachable.
 * plan->has_reversal is set to 1 when a reversal route is chosen. */
int  bfs_find_route_optimal(track_node *start, track_node *target,
                             int32_t d_brake, route_plan_t *plan);

/* Constrained optimal route (direct + one reversal) under blocked-node map. */
int  bfs_find_route_optimal_constrained(track_node *start, track_node *target,
                                        int32_t d_brake, const uint8_t *blocked,
                                        route_plan_t *plan);


#endif /* _route_priv_h_ */
