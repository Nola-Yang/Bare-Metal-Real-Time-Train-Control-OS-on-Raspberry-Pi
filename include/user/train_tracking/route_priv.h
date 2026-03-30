#ifndef _route_priv_h_
#define _route_priv_h_

#include <stdint.h>
#include "track_node.h"
#include "position.h"

/* Build the branch-node index cache. */
void route_init(void);

/* Return canonical sensors sorted by precomputed direct distance from start. */
int route_fill_sorted_direct_sensor_candidates(track_node *start,
                                               uint16_t *out_sensor_indices,
                                               int max_out);

/* Precomputed direct shortest-path distance from start to sensor. */
int32_t route_direct_sensor_dist(track_node *start, track_node *sensor);

/* ===== Distance / prediction ===== */

/* Follow the track from cur forward until reaching to.
 * Returns accumulated distance (mm), or -1 if not reached within max_hops. */
int32_t follow_dist(track_node *cur, track_node *to, int max_hops);

/* Return the first remaining sensor on pos->route_path from the active cursor,
 * or NULL when no sensor remains on the committed path. */
track_node *route_path_first_remaining_sensor(const train_pos_t *pos);

/* Infer which direction a branch should take for the train's current planned
 * path, falling back to the physical switch state when the next planned sensor
 * is ambiguous or unavailable. */
int route_branch_planned_dir(const train_pos_t *pos, track_node *branch);

/* Predict the next sensor after cur.  Writes dt (us) to *out_dt_us. */
track_node *predict_next_sensor(train_pos_t *pos, track_node *cur,
                                uint64_t *out_dt_us);

/* ===== Route planning constants ===== */

#define GOTO_MIN_DIST_FACTOR_DEFAULT 3
#define GOTO_MIN_DIST_FACTOR_SPEED_10 2
#define MIDREV_STOP_TOLERANCE_MM 50

static inline int route_goto_min_dist_factor(int goto_speed) {
    return (goto_speed == 10) ? GOTO_MIN_DIST_FACTOR_SPEED_10
                              : GOTO_MIN_DIST_FACTOR_DEFAULT;
}

/* ===== Route planning ===== */

/* Sum edge lengths in path[cursor..count-1].  Returns -1 on inconsistency. */
int32_t route_path_dist_from(const uint16_t *path, int cursor, int count);

/* Optimal route from start to target (or target->reverse), trying both
 * direct and single mid-route reversal.  d_brake is the minimum leg
 * length required for braking.  Returns 1 on success, 0 if unreachable.
 * plan->has_reversal is set to 1 when a reversal route is chosen. */
int  bfs_find_route_optimal(track_node *start, track_node *target,
                             int32_t d_brake, route_plan_t *plan);

/* Constrained optimal route (direct + one reversal) under blocked-node map. */
int  bfs_find_route_optimal_constrained(track_node *start, track_node *target,
                                        int32_t d_brake, int min_dist_factor,
                                        const uint8_t *blocked,
                                        const char *fixed_sw_dirs,
                                        route_plan_t *plan);

/* Bootstrap mid-route reversal for when no long-enough direct route exists.
 * Drives from start_rev to a far sensor F
 * (dist >= min_dist_factor * d_brake),
 * stops, reverses, then plans from F->reverse to target (dist >= d_brake).
 * Returns 1 and populates plan on success, 0 on failure. */
int  bfs_find_bootstrap_midrev(track_node *start_rev, track_node *target,
                                int32_t d_brake, int min_dist_factor,
                                const uint8_t *blocked,
                                const char *fixed_sw_dirs,
                                route_plan_t *plan);


#endif /* _route_priv_h_ */
