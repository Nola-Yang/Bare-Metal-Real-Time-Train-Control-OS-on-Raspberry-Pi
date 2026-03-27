#include "train_tracking/route_priv.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include <stddef.h>
#include <stdint.h>

/* ===== Branch node index cache ===== */

/* Indices of all NODE_BRANCH nodes in g_track[]. */
static int g_branch_idx[TRACK_MAX];
static int g_branch_count = 0;
static uint16_t g_canonical_sensor_idx[TRACK_MAX];
static int g_canonical_sensor_count = 0;
static uint16_t g_sorted_direct_sensor_idx[TRACK_MAX][TRACK_MAX];
static int g_sorted_direct_sensor_count[TRACK_MAX];
static int32_t g_direct_dist_matrix[TRACK_MAX][TRACK_MAX];

static int route_is_canonical_sensor(const track_node *node) {
    if (!node || node->type != NODE_SENSOR) return 0;
    if (!node->reverse || node->reverse->type != NODE_SENSOR) return 1;
    return node < node->reverse;
}

static void route_precompute_direct_sensor_cache(void);

void route_init(void) {
    g_branch_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type == NODE_BRANCH)
            g_branch_idx[g_branch_count++] = i;
    }
    route_precompute_direct_sensor_cache();
}

/* ===== Dijkstra shortest-path tables ===== */

#define DIJK_INF 0x7FFFFFFF

/*
 * Two independent Dijkstra tables:
 *   fwd_* — forward search (start → target, or start → reversal sensor)
 *   rev_* — reversal-leg search (reversal_sensor->reverse → target)
 *
 * Each table stores per-node shortest distance, finalized flag,
 * predecessor node index, and the switch that was set when entering
 * this node from its predecessor.
 */
static int32_t fwd_dist[TRACK_MAX];
static int8_t  fwd_done[TRACK_MAX];
static int16_t fwd_prev[TRACK_MAX];
static int16_t fwd_sw_num[TRACK_MAX];
static char    fwd_sw_dir[TRACK_MAX];

static int32_t rev_dist[TRACK_MAX];
static int8_t  rev_done[TRACK_MAX];
static int16_t rev_prev[TRACK_MAX];
static int16_t rev_sw_num[TRACK_MAX];
static char    rev_sw_dir[TRACK_MAX];

static route_plan_t g_opt_best_plan;
static route_plan_t g_opt_cand_plan;

/* ===== Dijkstra helpers ===== */

static void dijk_run(int32_t *dist, int8_t *done, int16_t *prev,
                     int16_t *sw_num, char *sw_dir,
                     const uint8_t *blocked, const char *fixed_sw_dirs,
                     int start_idx);

static void dijk_init(int32_t *dist, int8_t *done, int16_t *prev,
                      int16_t *sw_num, char *sw_dir, track_node *start) {
    for (int i = 0; i < TRACK_MAX; i++) {
        dist[i]   = DIJK_INF;
        done[i]   = 0;
        prev[i]   = -1;
        sw_num[i] = -1;
        sw_dir[i] = '?';
    }
    int si = (int)(start - g_track);
    if (si >= 0 && si < TRACK_MAX) dist[si] = 0;
}

/* Init and run Dijkstra from start in one call. */
static void dijk_run_from(int32_t *dist, int8_t *done, int16_t *prev,
                          int16_t *sw_num, char *sw_dir,
                          const uint8_t *blocked, const char *fixed_sw_dirs,
                          track_node *start) {
    dijk_init(dist, done, prev, sw_num, sw_dir, start);
    dijk_run(dist, done, prev, sw_num, sw_dir, blocked, fixed_sw_dirs,
             (int)(start - g_track));
}

/*
 * Dijkstra: only branch nodes (and the non-branch start) are ever
 * extracted as expansion candidates. When a branch is expanded, we chain-walk
 * forward through every non-branch node until the next branch, setting dist[]
 * and prev[] for all intermediate nodes along the way.
 */
static void dijk_run(int32_t *dist, int8_t *done, int16_t *prev,
                     int16_t *sw_num, char *sw_dir,
                     const uint8_t *blocked, const char *fixed_sw_dirs,
                     int start_idx) {
    for (;;) {
        int u = -1;
        int32_t min_d = DIJK_INF;

        if (g_track[start_idx].type != NODE_BRANCH &&
            !done[start_idx] && dist[start_idx] < min_d) {
            u = start_idx;
            min_d = dist[start_idx];
        }
        for (int bi = 0; bi < g_branch_count; bi++) {
            int i = g_branch_idx[bi];
            if (!done[i] && dist[i] < min_d) { u = i; min_d = dist[i]; }
        }
        if (u < 0) break;
        done[u] = 1;

        if (blocked && blocked[u] && u != start_idx) continue;

        track_node *n = &g_track[u];
        if (n->type == NODE_EXIT || n->type == NODE_NONE) continue;

        int dirs[2] = {DIR_AHEAD, DIR_AHEAD};
        int num_dirs = 1;
        if (n->type == NODE_BRANCH) {
            char fixed_dir = fixed_sw_dirs ? fixed_sw_dirs[u] : '?';
            if (fixed_dir == 'S') {
                dirs[0] = DIR_STRAIGHT;
            } else if (fixed_dir == 'C') {
                dirs[0] = DIR_CURVED;
            } else {
                dirs[0] = DIR_STRAIGHT;
                dirs[1] = DIR_CURVED;
                num_dirs = 2;
            }
        }
        for (int d = 0; d < num_dirs; d++) {
            int dir = (n->type == NODE_BRANCH) ? dirs[d] : DIR_AHEAD;
            track_edge *e = &n->edge[dir];
            if (!e->dest) continue;

            int16_t inh_sw_num = (n->type == NODE_BRANCH) ? (int16_t)n->num : -1;
            char    inh_sw_dir = (n->type == NODE_BRANCH)
                                     ? ((dir == DIR_STRAIGHT) ? 'S' : 'C') : '?';

            int32_t acc  = min_d;
            int     from = u;

            while (e && e->dest) {
                int v = (int)(e->dest - g_track);
                if (v < 0 || v >= TRACK_MAX) break;
                if (done[v]) break;
                if (blocked && blocked[v] && v != start_idx) break;

                acc += (int32_t)e->dist;
                if (acc < dist[v]) {
                    dist[v]   = acc;
                    prev[v]   = (int16_t)from;
                    sw_num[v] = inh_sw_num;
                    sw_dir[v] = inh_sw_dir;
                }

                track_node *vn = &g_track[v];
                if (vn->type == NODE_BRANCH || vn->type == NODE_EXIT) break;

                inh_sw_num = -1;
                inh_sw_dir = '?';
                from = v;
                e = &vn->edge[DIR_AHEAD];
            }
        }
    }
}

static void route_precompute_direct_sensor_cache(void) {
    g_canonical_sensor_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (route_is_canonical_sensor(&g_track[i])) {
            g_canonical_sensor_idx[g_canonical_sensor_count++] = (uint16_t)i;
        }
    }

    for (int start_idx = 0; start_idx < TRACK_MAX; start_idx++) {
        int count = 0;

        dijk_run_from(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir,
                      NULL, NULL, &g_track[start_idx]);
        for (int i = 0; i < TRACK_MAX; i++) {
            g_direct_dist_matrix[start_idx][i] = fwd_dist[i];
        }

        for (int ci = 0; ci < g_canonical_sensor_count; ci++) {
            uint16_t sensor_idx = g_canonical_sensor_idx[ci];
            int32_t dist = g_direct_dist_matrix[start_idx][sensor_idx];
            int insert_at = count;

            if (dist == DIJK_INF) continue;

            while (insert_at > 0) {
                uint16_t prev_idx =
                    g_sorted_direct_sensor_idx[start_idx][insert_at - 1];
                int32_t prev_dist = g_direct_dist_matrix[start_idx][prev_idx];

                if (prev_dist < dist ||
                    (prev_dist == dist && prev_idx < sensor_idx)) {
                    break;
                }
                g_sorted_direct_sensor_idx[start_idx][insert_at] =
                    g_sorted_direct_sensor_idx[start_idx][insert_at - 1];
                insert_at--;
            }

            g_sorted_direct_sensor_idx[start_idx][insert_at] = sensor_idx;
            count++;
        }

        g_sorted_direct_sensor_count[start_idx] = count;
    }
}

int route_fill_sorted_direct_sensor_candidates(track_node *start,
                                               uint16_t *out_sensor_indices,
                                               int max_out) {
    int start_idx;
    int count;

    if (!start) return 0;

    start_idx = (int)(start - g_track);
    if (start_idx < 0 || start_idx >= TRACK_MAX) return 0;

    count = g_sorted_direct_sensor_count[start_idx];
    if (max_out < 0) max_out = 0;
    for (int i = 0; out_sensor_indices && i < count && i < max_out; i++) {
        out_sensor_indices[i] = g_sorted_direct_sensor_idx[start_idx][i];
    }
    return count;
}

int32_t route_direct_sensor_dist(track_node *start, track_node *sensor) {
    int start_idx;
    int sensor_idx;
    int32_t dist;

    if (!start || !sensor) return -1;

    start_idx = (int)(start - g_track);
    sensor_idx = (int)(sensor - g_track);
    if (start_idx < 0 || start_idx >= TRACK_MAX ||
        sensor_idx < 0 || sensor_idx >= TRACK_MAX) {
        return -1;
    }

    dist = g_direct_dist_matrix[start_idx][sensor_idx];
    return (dist == DIJK_INF) ? -1 : dist;
}

/* Trace shortest path node indices from Dijkstra prev table to target.
 * Emits indices in forward order, including start and target. */
static int dijk_reconstruct_nodes(int16_t *prev, track_node *target,
                                  uint16_t *out_nodes, int *out_count,
                                  int max_nodes) {
    int tgt_idx = (int)(target - g_track);
    if (tgt_idx < 0 || tgt_idx >= TRACK_MAX || !out_nodes || !out_count || max_nodes <= 0) {
        return 0;
    }

    uint16_t tmp[TRACK_MAX];
    int top = 0;
    int idx = tgt_idx;
    while (idx >= 0 && idx < TRACK_MAX && top < TRACK_MAX) {
        tmp[top++] = (uint16_t)idx;
        idx = prev[idx];
    }
    if (top <= 0) return 0;

    *out_count = 0;
    for (int i = top - 1; i >= 0 && *out_count < max_nodes; i--) {
        out_nodes[*out_count] = tmp[i];
        (*out_count)++;
    }
    return (*out_count > 0);
}

/*
 * Trace shortest path from Dijkstra tables to `target`.
 * Fills out_sw_nums/out_sw_dirs/out_sw_count (forward order).
 * Returns total distance to target, or -1 if unreachable.
 */
static int32_t dijk_reconstruct(int32_t *dist, int16_t *prev,
                                int16_t *sw_num, char *sw_dir,
                                track_node *target,
                                int *out_sw_nums, char *out_sw_dirs,
                                int *out_sw_count, int max_sw) {
    int tgt_idx = (int)(target - g_track);
    if (tgt_idx < 0 || tgt_idx >= TRACK_MAX) return -1;
    if (dist[tgt_idx] == DIJK_INF) return -1;

    int   stk_num[20];
    char  stk_dir[20];
    int   stk_top = 0;
    int   idx = tgt_idx;
    while (prev[idx] >= 0 && stk_top < 20) {
        if (sw_num[idx] >= 0) {
            stk_num[stk_top] = sw_num[idx];
            stk_dir[stk_top] = sw_dir[idx];
            stk_top++;
        }
        idx = prev[idx];
    }

    *out_sw_count = 0;
    for (int s = stk_top - 1; s >= 0 && *out_sw_count < max_sw; s--) {
        out_sw_nums[*out_sw_count] = stk_num[s];
        out_sw_dirs[*out_sw_count] = stk_dir[s];
        (*out_sw_count)++;
    }
    return dist[tgt_idx];
}

static int32_t route_path_edge_dist(uint16_t from_idx, uint16_t to_idx) {
    track_node *from;
    track_node *to;

    if (from_idx >= TRACK_MAX || to_idx >= TRACK_MAX) return -1;

    from = &g_track[from_idx];
    to = &g_track[to_idx];
    if (from->type == NODE_BRANCH) {
        if (from->edge[DIR_STRAIGHT].dest == to) {
            return (int32_t)from->edge[DIR_STRAIGHT].dist;
        }
        if (from->edge[DIR_CURVED].dest == to) {
            return (int32_t)from->edge[DIR_CURVED].dist;
        }
        return -1;
    }
    if (from->edge[DIR_AHEAD].dest != to) return -1;
    return (int32_t)from->edge[DIR_AHEAD].dist;
}

static int32_t route_path_first_branch_dist(const uint16_t *path, int path_count) {
    int32_t total = 0;

    if (!path || path_count <= 1) return DIJK_INF;

    for (int i = 0; i < path_count - 1; i++) {
        int32_t edge_dist = route_path_edge_dist(path[i], path[i + 1]);
        if (edge_dist < 0) return -1;
        total += edge_dist;
        if (g_track[path[i + 1]].type == NODE_BRANCH) return total;
    }

    return DIJK_INF;
}

static int route_midrev_has_branch_clearance(const uint16_t *path, int path_count) {
    int32_t first_branch_dist = route_path_first_branch_dist(path, path_count);
    int32_t required_clearance = TRAIN_BODY_MM + MIDREV_STOP_TOLERANCE_MM;

    if (first_branch_dist < 0) return 0;
    if (first_branch_dist == DIJK_INF) return 1;

    /* When stopping to reverse, the whole train must already be clear of the
     * first turnout it will back into on the second leg. Include stop error so
     * a modest undershoot still leaves the full train clear of the switch. */
    return first_branch_dist >= required_clearance;
}

/* ===== Route planning (Dijkstra shortest distance) ===== */

int bfs_find_route_optimal(track_node *start, track_node *target,
                           int32_t d_brake, route_plan_t *plan) {
    return bfs_find_route_optimal_constrained(start, target, d_brake, NULL, NULL, plan);
}

int bfs_find_route_optimal_constrained(track_node *start, track_node *target,
                                       int32_t d_brake, const uint8_t *blocked,
                                       const char *fixed_sw_dirs,
                                       route_plan_t *plan) {
    if (!start || !target || !plan) return 0;

    track_node *tgts[2] = { target, target->reverse };

    int32_t best_total = DIJK_INF;
    route_plan_t *best = &g_opt_best_plan;
    route_plan_t *cand = &g_opt_cand_plan;
    best->sw_count     = 0;
    best->has_reversal = 0;
    int found = 0;

    for (int ti = 0; ti < 2; ti++) {
        track_node *tgt = tgts[ti];
        if (!tgt) continue;
        int tgt_idx = (int)(tgt - g_track);
        if (tgt_idx < 0 || tgt_idx >= TRACK_MAX) continue;

        /* Forward Dijkstra from start — fills fwd_* tables. */
        dijk_run_from(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir,
                      blocked, fixed_sw_dirs, start);

        if (fwd_dist[tgt_idx] < DIJK_INF) {
            int32_t d = fwd_dist[tgt_idx];
            if (d < best_total) {
                cand->has_reversal     = 0;
                cand->total_dist_mm    = d;
                cand->chosen_target    = tgt;
                dijk_reconstruct(fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                                 tgt, cand->sw_nums, cand->sw_dirs, &cand->sw_count, 20);
                if (!dijk_reconstruct_nodes(fwd_prev, tgt, cand->path_nodes,
                                            &cand->path_count, TRACK_MAX)) {
                    continue;
                }
                cand->path_count2 = 0;
                best_total = d;
                *best = *cand;
                found = 1;
            }
        }

        /* --- Reversal candidates: all sensors reachable with dist >= d_brake --- */
        for (int si = 0; si < TRACK_MAX; si++) {
            track_node *s = &g_track[si];
            if (s->type != NODE_SENSOR) continue;
            if (!s->reverse) continue;
            if (fwd_dist[si] == DIJK_INF) continue;

            if (fwd_dist[si] < GOTO_MIN_DIST_FACTOR * d_brake) continue;

            /* The continuation leg executes only after stopping at `s` and
             * reversing, so current self-owned fixed switch directions may no
             * longer apply by then. Revalidate the actual switch blockers when
             * the train reaches the midpoint. */
            dijk_run_from(rev_dist, rev_done, rev_prev, rev_sw_num, rev_sw_dir,
                          blocked, NULL, s->reverse);

            if (rev_dist[tgt_idx] == DIJK_INF) continue;
            if (rev_dist[tgt_idx] < GOTO_MIN_DIST_FACTOR * d_brake) continue;

            int32_t total = fwd_dist[si] + 2 * d_brake + rev_dist[tgt_idx];
            if (total >= best_total) continue;

            cand->has_reversal            = 1;
            cand->chosen_target           = tgt;
            cand->reversal_sensor         = s;
            cand->dist_to_reversal_mm     = fwd_dist[si];
            cand->dist_after_reversal_mm  = rev_dist[tgt_idx];
            cand->total_dist_mm           = total;

            /* First-leg switches (fwd_* still valid — not overwritten by rev_*). */
            dijk_reconstruct(fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                             s, cand->sw_nums, cand->sw_dirs, &cand->sw_count, 20);
            if (!dijk_reconstruct_nodes(fwd_prev, s, cand->path_nodes,
                                        &cand->path_count, TRACK_MAX)) {
                continue;
            }

            /* Second-leg switches. */
            dijk_reconstruct(rev_dist, rev_prev, rev_sw_num, rev_sw_dir,
                             tgt, cand->sw_nums2, cand->sw_dirs2, &cand->sw_count2, 20);
            if (!dijk_reconstruct_nodes(rev_prev, tgt, cand->path_nodes2,
                                        &cand->path_count2, TRACK_MAX)) {
                continue;
            }
            if (!route_midrev_has_branch_clearance(cand->path_nodes2,
                                                   cand->path_count2)) {
                continue;
            }

            best_total = total;
            *best = *cand;
            found = 1;
        }
    }

    if (!found) return 0;
    *plan = *best;
    return 1;
}

int bfs_find_bootstrap_midrev(track_node *start_rev, track_node *target,
                              int32_t d_brake, const uint8_t *blocked,
                              const char *fixed_sw_dirs,
                              route_plan_t *plan) {
    if (!start_rev || !target || !plan) return 0;

    int32_t threshold = (int32_t)GOTO_MIN_DIST_FACTOR * d_brake;

    dijk_run_from(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir,
                  blocked, fixed_sw_dirs, start_rev);

    track_node *tgts[2] = { target, target->reverse };

    int32_t best_total = DIJK_INF;
    int     found      = 0;

    for (int si = 0; si < TRACK_MAX; si++) {
        track_node *F = &g_track[si];
        if (F->type != NODE_SENSOR) continue;
        if (!F->reverse) continue;
        if (fwd_dist[si] == DIJK_INF) continue;
        if (fwd_dist[si] < threshold) continue;

        /* Bootstrap continuation runs only after the train retreats to `F`,
         * stops, and reverses, so do not pin it to the switch directions that
         * were fixed by the original stopped reservation. The midpoint resume
         * path still rechecks blockers before launch. */
        dijk_run_from(rev_dist, rev_done, rev_prev, rev_sw_num, rev_sw_dir,
                      blocked, NULL, F->reverse);

        for (int ti = 0; ti < 2; ti++) {
            track_node *tgt = tgts[ti];
            if (!tgt) continue;
            int tgt_idx = (int)(tgt - g_track);
            if (tgt_idx < 0 || tgt_idx >= TRACK_MAX) continue;
            if (rev_dist[tgt_idx] == DIJK_INF) continue;
            if (rev_dist[tgt_idx] < threshold) continue;

            int32_t total = fwd_dist[si] + rev_dist[tgt_idx];
            if (total >= best_total) continue;

            g_opt_cand_plan.has_reversal           = 1;
            g_opt_cand_plan.reversal_sensor        = F;
            g_opt_cand_plan.chosen_target          = tgt;
            g_opt_cand_plan.dist_to_reversal_mm    = fwd_dist[si];
            g_opt_cand_plan.dist_after_reversal_mm = rev_dist[tgt_idx];
            g_opt_cand_plan.total_dist_mm          = total;

            dijk_reconstruct(fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                             F, g_opt_cand_plan.sw_nums, g_opt_cand_plan.sw_dirs,
                             &g_opt_cand_plan.sw_count, 20);
            if (!dijk_reconstruct_nodes(fwd_prev, F, g_opt_cand_plan.path_nodes,
                                        &g_opt_cand_plan.path_count, TRACK_MAX))
                continue;

            dijk_reconstruct(rev_dist, rev_prev, rev_sw_num, rev_sw_dir,
                             tgt, g_opt_cand_plan.sw_nums2, g_opt_cand_plan.sw_dirs2,
                             &g_opt_cand_plan.sw_count2, 20);
            if (!dijk_reconstruct_nodes(rev_prev, tgt, g_opt_cand_plan.path_nodes2,
                                        &g_opt_cand_plan.path_count2, TRACK_MAX))
                continue;
            if (!route_midrev_has_branch_clearance(g_opt_cand_plan.path_nodes2,
                                                   g_opt_cand_plan.path_count2))
                continue;

            best_total = total;
            *plan      = g_opt_cand_plan;
            found      = 1;
        }
    }

    return found;
}
