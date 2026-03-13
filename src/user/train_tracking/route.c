/*
 * route.c — BFS route planning and speed
 *            prediction for the train position module.
 *
 * All functions here are called exclusively from position.c.
 */

#include "train_tracking/route_priv.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "timer.h"
#include "ui.h"
#include "kassert.h"
#include <stddef.h>
#include <stdint.h>

/* ===== Unreliable switch reliability helper ===== */

void resend_unreliable_switches(const int *sw_nums, const char *sw_dirs, int sw_count) {
    for (int i = 0; i < sw_count; i++) {
        if (sw_nums[i] == 1 || sw_nums[i] == 153 || sw_nums[i] == 155 || sw_nums[i] == 15) {
            track_set_switch(sw_nums[i], sw_dirs[i]);
        }
    }
}


/* ===== Branch node index cache ===== */

/* Indices of all NODE_BRANCH nodes in g_track[]. */
static int g_branch_idx[TRACK_MAX];
static int g_branch_count = 0;

void route_init(void) {
    g_branch_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type == NODE_BRANCH)
            g_branch_idx[g_branch_count++] = i;
    }
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
                     const uint8_t *blocked, int start_idx);

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
                          const uint8_t *blocked, track_node *start) {
    dijk_init(dist, done, prev, sw_num, sw_dir, start);
    dijk_run(dist, done, prev, sw_num, sw_dir, blocked, (int)(start - g_track));
}

/*
 * Dijkstra: only branch nodes (and the non-branch start) are ever
 * extracted as expansion candidates.  When a branch is expanded, we chain-walk
 * forward through every non-branch node until the next branch, setting dist[]
 * and prev[] for all intermediate nodes along the way.
 */
static void dijk_run(int32_t *dist, int8_t *done, int16_t *prev,
                     int16_t *sw_num, char *sw_dir,
                     const uint8_t *blocked, int start_idx) {
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

        int num_dirs = (n->type == NODE_BRANCH) ? 2 : 1;
        for (int d = 0; d < num_dirs; d++) {
            int dir = (n->type == NODE_BRANCH) ? d : DIR_AHEAD;
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

/* ===== Other static helpers ===== */

/* Return the edge to follow from node n using current switch states. */
static track_edge *get_next_edge(track_node *n) {
    if (!n) return NULL;
    switch (n->type) {
    case NODE_SENSOR:
    case NODE_MERGE:
    case NODE_ENTER:
        return &n->edge[DIR_AHEAD];
    case NODE_BRANCH: {
        int idx = track_switch_to_index(n->num);
        char state = '?';
        if (idx >= 0) {
            state = track_get_switch_state()[idx].state;
        }
        KASSERT(state == 'C' || state == 'S');
        int dir = (state == 'C') ? DIR_CURVED : DIR_STRAIGHT;
        return &n->edge[dir];
    }
    default:
        return NULL;
    }
}

/* ===== Distance / prediction ===== */

int32_t follow_dist(track_node *cur, track_node *to, int max_hops) {
    if (!cur || !to) return -1;
    if (cur == to) return 0;
    int32_t dist = 0;
    for (int h = 0; h < max_hops; h++) {
        track_edge *e = get_next_edge(cur);
        if (!e || !e->dest) return -1;
        dist += (int32_t)e->dist;
        cur = e->dest;
        if (cur == to) return dist;
        if (cur->type == NODE_EXIT) return -1;
    }
    return -1;
}


/* Walk forward from start and return the first SENSOR node, or NULL. */
static track_node *first_sensor_forward(track_node *start, int max_hops) {
    if (!start) return NULL;
    track_node *n = start;
    for (int h = 0; h < max_hops; h++) {
        if (n->type == NODE_SENSOR) return n;
        if (n->type == NODE_EXIT)   return NULL;
        track_edge *e = get_next_edge(n);
        if (!e || !e->dest) return NULL;
        n = e->dest;
    }
    return (n->type == NODE_SENSOR) ? n : NULL;
}

track_node *predict_next_sensor(train_pos_t *pos, track_node *cur,
                                uint64_t *out_dt_us) {
    if (!cur) {
        if (out_dt_us) *out_dt_us = 0;
        if (pos) { pos->pred_alt_sensor = NULL; pos->pred_branch_node = NULL; }
        return NULL;
    }

    track_node *n = cur;
    uint64_t total_us = 0;
    int hops = 0;
    int found_branch = 0;

    for (;;) {
        /* At the first branch: record the alternate-direction sensor. */
        if (!found_branch && n->type == NODE_BRANCH) {
            found_branch = 1;
            if (pos) {
                int sw_idx = track_switch_to_index(n->num);
                char st = (sw_idx >= 0) ? track_get_switch_state()[sw_idx].state : '?';
                int alt_dir = (st == 'S') ? DIR_CURVED : DIR_STRAIGHT;
                pos->pred_alt_sensor  = first_sensor_forward(n->edge[alt_dir].dest, 20);
                pos->pred_branch_node = n;
            }
        }

        track_edge *e = get_next_edge(n);
        if (!e || !e->dest) break;

        /* Only accumulate timing when speed is known; dt=0 means timing unknown. */
        if (pos->effective_v > 0) {
            int32_t v = pos->effective_v;
            total_us += (uint64_t)((int64_t)e->dist * 1000000LL / v);
        }

        n = e->dest;
        if (n->type == NODE_SENSOR) {
            if (out_dt_us) *out_dt_us = total_us;
            return n;
        }
        if (n->type == NODE_EXIT || ++hops > 80) break;
    }

    if (out_dt_us) *out_dt_us = 0;
    if (!found_branch && pos) { pos->pred_alt_sensor = NULL; pos->pred_branch_node = NULL; }
    return NULL;
}

/* ===== Route planning (Dijkstra shortest distance) ===== */

int bfs_find_route_constrained(track_node *start, track_node *target,
                               const uint8_t *blocked, route_plan_t *plan) {
    if (!start || !target || !plan) return 0;
    if (start == target) {
        plan->sw_count = 0;
        plan->total_dist_mm = 0;
        plan->has_reversal = 0;
        plan->chosen_target = target;
        plan->path_count = 1;
        plan->path_nodes[0] = (uint16_t)(start - g_track);
        plan->path_count2 = 0;
        return 1;
    }

    dijk_run_from(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir, blocked, start);

    int32_t d = dijk_reconstruct(fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                                  target,
                                  plan->sw_nums, plan->sw_dirs,
                                  &plan->sw_count, 20);
    if (d < 0) return 0;
    if (!dijk_reconstruct_nodes(fwd_prev, target, plan->path_nodes,
                                &plan->path_count, TRACK_MAX)) {
        return 0;
    }
    plan->total_dist_mm = d;
    plan->has_reversal = 0;
    plan->chosen_target = target;
    plan->path_count2 = 0;
    return 1;
}


int bfs_find_route_optimal(track_node *start, track_node *target,
                            int32_t d_brake, route_plan_t *plan) {
    return bfs_find_route_optimal_constrained(start, target, d_brake, NULL, plan);
}

int bfs_find_route_optimal_constrained(track_node *start, track_node *target,
                                       int32_t d_brake, const uint8_t *blocked,
                                       route_plan_t *plan) {
    if (!start || !target || !plan) return 0;

    track_node *tgts[2] = { target, target->reverse };

    int32_t    best_total = DIJK_INF;
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
        dijk_run_from(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir, blocked, start);

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

            /* Dijkstra from s->reverse — fills rev_* tables. */
            dijk_run_from(rev_dist, rev_done, rev_prev, rev_sw_num, rev_sw_dir, blocked, s->reverse);

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

            best_total = total;
            *best = *cand;
            found = 1;
        }
    }

    if (!found) return 0;
    *plan = *best;
    return 1;
}
