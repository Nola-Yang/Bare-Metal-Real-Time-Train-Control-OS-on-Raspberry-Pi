/*
 * route.c — BFS route planning and speed
 *            prediction for the train position module.
 *
 * All functions here are called exclusively from position.c.
 */

#include "train_tracking/route_priv.h"
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

/* ===== Fixed loop sensor definitions ===== */

#define LOOP_SENSOR_COUNT_INTERNAL 10

/* Track-array indices of the 10 fixed-loop sensors (in loop order).
 * Both Track A and Track B share the same inner loop:
 *   A3=2, B15=30, C6=37, C13=44, C16=47, D7=54, D9=56, D11=58, E7=70, E12=75
 * Path: A3->BR14(S)->MR11->C13->E7->D7->MR9->BR8(S)->D9->E12->BR7(S)->D11->C16->MR6->C6->MR15->B15->A3
 */
static const int LOOP_SENSOR_IDX_A[LOOP_SENSOR_COUNT_INTERNAL] =
    { 2, 30, 37, 44, 47, 54, 56, 58, 70, 75 };
static const int LOOP_SENSOR_IDX_B[LOOP_SENSOR_COUNT_INTERNAL] =
    { 2, 30, 37, 44, 47, 54, 56, 58, 70, 75 };

/* Forward traversal order of the 10 loop sensors (clockwise on track):
 * A3(2)->C13(44)->E7(70)->D7(54)->D9(56)->E12(75)->D11(58)->C16(47)->C6(37)->B15(30)->A3
 */
static const int LOOP_SENSOR_FORWARD_ORDER_A[LOOP_SENSOR_COUNT_INTERNAL] =
    { 2, 44, 70, 54, 56, 75, 58, 47, 37, 30 };
static const int LOOP_SENSOR_FORWARD_ORDER_B[LOOP_SENSOR_COUNT_INTERNAL] =
    { 2, 44, 70, 54, 56, 75, 58, 47, 37, 30 };

static inline const int *loop_sensor_idx(void) {
#ifdef TRACK_D
    return LOOP_SENSOR_IDX_A;
#else
    return LOOP_SENSOR_IDX_B;
#endif
}

static inline const int *loop_sensor_forward_order(void) {
#ifdef TRACK_D
    return LOOP_SENSOR_FORWARD_ORDER_A;
#else
    return LOOP_SENSOR_FORWARD_ORDER_B;
#endif
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

/* ===== Dijkstra helpers ===== */

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

static void dijk_run(int32_t *dist, int8_t *done, int16_t *prev,
                     int16_t *sw_num, char *sw_dir) {
    for (;;) {
        int u = -1;
        int32_t min_d = DIJK_INF;
        for (int i = 0; i < TRACK_MAX; i++) {
            if (!done[i] && dist[i] < min_d) { u = i; min_d = dist[i]; }
        }
        if (u < 0) break;
        done[u] = 1;

        track_node *n = &g_track[u];
        if (n->type == NODE_EXIT || n->type == NODE_NONE) continue;

        if (n->type == NODE_BRANCH) {
            for (int d = 0; d <= 1; d++) {   
                track_edge *e = &n->edge[d];
                if (!e->dest) continue;
                int v = (int)(e->dest - g_track);
                if (v < 0 || v >= TRACK_MAX || done[v]) continue;
                int32_t nd = min_d + (int32_t)e->dist;
                if (nd < dist[v]) {
                    dist[v]   = nd;
                    prev[v]   = (int16_t)u;
                    sw_num[v] = (int16_t)n->num;
                    sw_dir[v] = (d == DIR_STRAIGHT) ? 'S' : 'C';
                }
            }
        } else {
            track_edge *e = &n->edge[DIR_AHEAD];
            if (!e->dest) continue;
            int v = (int)(e->dest - g_track);
            if (v < 0 || v >= TRACK_MAX || done[v]) continue;
            int32_t nd = min_d + (int32_t)e->dist;
            if (nd < dist[v]) {
                dist[v]   = nd;
                prev[v]   = (int16_t)u;
                sw_num[v] = -1;
                sw_dir[v] = '?';
            }
        }
    }
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

/*
 * Returns the minimum mm-distance from `start` to `target`, trying both
 * directions at every branch encountered.  Returns -1 if unreachable within
 * max_hops.  Used internally by observe_path_and_correct_switches.
 * max_hops should be kept small (≤40) to bound stack depth.
 */
static int32_t min_dist_to(track_node *start, track_node *target, int max_hops) {
    if (!start || max_hops <= 0) return -1;
    if (start == target) return 0;
    if (start->type == NODE_EXIT) return -1;

    if (start->type == NODE_BRANCH) {
        int32_t ds = -1, dc = -1;
        if (start->edge[DIR_STRAIGHT].dest) {
            int32_t r = min_dist_to(start->edge[DIR_STRAIGHT].dest, target, max_hops - 1);
            if (r >= 0) ds = (int32_t)start->edge[DIR_STRAIGHT].dist + r;
        }
        if (start->edge[DIR_CURVED].dest) {
            int32_t r = min_dist_to(start->edge[DIR_CURVED].dest, target, max_hops - 1);
            if (r >= 0) dc = (int32_t)start->edge[DIR_CURVED].dist + r;
        }
        if (ds < 0) return dc;
        if (dc < 0) return ds;
        return (ds <= dc) ? ds : dc;
    }

    if (!start->edge[DIR_AHEAD].dest) return -1;
    int32_t r = min_dist_to(start->edge[DIR_AHEAD].dest, target, max_hops - 1);
    if (r < 0) return -1;
    return (int32_t)start->edge[DIR_AHEAD].dist + r;
}

/* ===== Loop sensor membership ===== */

int is_loop_sensor(int track_idx) {
    for (int i = 0; i < LOOP_SENSOR_COUNT_INTERNAL; i++) {
        if (loop_sensor_idx()[i] == track_idx) return 1;
    }
    return 0;
}

int is_forward_loop_sensor(track_node *n) {
    if (!n) return 0;
    return is_loop_sensor((int)(n - g_track));
}

int is_reverse_loop_sensor(track_node *n) {
    if (!n || !n->reverse) return 0;
    return is_loop_sensor((int)(n->reverse - g_track));
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


int follow_reaches_loop(track_node *start, int max_hops) {
    if (!start) return 0;
    track_node *n = start;
    for (int h = 0; h < max_hops; h++) {
        if (n->type == NODE_SENSOR && is_loop_sensor((int)(n - g_track))) return 1;
        track_edge *e = get_next_edge(n);
        if (!e || !e->dest) return 0;
        n = e->dest;
        if (n->type == NODE_EXIT) return 0;
    }
    return 0;
}

track_node *predict_next_sensor(train_pos_t *pos, track_node *cur,
                                uint64_t *out_dt_us) {
    if (!cur || pos->effective_v <= 0) {
        if (out_dt_us) *out_dt_us = 0;
        return NULL;
    }

    track_node *n = cur;
    uint64_t total_us = 0;
    int hops = 0;

    for (;;) {
        track_edge *e = get_next_edge(n);
        if (!e || !e->dest) break;

        int32_t v = pos->effective_v;
        total_us += (uint64_t)((int64_t)e->dist * 1000000LL / v);

        n = e->dest;
        if (n->type == NODE_SENSOR) {
            if (out_dt_us) *out_dt_us = total_us;
            return n;
        }
        if (n->type == NODE_EXIT || ++hops > 80) break;
    }

    if (out_dt_us) *out_dt_us = 0;
    return NULL;
}

/* ===== Switch path analysis ===== */

void observe_path_and_correct_switches(track_node *from, track_node *to) {
    if (!from || !to || from == to) return;

    track_node *n = from;
    int hops = 0;

    while (n != to && hops < 40) {
        if (n->type == NODE_EXIT) break;

        if (n->type == NODE_BRANCH) {
            track_node *branch = n;

            int32_t ds = branch->edge[DIR_STRAIGHT].dest
                         ? min_dist_to(branch->edge[DIR_STRAIGHT].dest, to, 40) : -1;
            int32_t dc = branch->edge[DIR_CURVED].dest
                         ? min_dist_to(branch->edge[DIR_CURVED].dest,   to, 40) : -1;

            /* Both unreachable: give up. */
            if (ds < 0 && dc < 0) break;

            /* Both reachable with identical distance: genuinely ambiguous. */
            if (ds >= 0 && dc >= 0 && ds == dc) break;

            /* Pick the shorter (or only reachable) direction. */
            char actual_dir;
            track_node *next;
            if (dc < 0 || (ds >= 0 && ds < dc)) {
                actual_dir = 'S';
                next       = branch->edge[DIR_STRAIGHT].dest;
            } else {
                actual_dir = 'C';
                next       = branch->edge[DIR_CURVED].dest;
            }

            int sw_idx = track_switch_to_index(branch->num);
            if (sw_idx >= 0) {
                char stored = track_get_switch_state()[sw_idx].state;
                if (stored != actual_dir) {
                    track_update_switch(branch->num, actual_dir);
                    ui_mark_switches_dirty();
                }
            }
            n = next;
        } else if (n->type != NODE_EXIT) {
            if (!n->edge[DIR_AHEAD].dest) break;
            n = n->edge[DIR_AHEAD].dest;
        }
        hops++;
    }
}

/* ===== Route planning (Dijkstra shortest distance) ===== */

int bfs_find_route(track_node *start, track_node *target, route_plan_t *plan) {
    if (!start || !target || !plan) return 0;
    if (start == target) {
        plan->sw_count = 0;
        plan->loop_exit_branch = NULL;
        plan->total_dist_mm = 0;
        plan->has_reversal = 0;
        return 1;
    }

    dijk_init(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir, start);
    dijk_run(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir);

    int32_t d = dijk_reconstruct(fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                                  target,
                                  plan->sw_nums, plan->sw_dirs,
                                  &plan->sw_count, 20);
    if (d < 0) return 0;
    plan->total_dist_mm = d;
    plan->loop_exit_branch = NULL;
    plan->has_reversal = 0;
    plan->chosen_target = NULL;
    return 1;
}


int bfs_find_route_optimal(track_node *start, track_node *target,
                            int32_t d_brake, route_plan_t *plan) {
    if (!start || !target || !plan) return 0;

    track_node *tgts[2] = { target, target->reverse };

    int32_t    best_total = DIJK_INF;
    route_plan_t best;
    best.sw_count     = 0;
    best.has_reversal = 0;
    int found = 0;

    for (int ti = 0; ti < 2; ti++) {
        track_node *tgt = tgts[ti];
        if (!tgt) continue;
        int tgt_idx = (int)(tgt - g_track);
        if (tgt_idx < 0 || tgt_idx >= TRACK_MAX) continue;

        /* Forward Dijkstra from start — fills fwd_* tables. */
        dijk_init(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir, start);
        dijk_run(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir);

        if (fwd_dist[tgt_idx] < DIJK_INF) {
            int32_t d = fwd_dist[tgt_idx];
            if (d < best_total) {
                route_plan_t rp;
                rp.loop_exit_branch = NULL;
                rp.has_reversal     = 0;
                rp.total_dist_mm    = d;
                rp.chosen_target    = tgt;
                dijk_reconstruct(fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                                  tgt, rp.sw_nums, rp.sw_dirs, &rp.sw_count, 20);
                best_total = d;
                best  = rp;
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
            dijk_init(rev_dist, rev_done, rev_prev, rev_sw_num, rev_sw_dir,
                      s->reverse);
            dijk_run(rev_dist, rev_done, rev_prev, rev_sw_num, rev_sw_dir);

            if (rev_dist[tgt_idx] == DIJK_INF) continue;
            if (rev_dist[tgt_idx] < GOTO_MIN_DIST_FACTOR * d_brake) continue;

            int32_t total = fwd_dist[si] + 2 * d_brake + rev_dist[tgt_idx];
            if (total >= best_total) continue;

            route_plan_t rp;
            rp.loop_exit_branch        = NULL;
            rp.has_reversal            = 1;
            rp.chosen_target           = tgt;
            rp.reversal_sensor         = s;
            rp.dist_to_reversal_mm     = fwd_dist[si];
            rp.dist_after_reversal_mm  = rev_dist[tgt_idx];
            rp.total_dist_mm           = total;

            /* First-leg switches (fwd_* still valid — not overwritten by rev_*). */
            dijk_reconstruct(fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                              s, rp.sw_nums, rp.sw_dirs, &rp.sw_count, 20);

            /* Second-leg switches. */
            dijk_reconstruct(rev_dist, rev_prev, rev_sw_num, rev_sw_dir,
                              tgt, rp.sw_nums2, rp.sw_dirs2, &rp.sw_count2, 20);

            best_total = total;
            best  = rp;
            found = 1;
        }
    }

    if (!found) return 0;
    *plan = best;
    return 1;
}


int bfs_find_route_to_loop(track_node *start, route_plan_t *plan) {
    if (!start || !plan) return 0;
    for (int i = 0; i < LOOP_SENSOR_COUNT_INTERNAL; i++) {
        track_node *loop_entry = &g_track[loop_sensor_idx()[i]];
        if (bfs_find_route(start, loop_entry, plan)) {
            return 1;
        }
    }
    return 0;
}

/* ===== Deferred route execution ===== */

void execute_pending_route(train_pos_t *pos) {
    if (!pos->pending_target) return;

    track_node *user_target = pos->pending_target;
    int32_t     offset      = pos->pending_offset_mm;

    track_node *plan_start = (pos->pred_next_sensor && pos->effective_v > 0)
                             ? pos->pred_next_sensor : pos->cur_sensor;
    if (!plan_start) KASSERT(0 && "No position anchor for route planning");

    if (!(is_forward_loop_sensor(plan_start) ||
          is_reverse_loop_sensor(plan_start))) return;

    track_node *ps_fwd = is_forward_loop_sensor(plan_start) ? plan_start
                                                             : plan_start->reverse;
    int ps_idx    = (int)(ps_fwd - g_track);
    const int *order = loop_sensor_forward_order();
    int start_pos = 0;
    for (int i = 0; i < LOOP_SENSOR_COUNT_INTERNAL; i++) {
        if (order[i] == ps_idx) { start_pos = i; break; }
    }
    
    int search_start = (start_pos + 1) % LOOP_SENSOR_COUNT_INTERNAL;

    route_plan_t rp;
    track_node  *target = NULL;
    track_node  *loop_plan_start = NULL;

    /* Try user_target first, then its reverse node.
     * For each candidate target, iterate through all loop sensors in the
     * direction of travel starting from the sensor after plan_start. */
    track_node *try_targets[2] = { user_target, user_target->reverse };

    for (int t = 0; t < 2 && !target; t++) {
        track_node *tgt = try_targets[t];
        if (!tgt) continue;

        for (int i = 0; i < LOOP_SENSOR_COUNT_INTERNAL; i++) {
            track_node *cand;
            if (pos->going_forward) {
                int ci = order[(search_start + i) % LOOP_SENSOR_COUNT_INTERNAL];
                cand = &g_track[ci];
            } else {
                int ci = order[(search_start + LOOP_SENSOR_COUNT_INTERNAL - i)
                               % LOOP_SENSOR_COUNT_INTERNAL];
                cand = g_track[ci].reverse;
            }
            if (!cand) continue;
            if (bfs_find_route(cand, tgt, &rp)) {
                target = tgt;
                loop_plan_start = cand;
                break;
            }
        }
    }

    KASSERT(target && loop_plan_start &&
            "No loop sensor reaches target or target_reverse");
    /* Apply route switches from far to near along the path. */
    for (int i = rp.sw_count - 1; i >= 0; i--) {
        track_set_switch(rp.sw_nums[i], rp.sw_dirs[i]);
        track_update_switch(rp.sw_nums[i], rp.sw_dirs[i]); 
    }
    resend_unreliable_switches(rp.sw_nums, rp.sw_dirs, rp.sw_count);

    if (rp.sw_count > 0) {
        ui_mark_switches_dirty();
    }

    pos->target_sensor    = target;
    pos->target_offset_mm = offset;
    pos->last_plan_valid      = 1;
    pos->last_plan_loop_start = loop_plan_start;
    pos->last_plan_target     = target;
    pos->last_plan_sw_count   = rp.sw_count;
    for (int i = 0; i < rp.sw_count; i++) {
        pos->last_plan_sw_nums[i] = rp.sw_nums[i];
        pos->last_plan_sw_dirs[i] = rp.sw_dirs[i];
    }
    pos->offroute_valid           = 0;
    pos->offroute_expected_sensor = NULL;
    pos->offroute_actual_sensor   = NULL;

    track_node *cur = pos->cur_sensor;
    if (cur) {
        int32_t dist_from_cur = follow_dist(cur, target, 200);
        if (dist_from_cur >= 0) {
            int32_t already_gone = 0;
            if (pos->effective_v > 0 && pos->cur_sensor_time > 0) {
                uint64_t now_us = read_timer();
                uint64_t dt     = now_us - pos->cur_sensor_time;
                already_gone    = (int32_t)(
                    (int64_t)pos->effective_v * (int64_t)dt / 1000000LL);
            }
            int32_t rem = dist_from_cur + offset - already_gone;
            pos->dist_to_target_mm = (rem > 0) ? rem : 0;
        }
    }

    pos->pending_target      = NULL;
    pos->pending_offset_mm   = 0;
    pos->stable_sensor_count = 0;
    pos->route_state         = TRAIN_STATE_ON_ROUTE;

    /* Refresh prediction with the newly set route switches.
     *
     * Must predict from cur_sensor, NOT from plan_start.
     * and uses cur_sensor_time as the absolute time anchor. */
    uint64_t dt = 0;
    pos->pred_next_sensor  = predict_next_sensor(pos, pos->cur_sensor, &dt);
    pos->pred_trigger_time = pos->cur_sensor_time + dt;

    ui_mark_position_dirty();
}

