/*
 * route.c — BFS route planning and speed
 *            prediction for the train position module.
 *
 * All functions here are called exclusively from position.c.
 */

#include "route_priv.h"
#include "track.h"
#include "track_data.h"
#include "timer.h"
#include "ui.h"
#include "kassert.h"
#include <stddef.h>
#include <stdint.h>

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
#ifdef TRACK_A
    return LOOP_SENSOR_IDX_A;
#else
    return LOOP_SENSOR_IDX_B;
#endif
}

static inline const int *loop_sensor_forward_order(void) {
#ifdef TRACK_A
    return LOOP_SENSOR_FORWARD_ORDER_A;
#else
    return LOOP_SENSOR_FORWARD_ORDER_B;
#endif
}

/* ===== BFS data ===== */

#define BFS_QUEUE_MAX 256

typedef struct {
    track_node *node;
    int16_t     parent;    /* index in queue; -1 for root */
    int16_t     sw_num;    /* user switch number (-1 = no switch on this hop) */
    char        sw_dir;
    char        _pad[3];
} bfs_entry_t;

static bfs_entry_t bfs_q[BFS_QUEUE_MAX];
static uint8_t     bfs_visited[TRACK_MAX];

/* ===== Static helpers (internal only) ===== */

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
        int dir = (state == 'C') ? DIR_CURVED : DIR_STRAIGHT;
        return &n->edge[dir];
    }
    default:
        return NULL;
    }
}

/*
 * Returns 1 if `target` is reachable from `start` within max_hops.
 * Intermediate sensor nodes are treated as ordinary waypoints 
 */
static int reaches_target(track_node *start, track_node *target, int max_hops) {
    if (!start) return 0;
    if (start == target) return 1;
    if (max_hops <= 0) return 0;
    if (start->type == NODE_EXIT) return 0;

    if (start->type == NODE_BRANCH) {
        return (reaches_target(start->edge[DIR_STRAIGHT].dest,
                               target, max_hops - 1) ||
                reaches_target(start->edge[DIR_CURVED].dest,
                               target, max_hops - 1));
    }
    if (!start->edge[DIR_AHEAD].dest) return 0;
    return reaches_target(start->edge[DIR_AHEAD].dest, target, max_hops - 1);
}

static void bfs_enqueue(int *tail, track_node *node, int parent_idx,
                        int sw_num, char sw_dir) {
    if (*tail >= BFS_QUEUE_MAX) return;
    int node_idx = (int)(node - g_track);
    if (node_idx < 0 || node_idx >= TRACK_MAX) return;
    if (bfs_visited[node_idx]) return;
    bfs_visited[node_idx] = 1;
    bfs_q[*tail].node    = node;
    bfs_q[*tail].parent  = (int16_t)parent_idx;
    bfs_q[*tail].sw_num  = (int16_t)sw_num;
    bfs_q[*tail].sw_dir  = sw_dir;
    (*tail)++;
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
        int16_t factor = e->time_factor_q8 ? e->time_factor_q8 : 256;
        int64_t base_us = (int64_t)e->dist * 1000000LL / v;
        total_us += (uint64_t)(base_us * factor / 256);

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


void update_edge_factors(track_node *from, track_node *to, int32_t ratio_q8) {
    if (!from || !to || from == to) return;
    track_node *n = from;
    for (int h = 0; h < 80; h++) {
        track_edge *e = get_next_edge(n);
        if (!e || !e->dest) break;

        int16_t old_f = e->time_factor_q8 ? e->time_factor_q8 : 256;
        int32_t new_f = (7 * (int32_t)old_f + ratio_q8) / 8;
        if (new_f < 128) new_f = 128;
        if (new_f > 512) new_f = 512;
        e->time_factor_q8 = (int16_t)new_f;

        n = e->dest;
        if (n == to) break;
        if (n->type == NODE_EXIT) break;
    }
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

            int s = branch->edge[DIR_STRAIGHT].dest &&
                    reaches_target(branch->edge[DIR_STRAIGHT].dest, to, 40);
            int c = branch->edge[DIR_CURVED].dest &&
                    reaches_target(branch->edge[DIR_CURVED].dest, to, 40);

            if (s == c) break;

            char actual_dir = s ? 'S' : 'C';
            track_node *next = s ? branch->edge[DIR_STRAIGHT].dest
                                 : branch->edge[DIR_CURVED].dest;

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

/* ===== BFS route planning ===== */

int bfs_find_route(track_node *start, track_node *target, route_plan_t *plan) {
    if (!start || !target || !plan) return 0;
    if (start == target) {
        plan->sw_count = 0;
        plan->loop_exit_branch = NULL;
        return 1;
    }

    for (int i = 0; i < TRACK_MAX; i++) bfs_visited[i] = 0;

    int head = 0, tail = 0;
    bfs_enqueue(&tail, start, -1, -1, '?');

    while (head < tail) {
        bfs_entry_t *cur = &bfs_q[head];
        track_node  *n   = cur->node;

        if (n == target) {
            plan->sw_count = 0;
            int stack[20];
            int stack_top = 0;
            int idx = head;
            while (idx >= 0 && stack_top < 20) {                              
                if (bfs_q[idx].sw_num >= 0)    
                   stack[stack_top++] = idx;  
                if (bfs_q[idx].parent < 0) break;                                         
                idx = (int)bfs_q[idx].parent;
            }
            
            for (int s = stack_top - 1; s >= 0 && plan->sw_count < 20; s--) {
                int qi = stack[s];
                plan->sw_nums[plan->sw_count] = (int)bfs_q[qi].sw_num;
                plan->sw_dirs[plan->sw_count] = bfs_q[qi].sw_dir;
                plan->sw_count++;
            }
            plan->loop_exit_branch = NULL;
            return 1;
        }

        head++;

        if (n->type == NODE_BRANCH) {
            if (n->edge[DIR_STRAIGHT].dest)
                bfs_enqueue(&tail, n->edge[DIR_STRAIGHT].dest,
                            head-1, n->num, 'S');
            if (n->edge[DIR_CURVED].dest)
                bfs_enqueue(&tail, n->edge[DIR_CURVED].dest,
                            head-1, n->num, 'C');
        } else if (n->type != NODE_EXIT) {
            if (n->edge[DIR_AHEAD].dest)
                bfs_enqueue(&tail, n->edge[DIR_AHEAD].dest, head-1, -1, '?');
        }
    }

    return 0;
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

    route_plan_t rp;
    track_node  *target = NULL;
    track_node  *loop_plan_start = NULL;

    /* Try user_target first, then its reverse node.
     * For each candidate target, iterate through all loop sensors in the
     * direction of travel starting from plan_start. */
    track_node *try_targets[2] = { user_target, user_target->reverse };

    for (int t = 0; t < 2 && !target; t++) {
        track_node *tgt = try_targets[t];
        if (!tgt) continue;

        for (int i = 0; i < LOOP_SENSOR_COUNT_INTERNAL; i++) {
            track_node *cand;
            if (pos->going_forward) {
                int ci = order[(start_pos + i) % LOOP_SENSOR_COUNT_INTERNAL];
                cand = &g_track[ci];
            } else {
                int ci = order[(start_pos + LOOP_SENSOR_COUNT_INTERNAL - i)
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
    }

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
