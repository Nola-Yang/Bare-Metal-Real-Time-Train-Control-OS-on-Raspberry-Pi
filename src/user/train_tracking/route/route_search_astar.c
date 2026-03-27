#include "route_search_internal.h"
#include "min_heap.h"
#include "kassert.h"

typedef struct {
    uint16_t node_idx;
} route_open_item_t;

static uint32_t g_route_heap_keys[ROUTE_HEAP_CAPACITY];
static route_open_item_t g_route_heap_items[ROUTE_HEAP_CAPACITY];

static int route_bitset_intersects(const uint32_t lhs[ROUTE_BLOCKED_WORDS],
                                   const uint32_t rhs[ROUTE_BLOCKED_WORDS]) {
    for (int i = 0; i < ROUTE_BLOCKED_WORDS; i++) {
        if (lhs[i] & rhs[i]) return 1;
    }
    return 0;
}

static void route_astar_init(int32_t *dist, int8_t *done, int16_t *prev,
                             int16_t *sw_num, char *sw_dir, int start_idx) {
    for (int i = 0; i < TRACK_MAX; i++) {
        dist[i] = DIJK_INF;
        done[i] = 0;
        prev[i] = -1;
        sw_num[i] = -1;
        sw_dir[i] = '?';
    }

    if (start_idx >= 0 && start_idx < TRACK_MAX) {
        dist[start_idx] = 0;
    }
}

static uint32_t route_open_key(int32_t g_cost, int32_t h_cost, int node_idx) {
    uint32_t f_cost = (uint32_t)(g_cost + h_cost);
    return (f_cost << ROUTE_HEAP_TIE_BITS) | (uint32_t)(node_idx & 0xFF);
}

static void route_push_open(MinHeap_t *heap, int node_idx, int goal_idx,
                            int32_t *dist, int32_t upper_bound_mm) {
    int32_t h_cost = route_free_track_dist_idx(node_idx, goal_idx);
    route_open_item_t item;

    if (node_idx < 0 || node_idx >= TRACK_MAX) return;
    if (dist[node_idx] == DIJK_INF || h_cost == DIJK_INF) return;
    if (upper_bound_mm != DIJK_INF &&
        dist[node_idx] + h_cost > upper_bound_mm) {
        return;
    }

    item.node_idx = (uint16_t)node_idx;
    KASSERT(min_heap_insert(heap, route_open_key(dist[node_idx], h_cost, node_idx),
                            &item));
}

static void route_expand_segment(MinHeap_t *heap, int u, const route_segment_t *seg,
                                 int start_idx, int goal_idx,
                                 const uint8_t *blocked,
                                 const uint32_t *blocked_bits,
                                 int32_t upper_bound_mm,
                                 int32_t *dist, int8_t *done,
                                 int16_t *prev, int16_t *sw_num,
                                 char *sw_dir) {
    int from = u;
    int16_t inh_sw_num = seg->first_sw_num;
    char inh_sw_dir = seg->first_sw_dir;
    int path_has_blocker =
        blocked && blocked_bits && route_bitset_intersects(seg->node_bits, blocked_bits);

    if (!seg->valid || dist[u] == DIJK_INF) return;

    for (int i = 0; i < seg->node_count; i++) {
        int v = (int)seg->nodes[i];
        int32_t new_dist;

        if (path_has_blocker && blocked[v] && v != start_idx) break;

        new_dist = dist[u] + seg->prefix_dist[i];
        if (new_dist < dist[v]) {
            dist[v] = new_dist;
            prev[v] = (int16_t)from;
            sw_num[v] = inh_sw_num;
            sw_dir[v] = inh_sw_dir;

            if (!done[v] &&
                (v == goal_idx || g_track[v].type == NODE_BRANCH)) {
                route_push_open(heap, v, goal_idx, dist, upper_bound_mm);
            }
        }

        if (g_track[v].type == NODE_BRANCH || g_track[v].type == NODE_EXIT) break;

        from = v;
        inh_sw_num = -1;
        inh_sw_dir = '?';
    }
}

int route_search_leg_astar(int32_t *dist, int8_t *done, int16_t *prev,
                           int16_t *sw_num, char *sw_dir,
                           track_node *start, track_node *goal,
                           const uint8_t *blocked,
                           const uint32_t *blocked_bits,
                           const char *fixed_sw_dirs,
                           int32_t upper_bound_mm) {
    MinHeap_t open;
    route_open_item_t item;
    uint32_t pop_key = 0;
    int start_idx;
    int goal_idx;

    if (!start || !goal) return 0;

    start_idx = (int)(start - g_track);
    goal_idx = (int)(goal - g_track);
    if (start_idx < 0 || start_idx >= TRACK_MAX ||
        goal_idx < 0 || goal_idx >= TRACK_MAX) {
        return 0;
    }
    if (upper_bound_mm < 0) return 0;
    if (goal_idx != start_idx && blocked && blocked[goal_idx]) return 0;
    if (route_free_track_dist_idx(start_idx, goal_idx) == DIJK_INF) return 0;

    route_astar_init(dist, done, prev, sw_num, sw_dir, start_idx);
    init_min_heap(&open, g_route_heap_keys, g_route_heap_items,
                  sizeof(g_route_heap_items[0]), ROUTE_HEAP_CAPACITY);
    route_push_open(&open, start_idx, goal_idx, dist, upper_bound_mm);

    while (min_heap_pop(&open, &pop_key, &item)) {
        int u = (int)item.node_idx;
        track_node *node;
        int32_t h_cost;
        int dirs[2] = { DIR_AHEAD, DIR_AHEAD };
        int num_dirs = 1;

        if (u < 0 || u >= TRACK_MAX) continue;
        if (done[u]) continue;
        if (dist[u] == DIJK_INF) continue;

        h_cost = route_free_track_dist_idx(u, goal_idx);
        if (h_cost == DIJK_INF) continue;
        if (upper_bound_mm != DIJK_INF &&
            dist[u] + h_cost > upper_bound_mm) {
            break;
        }

        done[u] = 1;
        if (u == goal_idx) return 1;

        node = &g_track[u];
        if (node->type == NODE_NONE || node->type == NODE_EXIT) continue;

        if (node->type == NODE_BRANCH) {
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
            int dir = (node->type == NODE_BRANCH) ? dirs[d] : DIR_AHEAD;
            route_expand_segment(&open, u, &g_route_segments[u][dir],
                                 start_idx, goal_idx, blocked, blocked_bits,
                                 upper_bound_mm, dist, done, prev, sw_num, sw_dir);
        }
    }

    return dist[goal_idx] != DIJK_INF &&
           (upper_bound_mm == DIJK_INF || dist[goal_idx] <= upper_bound_mm);
}
