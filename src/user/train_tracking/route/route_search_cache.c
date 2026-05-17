#include "route_search_internal.h"

static int g_branch_idx[TRACK_MAX];
static int g_branch_count = 0;
static uint16_t g_canonical_sensor_idx[TRACK_MAX];
static int g_canonical_sensor_count = 0;
static uint16_t g_sorted_direct_sensor_idx[TRACK_MAX][TRACK_MAX];
static int g_sorted_direct_sensor_count[TRACK_MAX];
static int32_t g_free_track_dist[TRACK_MAX][TRACK_MAX];

route_segment_t g_route_segments[TRACK_MAX][2];

static int32_t g_precompute_dist[TRACK_MAX];
static int8_t  g_precompute_done[TRACK_MAX];
static int16_t g_precompute_prev[TRACK_MAX];
static int16_t g_precompute_sw_num[TRACK_MAX];
static char    g_precompute_sw_dir[TRACK_MAX];

static int route_is_canonical_sensor(const track_node *node) {
    if (!node || node->type != NODE_SENSOR) return 0;
    if (!node->reverse || node->reverse->type != NODE_SENSOR) return 1;
    return node < node->reverse;
}

static void route_bitset_clear(uint32_t bits[ROUTE_BLOCKED_WORDS]) {
    for (int i = 0; i < ROUTE_BLOCKED_WORDS; i++) bits[i] = 0;
}

static void route_bitset_set(uint32_t bits[ROUTE_BLOCKED_WORDS], int idx) {
    if (idx < 0 || idx >= TRACK_MAX) return;
    bits[idx / 32] |= (uint32_t)1u << (idx % 32);
}

void route_pack_blocked_bits(const uint8_t *blocked,
                             uint32_t out_bits[ROUTE_BLOCKED_WORDS]) {
    route_bitset_clear(out_bits);
    if (!blocked) return;

    for (int i = 0; i < TRACK_MAX; i++) {
        if (blocked[i]) route_bitset_set(out_bits, i);
    }
}

int32_t route_free_track_dist_idx(int from_idx, int to_idx) {
    if (from_idx < 0 || from_idx >= TRACK_MAX ||
        to_idx < 0 || to_idx >= TRACK_MAX) {
        return DIJK_INF;
    }
    return g_free_track_dist[from_idx][to_idx];
}

static void dijk_run(int32_t *dist, int8_t *done, int16_t *prev,
                     int16_t *sw_num, char *sw_dir,
                     const uint8_t *blocked, const char *fixed_sw_dirs,
                     int start_idx) {
    for (;;) {
        int u = -1;
        int32_t min_d = DIJK_INF;

        if (start_idx >= 0 && start_idx < TRACK_MAX &&
            g_track[start_idx].type != NODE_BRANCH &&
            !done[start_idx] && dist[start_idx] < min_d) {
            u = start_idx;
            min_d = dist[start_idx];
        }
        for (int bi = 0; bi < g_branch_count; bi++) {
            int idx = g_branch_idx[bi];
            if (!done[idx] && dist[idx] < min_d) {
                u = idx;
                min_d = dist[idx];
            }
        }
        if (u < 0) break;
        done[u] = 1;

        if (blocked && blocked[u] && u != start_idx) continue;

        track_node *node = &g_track[u];
        if (node->type == NODE_EXIT || node->type == NODE_NONE) continue;

        int dirs[2] = { DIR_AHEAD, DIR_AHEAD };
        int num_dirs = 1;
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
            track_edge *edge = &node->edge[dir];
            int16_t inh_sw_num = (node->type == NODE_BRANCH) ? (int16_t)node->num : -1;
            char inh_sw_dir = (node->type == NODE_BRANCH)
                                  ? ((dir == DIR_STRAIGHT) ? 'S' : 'C')
                                  : '?';
            int32_t acc = min_d;
            int from = u;

            if (!edge->dest) continue;

            while (edge && edge->dest) {
                int v = (int)(edge->dest - g_track);
                if (v < 0 || v >= TRACK_MAX) break;
                if (done[v]) break;
                if (blocked && blocked[v] && v != start_idx) break;

                acc += (int32_t)edge->dist;
                if (acc < dist[v]) {
                    dist[v] = acc;
                    prev[v] = (int16_t)from;
                    sw_num[v] = inh_sw_num;
                    sw_dir[v] = inh_sw_dir;
                }

                track_node *next = &g_track[v];
                if (next->type == NODE_BRANCH || next->type == NODE_EXIT) break;

                inh_sw_num = -1;
                inh_sw_dir = '?';
                from = v;
                edge = &next->edge[DIR_AHEAD];
            }
        }
    }
}

static void dijk_run_from(int32_t *dist, int8_t *done, int16_t *prev,
                          int16_t *sw_num, char *sw_dir,
                          const uint8_t *blocked, const char *fixed_sw_dirs,
                          track_node *start) {
    for (int i = 0; i < TRACK_MAX; i++) {
        dist[i] = DIJK_INF;
        done[i] = 0;
        prev[i] = -1;
        sw_num[i] = -1;
        sw_dir[i] = '?';
    }

    if (!start) return;

    dist[(int)(start - g_track)] = 0;
    dijk_run(dist, done, prev, sw_num, sw_dir, blocked, fixed_sw_dirs,
             (int)(start - g_track));
}

static void route_precompute_segment_cache(void) {
    for (int start_idx = 0; start_idx < TRACK_MAX; start_idx++) {
        for (int dir = 0; dir < 2; dir++) {
            route_segment_t *seg = &g_route_segments[start_idx][dir];
            track_node *start = &g_track[start_idx];
            track_edge *edge;
            int32_t acc = 0;

            *seg = (route_segment_t){0};
            seg->first_sw_num = (start->type == NODE_BRANCH) ? (int16_t)start->num : -1;
            seg->first_sw_dir = (start->type == NODE_BRANCH)
                                    ? ((dir == DIR_STRAIGHT) ? 'S' : 'C')
                                    : '?';
            route_bitset_clear(seg->node_bits);

            if (start->type == NODE_NONE || start->type == NODE_EXIT) continue;
            if (start->type != NODE_BRANCH && dir != DIR_AHEAD) continue;

            edge = &start->edge[dir];
            if (!edge->dest) continue;

            seg->valid = 1;
            while (edge && edge->dest && seg->node_count < TRACK_MAX) {
                int v = (int)(edge->dest - g_track);
                if (v < 0 || v >= TRACK_MAX) break;

                acc += (int32_t)edge->dist;
                seg->nodes[seg->node_count] = (uint16_t)v;
                seg->prefix_dist[seg->node_count] = acc;
                route_bitset_set(seg->node_bits, v);
                seg->node_count++;

                if (g_track[v].type == NODE_BRANCH || g_track[v].type == NODE_EXIT) {
                    break;
                }

                edge = &g_track[v].edge[DIR_AHEAD];
            }
        }
    }
}

static void route_precompute_free_track_cache(void) {
    g_canonical_sensor_count = 0;

    for (int i = 0; i < TRACK_MAX; i++) {
        if (route_is_canonical_sensor(&g_track[i])) {
            g_canonical_sensor_idx[g_canonical_sensor_count++] = (uint16_t)i;
        }
    }

    for (int start_idx = 0; start_idx < TRACK_MAX; start_idx++) {
        int count = 0;

        dijk_run_from(g_precompute_dist, g_precompute_done,
                      g_precompute_prev, g_precompute_sw_num, g_precompute_sw_dir,
                      NULL, NULL, &g_track[start_idx]);
        for (int i = 0; i < TRACK_MAX; i++) {
            g_free_track_dist[start_idx][i] = g_precompute_dist[i];
        }

        for (int ci = 0; ci < g_canonical_sensor_count; ci++) {
            uint16_t sensor_idx = g_canonical_sensor_idx[ci];
            int32_t dist = g_free_track_dist[start_idx][sensor_idx];
            int insert_at = count;

            if (dist == DIJK_INF) continue;

            while (insert_at > 0) {
                uint16_t prev_idx =
                    g_sorted_direct_sensor_idx[start_idx][insert_at - 1];
                int32_t prev_dist = g_free_track_dist[start_idx][prev_idx];

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

void route_init(void) {
    g_branch_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type == NODE_BRANCH) {
            g_branch_idx[g_branch_count++] = i;
        }
    }

    route_precompute_segment_cache();
    route_precompute_free_track_cache();
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

    dist = g_free_track_dist[start_idx][sensor_idx];
    return (dist == DIJK_INF) ? -1 : dist;
}
