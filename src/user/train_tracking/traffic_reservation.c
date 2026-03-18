#include "train_tracking/traffic_manager.h"
#include "train_tracking/route_priv.h"
#include "traffic_manager_internal.h"
#include "ui.h"
#include <stdint.h>

#define MAX_MUTEX_ZONES 32
#define MAX_ZONE_NODES  16

static int node_owner[TRACK_MAX];
static uint32_t g_change_generation = 0;

/* Crossing + merge safety envelopes are modeled as mutex zones. */
static int g_zone_nodes[MAX_MUTEX_ZONES][MAX_ZONE_NODES];
static int g_zone_counts[MAX_MUTEX_ZONES];
static int g_zone_count = 0;
static uint32_t g_node_zone_mask[TRACK_MAX];

static void traffic_note_change(int routes_dirty) {
    g_change_generation++;
    if (routes_dirty) pos_mark_routes_dirty();
    ui_mark_position_dirty();
}

static void zone_add_idx(int zone, int idx) {
    if (zone < 0 || zone >= g_zone_count) return;
    if (idx < 0 || idx >= TRACK_MAX) return;

    for (int i = 0; i < g_zone_counts[zone]; i++) {
        if (g_zone_nodes[zone][i] == idx) return;
    }
    if (g_zone_counts[zone] >= MAX_ZONE_NODES) return;

    g_zone_nodes[zone][g_zone_counts[zone]++] = idx;
}

static void zone_add_node_pair(int zone, int idx) {
    zone_add_idx(zone, idx);
    zone_add_idx(zone, traffic_reverse_index(idx));
}

static int zone_begin(void) {
    if (g_zone_count >= MAX_MUTEX_ZONES) return -1;
    g_zone_counts[g_zone_count] = 0;
    return g_zone_count++;
}

static void zone_commit(int zone) {
    if (zone < 0 || zone >= g_zone_count) return;
    uint32_t bit = 1u << zone;
    for (int i = 0; i < g_zone_counts[zone]; i++) {
        int idx = g_zone_nodes[zone][i];
        if (idx >= 0 && idx < TRACK_MAX) g_node_zone_mask[idx] |= bit;
    }
}

static void expand_marks_with_zones(uint8_t marks[TRACK_MAX]) {
    int changed;
    do {
        changed = 0;
        uint32_t active_zones = 0;
        for (int i = 0; i < TRACK_MAX; i++) {
            if (marks[i]) active_zones |= g_node_zone_mask[i];
        }
        if (active_zones == 0) break;

        for (int zone = 0; zone < g_zone_count; zone++) {
            uint32_t bit = 1u << zone;
            if ((active_zones & bit) == 0) continue;
            int touched = 0;
            for (int i = 0; i < g_zone_counts[zone]; i++) {
                if (marks[g_zone_nodes[zone][i]]) {
                    touched = 1;
                    break;
                }
            }
            if (!touched) continue;
            for (int i = 0; i < g_zone_counts[zone]; i++) {
                int idx = g_zone_nodes[zone][i];
                if (!marks[idx]) {
                    marks[idx] = 1;
                    changed = 1;
                }
            }
        }
    } while (changed);
}

static void keep_mark_node(uint8_t keep[TRACK_MAX], track_node *n) {
    if (!n) return;
    int idx = traffic_node_index(n);
    int ridx = traffic_reverse_index(idx);
    if (idx >= 0) keep[idx] = 1;
    if (ridx >= 0) keep[ridx] = 1;
}

static void keep_mark_walk_dist(uint8_t keep[TRACK_MAX], track_node *start, int32_t dist_mm) {
    if (!start) return;
    track_node *cur = start;
    int32_t dist = 0;
    for (int h = 0; h < 200; h++) {
        keep_mark_node(keep, cur);
        if (dist >= dist_mm) break;
        track_edge *e = traffic_tm_get_next_edge(cur);
        if (!e || !e->dest) break;
        dist += e->dist;
        cur = e->dest;
    }
}

static void keep_mark_walk_to(uint8_t keep[TRACK_MAX], track_node *start, track_node *end) {
    if (!start || !end) return;
    track_node *cur = start;
    for (int h = 0; h < 200; h++) {
        keep_mark_node(keep, cur);
        if (cur == end) break;
        track_edge *e = traffic_tm_get_next_edge(cur);
        if (!e || !e->dest) break;
        cur = e->dest;
    }
}

static void build_plan_marks(uint8_t want[TRACK_MAX], const route_plan_t *plan) {
    for (int i = 0; i < TRACK_MAX; i++) want[i] = 0;
    if (!plan) return;

    for (int leg = 0; leg < 2; leg++) {
        const uint16_t *path = (leg == 0) ? plan->path_nodes : plan->path_nodes2;
        int path_count = (leg == 0) ? plan->path_count : plan->path_count2;
        if (path_count <= 0) continue;

        for (int i = 0; i < path_count; i++) {
            int idx = (int)path[i];
            int ridx = traffic_reverse_index(idx);
            if (idx >= 0 && idx < TRACK_MAX) want[idx] = 1;
            if (ridx >= 0 && ridx < TRACK_MAX) want[ridx] = 1;
        }
    }

    expand_marks_with_zones(want);
}

static int plan_has_conflict(int train_num, const uint8_t want[TRACK_MAX]) {
    for (int i = 0; i < TRACK_MAX; i++) {
        if (!want[i]) continue;
        if (node_owner[i] >= 0 && node_owner[i] != train_num) return 1;
    }
    return 0;
}

void traffic_reservation_init(void) {
    for (int i = 0; i < TRACK_MAX; i++) node_owner[i] = -1;
    g_change_generation = 0;

    g_zone_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) g_node_zone_mask[i] = 0;

    /* Diamond crossing: all four paired turnouts share one mutex zone. */
    int crossing_zone = zone_begin();
    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        if (n->type == NODE_BRANCH && 153 <= n->num && n->num <= 156) {
            zone_add_node_pair(crossing_zone, i);
        }
    }
    zone_commit(crossing_zone);

    /* Each merge owns its two approach legs plus the paired branch node. */
    for (int merge_idx = 0; merge_idx < TRACK_MAX; merge_idx++) {
        if (g_track[merge_idx].type != NODE_MERGE) continue;

        int zone = zone_begin();
        if (zone < 0) break;

        zone_add_node_pair(zone, merge_idx);
        for (int src_idx = 0; src_idx < TRACK_MAX; src_idx++) {
            track_node *src = &g_track[src_idx];
            for (int dir = 0; dir < 2; dir++) {
                if (src->edge[dir].dest == &g_track[merge_idx]) {
                    zone_add_node_pair(zone, src_idx);
                }
            }
        }
        zone_commit(zone);
    }
}

void traffic_build_constraints(int requester_train, uint8_t blocked[TRACK_MAX]) {
    if (!blocked) return;
    for (int i = 0; i < TRACK_MAX; i++) {
        int owner = node_owner[i];
        blocked[i] = (owner >= 0 && owner != requester_train) ? 1 : 0;
    }
    expand_marks_with_zones(blocked);
}

int traffic_reserve_plan(int train_num, track_node *start, const route_plan_t *plan) {
    (void)start;
    if (!plan || train_num < 0) return 0;
    uint8_t want[TRACK_MAX];
    int changed = 0;

    build_plan_marks(want, plan);
    if (plan_has_conflict(train_num, want)) return 0;

    for (int i = 0; i < TRACK_MAX; i++) {
        if (!want[i]) continue;
        if (node_owner[i] != train_num) {
            node_owner[i] = train_num;
            changed = 1;
        }
    }

    if (changed) traffic_note_change(0);
    return 1;
}

int traffic_can_reserve_plan(int train_num, const route_plan_t *plan) {
    if (!plan || train_num < 0) return 0;
    uint8_t want[TRACK_MAX];
    build_plan_marks(want, plan);
    return !plan_has_conflict(train_num, want);
}

void traffic_release_train(int train_num) {
    int changed = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (node_owner[i] == train_num) {
            node_owner[i] = -1;
            changed = 1;
        }
    }
    if (changed) traffic_note_change(1);
}

void traffic_release_train_keep_body(int train_num, track_node *last_hit,
                                     int32_t body_mm, track_node *next_hit) {
    (void)body_mm;
    uint8_t keep[TRACK_MAX];
    int keep_to_next = 0;

    for (int i = 0; i < TRACK_MAX; i++) keep[i] = 0;

    if (last_hit && next_hit) {
        keep_to_next = (last_hit == next_hit) ||
                       (follow_dist(last_hit, next_hit, 120) >= 0);
    }

    if (last_hit) {
        keep_mark_node(keep, last_hit);
        if (last_hit->reverse) {
            keep_mark_walk_dist(keep, last_hit->reverse, 0);
        }
        if (keep_to_next) {
            if (last_hit != next_hit) {
                keep_mark_walk_to(keep, last_hit, next_hit);
            }
            keep_mark_walk_dist(keep, next_hit, 0);
        }
    }

    expand_marks_with_zones(keep);

    int changed = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (node_owner[i] != train_num) continue;
        if (!keep[i]) {
            node_owner[i] = -1;
            changed = 1;
        }
    }

    if (last_hit) {
        int idx = traffic_node_index(last_hit);
        int ridx = traffic_reverse_index(idx);
        if (idx >= 0 && node_owner[idx] != train_num) {
            node_owner[idx] = train_num;
            changed = 1;
        }
        if (ridx >= 0 && node_owner[ridx] != train_num) {
            node_owner[ridx] = train_num;
            changed = 1;
        }
    }
    if (changed) traffic_note_change(1);
}

void traffic_refresh_route_reservation(int train_num, track_node *cur_sensor,
                                       track_node *next_hit,
                                       const uint16_t *path, int path_cursor,
                                       int path_count) {
    if (train_num < 0 || !cur_sensor) return;

    uint8_t keep[TRACK_MAX];
    for (int i = 0; i < TRACK_MAX; i++) keep[i] = 0;

    keep_mark_node(keep, cur_sensor);

    if (next_hit) {
        int keep_to_next = (cur_sensor == next_hit) ||
                           (follow_dist(cur_sensor, next_hit, 120) >= 0);
        if (keep_to_next) keep_mark_walk_to(keep, cur_sensor, next_hit);
    }

    if (path && path_count > 0) {
        if (path_cursor < 0) path_cursor = 0;
        if (path_cursor > path_count) path_cursor = path_count;
        for (int i = path_cursor; i < path_count; i++) {
            int idx = (int)path[i];
            if (idx < 0 || idx >= TRACK_MAX) continue;
            keep[idx] = 1;
            int ridx = traffic_reverse_index(idx);
            if (ridx >= 0) keep[ridx] = 1;
        }
    }

    expand_marks_with_zones(keep);

    int changed = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (node_owner[i] != train_num) continue;
        if (!keep[i]) {
            node_owner[i] = -1;
            changed = 1;
        }
    }

    int idx = traffic_node_index(cur_sensor);
    int ridx = traffic_reverse_index(idx);
    if (idx >= 0 && node_owner[idx] != train_num) {
        node_owner[idx] = train_num;
        changed = 1;
    }
    if (ridx >= 0 && node_owner[ridx] != train_num) {
        node_owner[ridx] = train_num;
        changed = 1;
    }

    if (changed) traffic_note_change(1);
}

int traffic_can_set_switch(int sw_num, int requester_train) {
    (void)requester_train;
    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        if (n->type != NODE_BRANCH || n->num != sw_num) continue;

        int checks[6];
        int c = 0;
        checks[c++] = i;
        checks[c++] = traffic_reverse_index(i);
        if (n->edge[DIR_STRAIGHT].dest) {
            int idx = traffic_node_index(n->edge[DIR_STRAIGHT].dest);
            checks[c++] = idx;
            checks[c++] = traffic_reverse_index(idx);
        }
        if (n->edge[DIR_CURVED].dest) {
            int idx = traffic_node_index(n->edge[DIR_CURVED].dest);
            checks[c++] = idx;
            checks[c++] = traffic_reverse_index(idx);
        }

        for (int j = 0; j < c; j++) {
            int idx = checks[j];
            if (idx < 0 || idx >= TRACK_MAX) continue;
            int owner = node_owner[idx];
            if (owner >= 0) return owner;
        }
    }
    return -1;
}

int traffic_get_reserved_nodes(int train_num, uint16_t *out, int max_nodes) {
    int total = 0;
    if (max_nodes < 0) max_nodes = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (node_owner[i] != train_num) continue;
        if (out && total < max_nodes) {
            out[total] = (uint16_t)i;
        }
        total++;
    }
    return total;
}

int traffic_is_reserved_by(track_node *node, int train_num) {
    int idx = traffic_node_index(node);
    if (idx < 0) return 0;
    return node_owner[idx] == train_num;
}

uint32_t traffic_get_change_generation(void) {
    return g_change_generation;
}
