#include "traffic_attr_internal.h"

int sensor_hops_between(track_node *from_sensor, track_node *hit_sensor, int max_hops) {
    if (!from_sensor || !hit_sensor) return -1;
    if (from_sensor == hit_sensor) return 0;

    track_node *cur = from_sensor;
    int sensor_hops = 0;
    for (int h = 0; h < max_hops; h++) {
        track_edge *e = traffic_tm_get_next_edge(cur);
        if (!e || !e->dest) return -1;
        cur = e->dest;
        if (cur->type == NODE_SENSOR) sensor_hops++;
        if (cur == hit_sensor) return sensor_hops;
        if (cur->type == NODE_EXIT) return -1;
    }
    return -1;
}

static int path_edge_dist(track_node *a, track_node *b) {
    if (!a || !b) return -1;
    if (a->type == NODE_BRANCH) {
        if (a->edge[DIR_STRAIGHT].dest == b) return (int)a->edge[DIR_STRAIGHT].dist;
        if (a->edge[DIR_CURVED].dest == b) return (int)a->edge[DIR_CURVED].dist;
        return -1;
    }
    if (a->edge[DIR_AHEAD].dest == b) return (int)a->edge[DIR_AHEAD].dist;
    return -1;
}

/* Follow forward from `start` and return how many sensors on that leg are
 * skipped before reaching `hit` (0 = first reachable sensor on that leg). */
static int sensor_skip_forward_from(track_node *start, track_node *hit,
                                    int max_hops, int32_t *out_dist_mm) {
    if (out_dist_mm) *out_dist_mm = -1;
    if (!start || !hit) return -1;

    track_node *cur = start;
    int32_t dist_mm = 0;
    int sensor_hops = 0;

    for (int h = 0; h < max_hops; h++) {
        if (cur->type == NODE_SENSOR) {
            sensor_hops++;
            if (cur == hit) {
                if (out_dist_mm) *out_dist_mm = dist_mm;
                return sensor_hops - 1;
            }
        }
        if (cur->type == NODE_EXIT) return -1;

        track_edge *e = traffic_tm_get_next_edge(cur);
        if (!e || !e->dest) return -1;
        dist_mm += (int32_t)e->dist;
        cur = e->dest;
    }

    if (cur->type == NODE_SENSOR && cur == hit) {
        sensor_hops++;
        if (out_dist_mm) *out_dist_mm = dist_mm;
        return sensor_hops - 1;
    }

    return -1;
}

/* Scan the current prediction leg for a turnout mismatch and allow the hit to
 * match any of the first few sensors on that alternate leg, not only the first
 * one. This covers cases where the first sensor on the wrong branch is dead. */
int current_leg_alt_branch_skip_to_hit(const train_pos_t *pos,
                                       track_node *hit,
                                       int32_t *out_dist_mm) {
    if (out_dist_mm) *out_dist_mm = -1;
    if (!pos || !hit || !pos->cur_sensor || !pos->pred.next_sensor) return -1;

    track_node *cur = pos->cur_sensor;
    int32_t dist_to_branch_mm = 0;
    for (int h = 0; h < 80; h++) {
        track_edge *e = traffic_tm_get_next_edge(cur);
        if (!e || !e->dest) return -1;
        dist_to_branch_mm += (int32_t)e->dist;
        cur = e->dest;
        if (cur == pos->pred.next_sensor || cur->type == NODE_SENSOR) return -1;
        if (cur->type != NODE_BRANCH) continue;

        int planned_dir = route_branch_planned_dir(pos, cur);
        if (planned_dir != DIR_STRAIGHT && planned_dir != DIR_CURVED) return -1;

        int alt_dir = (planned_dir == DIR_STRAIGHT) ? DIR_CURVED : DIR_STRAIGHT;
        int32_t alt_tail_mm = -1;
        int skip = sensor_skip_forward_from(cur->edge[alt_dir].dest, hit,
                                            OFF_ROUTE_PATH_MAX_HOPS, &alt_tail_mm);
        if (skip < 0) continue;

        if (out_dist_mm) {
            *out_dist_mm = dist_to_branch_mm + (int32_t)cur->edge[alt_dir].dist;
            if (alt_tail_mm > 0) *out_dist_mm += alt_tail_mm;
        }
        return skip;
    }
    return -1;
}

/* Scan the remaining planned path for a later branch whose unplanned leg leads
 * to the hit sensor. Like current_leg_alt_branch_skip_to_hit(), this accepts a
 * later sensor on that wrong leg when the first one never fired. */
int route_path_alt_branch_skip_to_hit(const train_pos_t *pos,
                                      track_node *hit,
                                      int32_t *out_dist_mm) {
    if (out_dist_mm) *out_dist_mm = -1;
    if (!pos || !hit || pos->route_path_count <= 1) return -1;

    int start = pos->route_path_cursor;
    if (start < 0) start = 0;
    if (start >= pos->route_path_count - 1) return -1;

    int32_t dist_mm = 0;
    for (int i = start; i < pos->route_path_count - 1; i++) {
        int idx = (int)pos->route_path[i];
        int next_idx = (int)pos->route_path[i + 1];
        if (idx < 0 || idx >= TRACK_MAX || next_idx < 0 || next_idx >= TRACK_MAX) return -1;

        if (i > start) {
            int prev_idx = (int)pos->route_path[i - 1];
            if (prev_idx < 0 || prev_idx >= TRACK_MAX) return -1;
            int edge_mm = path_edge_dist(&g_track[prev_idx], &g_track[idx]);
            if (edge_mm < 0) return -1;
            dist_mm += edge_mm;
        }

        track_node *node = &g_track[idx];
        if (node->type != NODE_BRANCH) continue;

        track_node *planned_next = &g_track[next_idx];
        int planned_dir = -1;
        if (node->edge[DIR_STRAIGHT].dest == planned_next) {
            planned_dir = DIR_STRAIGHT;
        } else if (node->edge[DIR_CURVED].dest == planned_next) {
            planned_dir = DIR_CURVED;
        } else {
            continue;
        }

        int alt_dir = (planned_dir == DIR_STRAIGHT) ? DIR_CURVED : DIR_STRAIGHT;
        int32_t alt_tail_mm = -1;
        int skip = sensor_skip_forward_from(node->edge[alt_dir].dest, hit,
                                            OFF_ROUTE_PATH_MAX_HOPS, &alt_tail_mm);
        if (skip < 0) continue;

        if (out_dist_mm) {
            *out_dist_mm = dist_mm + (int32_t)node->edge[alt_dir].dist;
            if (alt_tail_mm > 0) *out_dist_mm += alt_tail_mm;
        }
        return skip;
    }

    return -1;
}

int route_path_skip_to_hit(const train_pos_t *pos, track_node *hit,
                           int32_t *out_dist_mm) {
    if (out_dist_mm) *out_dist_mm = -1;
    if (!pos || !hit || pos->route_path_count <= 0) return -1;

    int start = pos->route_path_cursor;
    if (start < 0) start = 0;
    if (start >= pos->route_path_count) return -1;

    while (start < pos->route_path_count) {
        int idx = (int)pos->route_path[start];
        if (idx >= 0 && idx < TRACK_MAX && g_track[idx].type == NODE_SENSOR) break;
        start++;
    }
    if (start >= pos->route_path_count) return -1;

    int32_t dist_mm = 0;
    int sensor_hops = 0;
    for (int i = start; i < pos->route_path_count; i++) {
        int idx = (int)pos->route_path[i];
        if (idx < 0 || idx >= TRACK_MAX) return -1;

        if (i > start) {
            int prev_idx = (int)pos->route_path[i - 1];
            if (prev_idx < 0 || prev_idx >= TRACK_MAX) return -1;
            int edge_mm = path_edge_dist(&g_track[prev_idx], &g_track[idx]);
            if (edge_mm < 0) return -1;
            dist_mm += edge_mm;
        }

        track_node *node = &g_track[idx];
        if (node->type == NODE_SENSOR && i > start) sensor_hops++;
        if (node != hit) continue;
        if (node->type != NODE_SENSOR) return -1;
        if (out_dist_mm) *out_dist_mm = dist_mm;
        return (sensor_hops > 0) ? (sensor_hops - 1) : 0;
    }

    return -1;
}
