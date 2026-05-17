#include "train_tracking/route_priv.h"
#include "train_tracking/position_priv.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "kassert.h"
#include <stddef.h>
#include <stdint.h>

/* ===== Path-based distance helpers ===== */

int32_t route_path_dist_from(const uint16_t *path, int cursor, int count) {
    int32_t total = 0;
    for (int i = cursor; i < count - 1; i++) {
        track_node *a = &g_track[path[i]];
        track_node *b = &g_track[path[i + 1]];
        int32_t d = -1;
        if (a->type == NODE_BRANCH) {
            if (a->edge[DIR_STRAIGHT].dest == b) d = (int32_t)a->edge[DIR_STRAIGHT].dist;
            else if (a->edge[DIR_CURVED].dest == b) d = (int32_t)a->edge[DIR_CURVED].dist;
        } else {
            if (a->edge[DIR_AHEAD].dest == b) d = (int32_t)a->edge[DIR_AHEAD].dist;
        }
        if (d < 0) return -1;
        total += d;
    }
    return total;
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

track_node *route_path_first_remaining_sensor(const train_pos_t *pos) {
    if (!pos || pos->route_path_count <= 0) return NULL;

    int start = pos->route_path_cursor;
    if (start < 0) start = 0;
    if (start >= pos->route_path_count) return NULL;

    for (int i = start; i < pos->route_path_count; i++) {
        int idx = (int)pos->route_path[i];
        if (idx < 0 || idx >= TRACK_MAX) continue;
        if (g_track[idx].type == NODE_SENSOR) return &g_track[idx];
    }
    return NULL;
}

int route_branch_planned_dir(const train_pos_t *pos, track_node *branch) {
    if (!branch || branch->type != NODE_BRANCH) return -1;

    track_node *planned_sensor = route_path_first_remaining_sensor(pos);
    if (!planned_sensor && pos) planned_sensor = pos->pred.next_sensor;

    if (planned_sensor) {
        int straight = follow_dist(branch->edge[DIR_STRAIGHT].dest,
                                   planned_sensor,
                                   OFF_ROUTE_PATH_MAX_HOPS) >= 0;
        int curved = follow_dist(branch->edge[DIR_CURVED].dest,
                                 planned_sensor,
                                 OFF_ROUTE_PATH_MAX_HOPS) >= 0;
        if (straight != curved) return straight ? DIR_STRAIGHT : DIR_CURVED;
    }

    int sw_idx = track_switch_to_index(branch->num);
    char state = (sw_idx >= 0) ? track_get_switch_state()[sw_idx].state : '?';
    if (state == 'S') return DIR_STRAIGHT;
    if (state == 'C') return DIR_CURVED;
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
        if (pos) { pos->pred.alt_sensor = NULL; pos->pred.branch_node = NULL; }
        return NULL;
    }

    int is_accel = pos && pos->is_accelerating && pos->accel_a_eff > 0;

    track_node *n = cur;
    uint64_t total_us = 0;
    int32_t  d_total  = 0;  /* accumulated distance to next sensor */
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
                pos->pred.alt_sensor  = first_sensor_forward(n->edge[alt_dir].dest, 20);
                pos->pred.branch_node = n;
            }
        }

        track_edge *e = get_next_edge(n);
        if (!e || !e->dest) break;

        d_total += (int32_t)e->dist;

        /* In constant-speed mode accumulate timing per edge as before. */
        if (!is_accel && pos->effective_v > 0) {
            total_us += (uint64_t)((int64_t)e->dist * 1000000LL / pos->effective_v);
        }

        n = e->dest;
        if (n->type == NODE_SENSOR) {
            if (!found_branch && pos) {
                pos->pred.alt_sensor = NULL;
                pos->pred.branch_node = NULL;
            }
            if (is_accel) {
                /* Kinematic timing: train accelerates from v0 to v_goto then cruises.
                 *   d1 = (v_goto^2 - v0^2) / (2*a)  — distance remaining in accel phase
                 * Case A (d >= d1): sensor is beyond the accel zone.
                 *   t = (v_goto - v0)/a  +  (d - d1)/v_goto
                 * Case B (d < d1): sensor fires while still accelerating.
                 *   Conservative estimate: use v0 as constant speed (overestimates t,
                 *   safe for dead-track deadline). */
                int32_t v0    = pos->effective_v;
                int32_t v_end = speed_table_get_v(pos->train_ind, pos->goto_speed);
                int32_t a     = pos->accel_a_eff;
                /* d1 = (v_end+v0)*(v_end-v0) / (2*a), computed to avoid overflow */
                int32_t d1 = (v_end > v0)
                             ? (int32_t)((int64_t)(v_end + v0) * (v_end - v0) / (2LL * a))
                             : 0;
                if (d_total >= d1) {
                    int64_t t1_us = (int64_t)(v_end - v0) * 1000000LL / (int64_t)a;
                    int64_t t2_us = (d_total > d1)
                                    ? (int64_t)(d_total - d1) * 1000000LL / (int64_t)v_end
                                    : 0LL;
                    total_us = (uint64_t)(t1_us + t2_us);
                } else {
                    /* Sensor is within accel zone. */
                    if (v0 > 0) {
                        total_us = (uint64_t)((int64_t)d_total * 1000000LL / v0);
                    } else {
                        /* v0==0: train is still in GO_LATENCY_US window.
                         * Use half of v_end as rough average. */
                        total_us = (uint64_t)((int64_t)d_total * 2000000LL / v_end);
                    }
                }
            }
            if (out_dt_us) *out_dt_us = total_us;
            return n;
        }
        if (n->type == NODE_EXIT || ++hops > 80) break;
    }

    if (out_dt_us) *out_dt_us = 0;
    if (!found_branch && pos) { pos->pred.alt_sensor = NULL; pos->pred.branch_node = NULL; }
    return NULL;
}
