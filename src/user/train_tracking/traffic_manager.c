#include "train_tracking/traffic_manager.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "track.h"
#include "kassert.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

#define ATTR_TIME_GATE_US 2500000ULL
#define ATTR_MAX_SKIP     2
#define ATTR_MARGIN       120

static int node_owner[TRACK_MAX];

/* Crossing zone mutex: SW153-156 diamond crossing */
static int g_crossing_idx[8];
static int g_crossing_count = 0;

static int spurious_sensor_count = 0;
static int ambiguous_sensor_count = 0;
static uint16_t last_spurious_sensor_id = 0;
static uint16_t last_ambiguous_sensor_id = 0;
static uint64_t last_spurious_time_us = 0;
static uint64_t last_ambiguous_time_us = 0;

static int node_index(track_node *n) {
    if (!n) return -1;
    int idx = (int)(n - g_track);
    return (idx >= 0 && idx < TRACK_MAX) ? idx : -1;
}

static int reverse_index(int idx) {
    if (idx < 0 || idx >= TRACK_MAX) return -1;
    track_node *r = g_track[idx].reverse;
    return r ? node_index(r) : -1;
}

static int abs64(int64_t x) {
    return (x < 0) ? (int)(-x) : (int)x;
}

static int should_consider_for_attr(const train_pos_t *pos) {
    if (!pos || pos->train_num < 0) return 0;
    if (pos->route_state == TRAIN_STATE_STOPPED ||
        pos->route_state == TRAIN_STATE_DEAD_TRACK ||
        pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
        return 0;
    }
    return 1;
}

static int is_bootstrap_state(train_route_state_t st) {
    return (st == TRAIN_STATE_UNKNOWN ||
            st == TRAIN_STATE_FIND_POS);
}

static track_edge *tm_get_next_edge(track_node *n) {
    if (!n) return NULL;
    switch (n->type) {
    case NODE_SENSOR:
    case NODE_MERGE:
    case NODE_ENTER:
        return &n->edge[DIR_AHEAD];
    case NODE_BRANCH: {
        int sw_idx = track_switch_to_index(n->num);
        if (sw_idx < 0) return NULL;
        char st = track_get_switch_state()[sw_idx].state;
        if (st != 'S' && st != 'C') return NULL;
        return &n->edge[(st == 'C') ? DIR_CURVED : DIR_STRAIGHT];
    }
    default:
        return NULL;
    }
}

/* Returns #sensor hops from from_sensor to hit_sensor along current switches.
 * 0 means same sensor, 1 means immediate next sensor, etc. */
static int sensor_hops_between(track_node *from_sensor, track_node *hit_sensor, int max_hops) {
    if (!from_sensor || !hit_sensor) return -1;
    if (from_sensor == hit_sensor) return 0;

    track_node *cur = from_sensor;
    int sensor_hops = 0;
    for (int h = 0; h < max_hops; h++) {
        track_edge *e = tm_get_next_edge(cur);
        if (!e || !e->dest) return -1;
        cur = e->dest;
        if (cur->type == NODE_SENSOR) sensor_hops++;
        if (cur == hit_sensor) return sensor_hops;
        if (cur->type == NODE_EXIT) return -1;
    }
    return -1;
}

void traffic_init(void) {
    for (int i = 0; i < TRACK_MAX; i++) node_owner[i] = -1;
    spurious_sensor_count = 0;
    ambiguous_sensor_count = 0;
    last_spurious_sensor_id = 0;
    last_ambiguous_sensor_id = 0;
    last_spurious_time_us = 0;
    last_ambiguous_time_us = 0;

    /* Cache crossing node indices for SW153-156 diamond */
    g_crossing_count = 0;
    for (int i = 0; i < TRACK_MAX && g_crossing_count < 8; i++) {
        track_node *n = &g_track[i];
        if (n->type != NODE_BRANCH) continue;
        if (n->num < 153 || n->num > 156) continue;
        if (g_crossing_count < 8) g_crossing_idx[g_crossing_count++] = i;
        int ridx = reverse_index(i);
        if (ridx >= 0 && g_crossing_count < 8) g_crossing_idx[g_crossing_count++] = ridx;
    }
}

static int crossing_is_blocked_by_other(int train_num) {
    for (int i = 0; i < g_crossing_count; i++) {
        int idx = g_crossing_idx[i];
        if (node_owner[idx] >= 0 && node_owner[idx] != train_num)
            return 1;
    }
    return 0;
}

void traffic_build_constraints(int requester_train, uint8_t blocked[TRACK_MAX]) {
    if (!blocked) return;
    for (int i = 0; i < TRACK_MAX; i++) {
        int owner = node_owner[i];
        blocked[i] = (owner >= 0 && owner != requester_train) ? 1 : 0;
    }

    /* Crossing zone mutex: if any crossing node is blocked, block all of them */
    int crossing_taken = 0;
    for (int i = 0; i < g_crossing_count; i++) {
        if (blocked[g_crossing_idx[i]]) { crossing_taken = 1; break; }
    }
    if (crossing_taken) {
        for (int i = 0; i < g_crossing_count; i++)
            blocked[g_crossing_idx[i]] = 1;
    }
}

int traffic_reserve_plan(int train_num, track_node *start, const route_plan_t *plan) {
    (void)start;
    if (!plan || train_num < 0) return 0;

    /* Crossing zone pre-check: if plan touches crossing and another train owns it, fail */
    int plan_touches_crossing = 0;
    for (int p = 0; p < plan->path_count && !plan_touches_crossing; p++) {
        for (int ci = 0; ci < g_crossing_count; ci++) {
            if (plan->path_nodes[p] == g_crossing_idx[ci]) {
                plan_touches_crossing = 1; break;
            }
        }
    }
    for (int p = 0; p < plan->path_count2 && !plan_touches_crossing; p++) {
        for (int ci = 0; ci < g_crossing_count; ci++) {
            if (plan->path_nodes2[p] == g_crossing_idx[ci]) {
                plan_touches_crossing = 1; break;
            }
        }
    }
    if (plan_touches_crossing && crossing_is_blocked_by_other(train_num))
        return 0;

    int touched[TRACK_MAX * 2];
    int touched_count = 0;

    for (int leg = 0; leg < 2; leg++) {
        const uint16_t *path = (leg == 0) ? plan->path_nodes : plan->path_nodes2;
        int path_count = (leg == 0) ? plan->path_count : plan->path_count2;
        if (path_count <= 0) continue;

        for (int i = 0; i < path_count; i++) {
            int idx = (int)path[i];
            int pair = reverse_index(idx);
            int to_check[2] = { idx, pair };

            for (int k = 0; k < 2; k++) {
                int nidx = to_check[k];
                if (nidx < 0 || nidx >= TRACK_MAX) continue;
                int owner = node_owner[nidx];
                if (owner >= 0 && owner != train_num) {
                    for (int r = 0; r < touched_count; r++) {
                        if (node_owner[touched[r]] == train_num) {
                            node_owner[touched[r]] = -1;
                        }
                    }
                    return 0;
                }
                if (owner < 0) {
                    node_owner[nidx] = train_num;
                    touched[touched_count++] = nidx;
                }
            }
        }
    }

    ui_mark_position_dirty();
    return 1;
}

void traffic_release_train(int train_num) {
    int changed = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (node_owner[i] == train_num) {
            node_owner[i] = -1;
            changed = 1;
        }
    }
    if (changed) ui_mark_position_dirty();
}

/* Keep a node and its reverse pair in the keep-set. */
static void keep_add(int keep[], int *kc, track_node *n) {
    if (!n) return;
    int idx  = node_index(n);
    int ridx = reverse_index(idx);
    if (idx  >= 0 && *kc < TRACK_MAX * 2) keep[(*kc)++] = idx;
    if (ridx >= 0 && *kc < TRACK_MAX * 2) keep[(*kc)++] = ridx;
}

/* Walk from 'start' along forward edges for up to dist_mm, keeping every node. */
static void keep_walk_dist(int keep[], int *kc, track_node *start, int32_t dist_mm) {
    if (!start) return;
    track_node *cur = start;
    int32_t dist = 0;
    for (int h = 0; h < 200; h++) {
        keep_add(keep, kc, cur);
        if (dist >= dist_mm) break;
        track_edge *e = tm_get_next_edge(cur);
        if (!e || !e->dest) break;
        dist += e->dist;
        cur = e->dest;
    }
}

/* Walk from 'start' to 'end' along forward edges, keeping every node. */
static void keep_walk_to(int keep[], int *kc, track_node *start, track_node *end) {
    if (!start || !end) return;
    track_node *cur = start;
    for (int h = 0; h < 200; h++) {
        keep_add(keep, kc, cur);
        if (cur == end) break;
        track_edge *e = tm_get_next_edge(cur);
        if (!e || !e->dest) break;
        cur = e->dest;
    }
}

void traffic_release_train_keep_body(int train_num, track_node *front,
                                     int going_forward, int32_t body_mm,
                                     track_node *end) {
    int keep[TRACK_MAX * 2];
    int keep_count = 0;

    if (end == NULL) {
        keep_add(keep, &keep_count, front);
        keep_walk_dist(keep, &keep_count, front, body_mm);
        if (front && front->reverse)
            keep_walk_dist(keep, &keep_count, front->reverse, body_mm);

    } else if (going_forward) {
        if (front == end) {
            keep_add(keep, &keep_count, front);
            if (end->reverse)
                keep_walk_dist(keep, &keep_count, end->reverse, body_mm); /* 200mm before */
            keep_walk_dist(keep, &keep_count, end, 100);                   /* 100mm after */
        } else {
            keep_walk_to(keep, &keep_count, front, end);
            if (end->reverse)
                keep_walk_dist(keep, &keep_count, end->reverse, 100);
            /* Keep train body behind the head */
            if (front->reverse)
                keep_walk_dist(keep, &keep_count, front->reverse, body_mm);
        }

    } else {
        if (front == end) {
            keep_add(keep, &keep_count, front);
            if (end->reverse)
                keep_walk_dist(keep, &keep_count, end->reverse, 100);  /* 100mm before */
            keep_walk_dist(keep, &keep_count, end, body_mm);            /* 200mm after */
        } else {
            keep_walk_to(keep, &keep_count, front, end);
            keep_walk_dist(keep, &keep_count, end, body_mm);
        }
    }

    int changed = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (node_owner[i] != train_num) continue;
        int keep_it = 0;
        for (int k = 0; k < keep_count; k++) {
            if (keep[k] == i) { keep_it = 1; break; }
        }
        if (!keep_it) { node_owner[i] = -1; changed = 1; }
    }

    if (front) {
        int idx  = node_index(front);
        int ridx = reverse_index(idx);
        if (idx  >= 0 && node_owner[idx]  != train_num) { node_owner[idx]  = train_num; changed = 1; }
        if (ridx >= 0 && node_owner[ridx] != train_num) { node_owner[ridx] = train_num; changed = 1; }
    }
    if (changed) ui_mark_position_dirty();
}

void traffic_release_passed(int train_num, track_node *from, track_node *to) {
    if (train_num < 0 || !from || !to) return;
    if (from == to) return;

    track_node *cur = from;
    int changed = 0;
    for (int h = 0; h < 120; h++) {
        int idx = node_index(cur);
        int ridx = reverse_index(idx);
        if (idx >= 0 && node_owner[idx] == train_num) {
            node_owner[idx] = -1;
            changed = 1;
        }
        if (ridx >= 0 && node_owner[ridx] == train_num) {
            node_owner[ridx] = -1;
            changed = 1;
        }

        track_edge *e = tm_get_next_edge(cur);
        if (!e || !e->dest) {
            if (changed) ui_mark_position_dirty();
            return;
        }
        cur = e->dest;
        if (cur == to || cur->type == NODE_EXIT) {
            if (changed) ui_mark_position_dirty();
            return;
        }
    }
    if (changed) ui_mark_position_dirty();
}

int traffic_can_set_switch(int sw_num, int requester_train) {
    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        if (n->type != NODE_BRANCH || n->num != sw_num) continue;

        int checks[6];
        int c = 0;
        checks[c++] = i;
        checks[c++] = reverse_index(i);
        if (n->edge[DIR_STRAIGHT].dest) checks[c++] = node_index(n->edge[DIR_STRAIGHT].dest);
        if (n->edge[DIR_CURVED].dest) checks[c++] = node_index(n->edge[DIR_CURVED].dest);
        if (n->edge[DIR_STRAIGHT].dest) checks[c++] = reverse_index(node_index(n->edge[DIR_STRAIGHT].dest));
        if (n->edge[DIR_CURVED].dest) checks[c++] = reverse_index(node_index(n->edge[DIR_CURVED].dest));

        for (int j = 0; j < c; j++) {
            int idx = checks[j];
            if (idx < 0 || idx >= TRACK_MAX) continue;
            int owner = node_owner[idx];
            if (owner >= 0 && owner != requester_train) return owner;
        }
    }
    return -1;
}

train_pos_t *traffic_attribute_sensor(track_node *hit, uint64_t time_us) {
    if (!hit || hit->type != NODE_SENSOR) return NULL;

    train_pos_t *best = NULL;
    train_pos_t *second = NULL;
    int32_t best_score = -1;
    int32_t second_score = -1;
    int best_conf = 0;
    int second_conf = 0;
    int best_terr = 0x7fffffff;
    int second_terr = 0x7fffffff;

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (!should_consider_for_attr(pos)) continue;

        int32_t score = 0;
        int32_t conf = 0;
        int has_candidate = 0;

        if (pos->pred.next_sensor == hit) {
            score = 10000;
            conf = 3;
            has_candidate = 1;
        } else if (pos->pred.alt_sensor == hit) {
            score = 8500;
            conf = 2;
            has_candidate = 1;
        } else if (pos->pred.next_sensor) {
            int32_t pred_dist = follow_dist(pos->pred.next_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
            if (pred_dist >= 0) {
                int hops = sensor_hops_between(pos->pred.next_sensor, hit, 120);
                int skip = (hops >= 0) ? (hops - 1) : 99;
                if (skip <= ATTR_MAX_SKIP) {
                    score = 7400 - skip * 600 - pred_dist / 20;
                    conf = 2;
                    has_candidate = 1;
                }
            }
        }

        if (!has_candidate && pos->cur_sensor) {
            int32_t cur_dist = follow_dist(pos->cur_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
            if (cur_dist >= 0) {
                score = 2800 - cur_dist / 25;
                conf = 1;
                has_candidate = 1;
            }
        }

        /* Upstream-of-prediction fallback: fires only when trigger_time==0
         * (just reversed, no timing yet).  
         * Covers: overshoot-then-reverse fires target->reverse before the
         * predicted reversal sensor. */
        if (!has_candidate &&
            pos->route_state == TRAIN_STATE_ON_ROUTE &&
            pos->pred.next_sensor != NULL &&
            pos->pred.trigger_time == 0) {
            int32_t to_pred = follow_dist(hit, pos->pred.next_sensor, 10);
            if (to_pred >= 0 && to_pred <= 800) {
                score = 2600 - to_pred / 20;
                conf = 1;
                has_candidate = 1;
            }
        }

        if (!has_candidate || score <= 0) continue;

        int terr = 0x7fffffff;
        if (pos->pred.trigger_time > 0) {
            terr = abs64((int64_t)time_us - (int64_t)pos->pred.trigger_time);
            if ((uint64_t)terr > ATTR_TIME_GATE_US && pos->pred.next_sensor != hit) {
                continue;
            }
            if ((uint64_t)terr <= ATTR_TIME_GATE_US) {
                score += (int32_t)((ATTR_TIME_GATE_US - (uint64_t)terr) / 20000ULL);
            } else {
                score -= 200;
            }
        }

        if (score > best_score) {
            second_score = best_score;
            second = best;
            second_conf = best_conf;
            second_terr = best_terr;
            best_score = score;
            best = pos;
            best_conf = conf;
            best_terr = terr;
        } else if (score > second_score) {
            second_score = score;
            second = pos;
            second_conf = conf;
            second_terr = terr;
        }
    }

    if (!best) {
        train_pos_t *bootstrap = NULL;
        int bootstrap_count = 0;
        for (int i = 0; i < MAX_POS_TRAINS; i++) {
            train_pos_t *pos = &g_pos[i];
            if (!should_consider_for_attr(pos)) continue;
            if (pos->cur_sensor != NULL) continue;
            if (!is_bootstrap_state(pos->route_state)) continue;
            bootstrap = pos;
            bootstrap_count++;
            if (bootstrap_count > 1) break;
        }

        if (bootstrap_count == 1 && bootstrap != NULL) {
            return bootstrap;
        }

        if (bootstrap_count > 1) {
            ambiguous_sensor_count++;
            last_ambiguous_sensor_id = (uint16_t)((int)(hit - g_track) + 1);
            last_ambiguous_time_us = time_us;
            ui_mark_position_dirty();
            return NULL;
        }

        spurious_sensor_count++;
        last_spurious_sensor_id = (uint16_t)((int)(hit - g_track) + 1);
        last_spurious_time_us = time_us;
        ui_mark_position_dirty();
        return NULL;
    }
    if (second_score >= 0 && best_score - second_score <= ATTR_MARGIN) {
       
        train_pos_t *chosen = best;
        int chosen_conf = best_conf;
        int chosen_terr = best_terr;
        int32_t chosen_score = best_score;

        if (second) {
            if (second_conf > chosen_conf) {
                chosen = second;
                chosen_conf = second_conf;
                chosen_terr = second_terr;
                chosen_score = second_score;
            } else if (second_conf == chosen_conf) {
                if (second_terr < chosen_terr) {
                    chosen = second;
                    chosen_terr = second_terr;
                    chosen_score = second_score;
                } else if (second_terr == chosen_terr) {
                    if (second_score > chosen_score ||
                        (second_score == chosen_score &&
                         second->train_num < chosen->train_num)) {
                        chosen = second;
                        chosen_score = second_score;
                    }
                }
            }
        }

        ambiguous_sensor_count++;
        last_ambiguous_sensor_id = (uint16_t)((int)(hit - g_track) + 1);
        last_ambiguous_time_us = time_us;
        ui_mark_position_dirty();
        return chosen;
    }

    return best;
}

void traffic_get_sensor_stats(int *spurious, int *ambiguous) {
    if (spurious) *spurious = spurious_sensor_count;
    if (ambiguous) *ambiguous = ambiguous_sensor_count;
}

void traffic_get_sensor_stats_ex(traffic_sensor_stats_t *out) {
    if (!out) return;
    out->spurious_count = spurious_sensor_count;
    out->ambiguous_count = ambiguous_sensor_count;
    out->last_spurious_sensor_id = last_spurious_sensor_id;
    out->last_ambiguous_sensor_id = last_ambiguous_sensor_id;
    out->last_spurious_time_us = last_spurious_time_us;
    out->last_ambiguous_time_us = last_ambiguous_time_us;
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
    int idx = node_index(node);
    if (idx < 0) return 0;
    return node_owner[idx] == train_num;
}

