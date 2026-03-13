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

static void clear_attr_diagnostics(void) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        if (g_pos[i].train_num < 0) continue;
        g_pos[i].last_attr_score = 0;
        g_pos[i].last_attr_conf = 0;
    }
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
            st == TRAIN_STATE_LOOP_FIND_DIR);
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
}

void traffic_build_constraints(int requester_train, uint8_t blocked[TRACK_MAX]) {
    if (!blocked) return;
    for (int i = 0; i < TRACK_MAX; i++) {
        int owner = node_owner[i];
        blocked[i] = (owner >= 0 && owner != requester_train) ? 1 : 0;
    }
}

int traffic_reserve_plan(int train_num, track_node *start, const route_plan_t *plan) {
    (void)start;
    if (!plan || train_num < 0) return 0;

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

void traffic_release_train_keep_position(int train_num, track_node *cur) {
    int keep0 = cur ? node_index(cur) : -1;
    int keep1 = (keep0 >= 0) ? reverse_index(keep0) : -1;
    int changed = 0;

    for (int i = 0; i < TRACK_MAX; i++) {
        if (node_owner[i] != train_num) continue;
        if (i == keep0 || i == keep1) continue;
        node_owner[i] = -1;
        changed = 1;
    }
   
    if (keep0 >= 0 && node_owner[keep0] != train_num) {
        node_owner[keep0] = train_num;
        changed = 1;
    }
    if (keep1 >= 0 && node_owner[keep1] != train_num) {
        node_owner[keep1] = train_num;
        changed = 1;
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

    clear_attr_diagnostics();

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

        if (pos->pred_next_sensor == hit) {
            score = 10000;
            conf = 3;
            has_candidate = 1;
        } else if (pos->pred_alt_sensor == hit) {
            score = 8500;
            conf = 2;
            has_candidate = 1;
        } else if (pos->pred_next_sensor) {
            int32_t pred_dist = follow_dist(pos->pred_next_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
            if (pred_dist >= 0) {
                int hops = sensor_hops_between(pos->pred_next_sensor, hit, 120);
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

        if (!has_candidate || score <= 0) continue;

        int terr = 0x7fffffff;
        if (pos->pred_trigger_time > 0) {
            terr = abs64((int64_t)time_us - (int64_t)pos->pred_trigger_time);
            if ((uint64_t)terr > ATTR_TIME_GATE_US && pos->pred_next_sensor != hit) {
                continue;
            }
            if ((uint64_t)terr <= ATTR_TIME_GATE_US) {
                score += (int32_t)((ATTR_TIME_GATE_US - (uint64_t)terr) / 20000ULL);
            } else {
                score -= 200;
            }
        }

        pos->last_attr_score = score;
        pos->last_attr_conf = conf;

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
            bootstrap->last_attr_score = 1;
            bootstrap->last_attr_conf = 1;
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
        chosen->last_attr_conf = (chosen_conf > 0) ? chosen_conf : 1;
        ui_mark_position_dirty();
        return chosen;
    }

    best->last_attr_conf = 3;
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

int traffic_get_reserved_train_list(int *out_trains, int max_trains) {
    int owners[8];
    int n = 0;
    if (max_trains < 0) max_trains = 0;

    for (int i = 0; i < TRACK_MAX; i++) {
        int owner = node_owner[i];
        if (owner < 0) continue;
        int seen = 0;
        for (int j = 0; j < n; j++) {
            if (owners[j] == owner) {
                seen = 1;
                break;
            }
        }
        if (!seen && n < (int)(sizeof(owners) / sizeof(owners[0]))) {
            owners[n++] = owner;
        }
    }

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (owners[j] < owners[i]) {
                int t = owners[i];
                owners[i] = owners[j];
                owners[j] = t;
            }
        }
    }

    if (out_trains) {
        int m = (n < max_trains) ? n : max_trains;
        for (int i = 0; i < m; i++) out_trains[i] = owners[i];
    }
    return n;
}
