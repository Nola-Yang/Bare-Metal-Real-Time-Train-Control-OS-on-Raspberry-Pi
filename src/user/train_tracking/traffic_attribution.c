#include "train_tracking/traffic_manager.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "traffic_manager_internal.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

#define ATTR_TIME_GATE_US     2500000ULL
#define ATTR_ALT_TIME_GATE_US 6000000ULL
#define ATTR_MAX_SKIP         2
#define ATTR_MARGIN           120

static int spurious_sensor_count = 0;
static int ambiguous_sensor_count = 0;
static uint16_t last_spurious_sensor_id = 0;
static uint16_t last_ambiguous_sensor_id = 0;
static uint64_t last_spurious_time_us = 0;
static uint64_t last_ambiguous_time_us = 0;

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

static int sensor_hops_between(track_node *from_sensor, track_node *hit_sensor, int max_hops) {
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

static track_node *first_sensor_forward_from(track_node *start, int max_hops,
                                             int32_t *out_dist_mm) {
    if (out_dist_mm) *out_dist_mm = -1;
    if (!start) return NULL;

    track_node *cur = start;
    int32_t dist_mm = 0;
    for (int h = 0; h < max_hops; h++) {
        if (cur->type == NODE_SENSOR) {
            if (out_dist_mm) *out_dist_mm = dist_mm;
            return cur;
        }
        if (cur->type == NODE_EXIT) return NULL;

        track_edge *e = traffic_tm_get_next_edge(cur);
        if (!e || !e->dest) return NULL;
        dist_mm += (int32_t)e->dist;
        cur = e->dest;
    }

    if (cur->type == NODE_SENSOR) {
        if (out_dist_mm) *out_dist_mm = dist_mm;
        return cur;
    }
    return NULL;
}

/* Rebuild the first alternate-branch sensor between cur_sensor and the current
 * predicted next sensor. This is a fallback when pred.alt_sensor went stale or
 * was never cached, while the software switch state still describes the planned
 * route up to the next sensor. */
static int current_leg_alt_branch_match(const train_pos_t *pos, track_node *hit,
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

        int sw_idx = track_switch_to_index(cur->num);
        char state = (sw_idx >= 0) ? track_get_switch_state()[sw_idx].state : '?';
        if (state != 'S' && state != 'C') return -1;

        int alt_dir = (state == 'S') ? DIR_CURVED : DIR_STRAIGHT;
        int32_t alt_tail_mm = -1;
        track_node *alt_sensor = first_sensor_forward_from(cur->edge[alt_dir].dest,
                                                           20, &alt_tail_mm);
        if (!alt_sensor || alt_sensor != hit) continue;

        if (out_dist_mm) {
            *out_dist_mm = dist_to_branch_mm + (int32_t)cur->edge[alt_dir].dist;
            if (alt_tail_mm > 0) *out_dist_mm += alt_tail_mm;
        }
        return 0;
    }
    return -1;
}

/* Scan the remaining planned path for a later branch whose unplanned leg leads
 * directly to the hit sensor. This keeps attribution working when the turnout
 * failure happens beyond the currently cached pred.alt_sensor. */
static int route_path_alt_branch_match(const train_pos_t *pos, track_node *hit,
                                       int32_t *out_dist_mm) {
    if (out_dist_mm) *out_dist_mm = -1;
    if (!pos || !hit || pos->route_path_count <= 1) return -1;

    int start = pos->route_path_cursor;
    if (start < 0) start = 0;
    if (start >= pos->route_path_count - 1) return -1;

    int32_t dist_mm = 0;
    int main_sensor_hops = 0;
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
        if (node->type == NODE_SENSOR && i > start) main_sensor_hops++;
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
        track_node *alt_sensor = first_sensor_forward_from(node->edge[alt_dir].dest,
                                                           20, &alt_tail_mm);
        if (!alt_sensor || alt_sensor != hit) continue;

        if (out_dist_mm) {
            *out_dist_mm = dist_mm + (int32_t)node->edge[alt_dir].dist;
            if (alt_tail_mm > 0) *out_dist_mm += alt_tail_mm;
        }
        return main_sensor_hops;
    }

    return -1;
}

static int route_path_skip_to_hit(const train_pos_t *pos, track_node *hit,
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

void traffic_attr_init(void) {
    spurious_sensor_count = 0;
    ambiguous_sensor_count = 0;
    last_spurious_sensor_id = 0;
    last_ambiguous_sensor_id = 0;
    last_spurious_time_us = 0;
    last_ambiguous_time_us = 0;
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
        int uses_alt_path = 0;

        if (pos->pred.next_sensor == hit) {
            score = 10000;
            conf = 3;
            has_candidate = 1;
        } else if (pos->pred.alt_sensor == hit) {
            score = 8500;
            conf = 2;
            has_candidate = 1;
            uses_alt_path = 1;
        }

        if (!has_candidate) {
            int32_t alt_dist = -1;
            int alt_skip = current_leg_alt_branch_match(pos, hit, &alt_dist);
            if (alt_skip >= 0) {
                score = 8250 - alt_skip * 550 - ((alt_dist > 0) ? (alt_dist / 20) : 0);
                conf = 2;
                has_candidate = 1;
                uses_alt_path = 1;
            }
        }

        if (!has_candidate && pos->pred.alt_sensor != NULL) {
            int32_t alt_dist = follow_dist(pos->pred.alt_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
            int32_t pred_dist = -1;
            if (pos->pred.next_sensor != NULL) {
                pred_dist = follow_dist(pos->pred.next_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
            }
            if (alt_dist >= 0 && pred_dist < 0) {
                int hops = sensor_hops_between(pos->pred.alt_sensor, hit, 120);
                int skip = (hops >= 0) ? (hops - 1) : 99;
                if (skip <= ATTR_MAX_SKIP) {
                    score = 7900 - skip * 550 - alt_dist / 20;
                    conf = 2;
                    has_candidate = 1;
                    uses_alt_path = 1;
                }
            }
        }

        if (!has_candidate && pos->route_state == TRAIN_STATE_ON_ROUTE) {
            int32_t alt_dist = -1;
            int skip = route_path_alt_branch_match(pos, hit, &alt_dist);
            if (skip >= 0 && skip <= ATTR_MAX_SKIP) {
                score = 8000 - skip * 550 - ((alt_dist > 0) ? (alt_dist / 20) : 0);
                conf = 2;
                has_candidate = 1;
                uses_alt_path = 1;
            }
        }

        if (!has_candidate && pos->route_state == TRAIN_STATE_ON_ROUTE) {
            int32_t path_dist = -1;
            int skip = route_path_skip_to_hit(pos, hit, &path_dist);
            if (skip >= 0 && skip <= ATTR_MAX_SKIP) {
                score = 7600 - skip * 600 - ((path_dist > 0) ? (path_dist / 20) : 0);
                conf = 2;
                has_candidate = 1;
            }
        }

        if (!has_candidate && pos->pred.next_sensor) {
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
            if (!uses_alt_path &&
                (uint64_t)terr > ATTR_TIME_GATE_US &&
                pos->pred.next_sensor != hit) {
                continue;
            }
            if (uses_alt_path) {
                if ((uint64_t)terr <= ATTR_ALT_TIME_GATE_US) {
                    score += (int32_t)((ATTR_ALT_TIME_GATE_US - (uint64_t)terr) / 40000ULL);
                } else {
                    score -= 100;
                }
            } else if ((uint64_t)terr <= ATTR_TIME_GATE_US) {
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
