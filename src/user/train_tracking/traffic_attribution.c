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
#define ATTR_SKIP_TIME_SLACK_US 500000ULL
#define ATTR_RESCUE_AMBIG_US  250000

typedef struct {
    track_node *sensor;
    uint64_t    time_us;
    uint8_t     active;
} pending_spurious_t;

static int confirmed_spurious_sensor_count = 0;
static int ambiguous_sensor_count = 0;
static uint16_t last_confirmed_spurious_sensor_id = 0;
static uint16_t last_ambiguous_sensor_id = 0;
static uint64_t last_confirmed_spurious_time_us = 0;
static uint64_t last_ambiguous_time_us = 0;
static pending_spurious_t pending_spurious = {0};

static int abs64(int64_t x) {
    return (x < 0) ? (int)(-x) : (int)x;
}

static int abs_time_us(uint64_t a, uint64_t b) {
    return (a >= b) ? (int)(a - b) : (int)(b - a);
}

static int attr_estimate_speed_mm_s(const train_pos_t *pos) {
    if (!pos) return 0;
    if (pos->effective_v > 0) return pos->effective_v;

    int user_speed = pos->user_speed;
    if (user_speed <= 0 || user_speed > 14) user_speed = GOTO_USER_SPEED;
    return speed_table_get_v(pos->train_ind, user_speed);
}

static int attr_time_from_last_hit_ok(const train_pos_t *pos, int32_t dist_mm,
                                      int skip_count, uint64_t time_us,
                                      int *out_terr, uint64_t *out_gate_us) {
    if (out_terr) *out_terr = 0x7fffffff;
    if (out_gate_us) *out_gate_us = 0;
    if (!pos || !pos->cur_sensor || pos->cur_sensor_time == 0 || dist_mm < 0) return 0;

    int32_t v = attr_estimate_speed_mm_s(pos);
    if (v <= 0) return 0;

    uint64_t dt_us = (uint64_t)((int64_t)dist_mm * 1000000LL / (int64_t)v);
    uint64_t expected_us = pos->cur_sensor_time + dt_us;
    int terr = abs_time_us(time_us, expected_us);

    uint64_t gate_us = ATTR_TIME_GATE_US +
                       (uint64_t)skip_count * ATTR_SKIP_TIME_SLACK_US +
                       dt_us / 4;
    if (gate_us > ATTR_ALT_TIME_GATE_US) gate_us = ATTR_ALT_TIME_GATE_US;

    if (out_terr) *out_terr = terr;
    if (out_gate_us) *out_gate_us = gate_us;
    return ((uint64_t)terr <= gate_us);
}

static int should_consider_for_attr(const train_pos_t *pos) {
    if (!pos || pos->train_num < 0) return 0;
    if (pos->route_state == TRAIN_STATE_STOPPED ||
        pos->route_state == TRAIN_STATE_DEAD_TRACK ||
        pos->route_state == TRAIN_STATE_WAIT_SWITCH_SETTLE ||
        pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
        return 0;
    }
    return 1;
}

static int attr_has_remaining_route_path(const train_pos_t *pos) {
    if (!pos) return 0;
    return pos->route_state == TRAIN_STATE_ON_ROUTE ||
           pos->route_state == TRAIN_STATE_STOPPING;
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
static int current_leg_alt_branch_skip_to_hit(const train_pos_t *pos,
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

        int sw_idx = track_switch_to_index(cur->num);
        char state = (sw_idx >= 0) ? track_get_switch_state()[sw_idx].state : '?';
        if (state != 'S' && state != 'C') return -1;

        int alt_dir = (state == 'S') ? DIR_CURVED : DIR_STRAIGHT;
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
static int route_path_alt_branch_skip_to_hit(const train_pos_t *pos,
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

static traffic_attr_result_t traffic_attr_null_result(void) {
    traffic_attr_result_t out = {0};
    return out;
}

static void attr_confirm_pending_spurious(void) {
    if (!pending_spurious.active || pending_spurious.sensor == NULL) return;

    confirmed_spurious_sensor_count++;
    last_confirmed_spurious_sensor_id =
        (uint16_t)((int)(pending_spurious.sensor - g_track) + 1);
    last_confirmed_spurious_time_us = pending_spurious.time_us;
    pending_spurious.sensor = NULL;
    pending_spurious.time_us = 0;
    pending_spurious.active = 0;
    ui_mark_position_dirty();
}

static void attr_set_pending_spurious(track_node *sensor, uint64_t time_us) {
    if (!sensor || sensor->type != NODE_SENSOR) return;
    pending_spurious.sensor = sensor;
    pending_spurious.time_us = time_us;
    pending_spurious.active = 1;
    ui_mark_position_dirty();
}

static void attr_clear_pending_spurious(void) {
    if (!pending_spurious.active) return;
    pending_spurious.sensor = NULL;
    pending_spurious.time_us = 0;
    pending_spurious.active = 0;
    ui_mark_position_dirty();
}

static void attr_mark_ambiguous(track_node *hit, uint64_t time_us) {
    ambiguous_sensor_count++;
    last_ambiguous_sensor_id = (uint16_t)((int)(hit - g_track) + 1);
    last_ambiguous_time_us = time_us;
    ui_mark_position_dirty();
}

static int sensors_form_adjacent_pair(track_node *first,
                                      track_node *second,
                                      int32_t *out_dist_mm) {
    if (out_dist_mm) *out_dist_mm = -1;
    if (!first || !second) return 0;
    if (first->type != NODE_SENSOR || second->type != NODE_SENSOR) return 0;

    int32_t dist_mm = follow_dist(first, second, 80);
    if (dist_mm < 0) return 0;
    if (sensor_hops_between(first, second, 80) != 1) return 0;

    if (out_dist_mm) *out_dist_mm = dist_mm;
    return 1;
}

static track_node *attr_nth_sensor_forward(track_node *start,
                                           int nth,
                                           int max_hops,
                                           int32_t *out_dist_mm) {
    if (out_dist_mm) *out_dist_mm = -1;
    if (!start || nth <= 0) return NULL;

    track_node *cur = start;
    int32_t dist_mm = 0;
    int sensor_count = 0;

    for (int h = 0; h < max_hops; h++) {
        track_edge *e = traffic_tm_get_next_edge(cur);
        if (!e || !e->dest) return NULL;
        dist_mm += (int32_t)e->dist;
        cur = e->dest;
        if (cur->type == NODE_SENSOR) {
            sensor_count++;
            if (sensor_count == nth) {
                if (out_dist_mm) *out_dist_mm = dist_mm;
                return cur;
            }
        }
        if (cur->type == NODE_EXIT) return NULL;
    }

    return NULL;
}

static uint64_t attr_isqrt_u64(uint64_t x) {
    uint64_t res = 0;
    uint64_t bit = 1ULL << 62;

    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

static int attr_travel_time_from_launch_us(const train_pos_t *pos,
                                           int32_t dist_mm,
                                           uint64_t *out_dt_us) {
    if (out_dt_us) *out_dt_us = 0;
    if (!pos || dist_mm < 0) return 0;

    int32_t a = pos->accel_a_eff;
    int32_t v_end = speed_table_get_v(pos->train_ind, GOTO_USER_SPEED);

    if (a <= 0 || v_end <= 0) return 0;

    int32_t d_accel = (int32_t)((int64_t)v_end * (int64_t)v_end / (2LL * (int64_t)a));
    uint64_t dt_us = 0;

    if (dist_mm >= d_accel) {
        uint64_t t_accel_us = (uint64_t)((int64_t)v_end * 1000000LL / (int64_t)a);
        uint64_t t_cruise_us = (uint64_t)((int64_t)(dist_mm - d_accel) * 1000000LL /
                                          (int64_t)v_end);
        dt_us = t_accel_us + t_cruise_us;
    } else {
        uint64_t sq_us = (2000000000000ULL * (uint64_t)dist_mm) / (uint64_t)a;
        dt_us = attr_isqrt_u64(sq_us);
    }

    if (out_dt_us) *out_dt_us = dt_us;
    return 1;
}

static int attr_expected_sensor_time(const train_pos_t *pos,
                                     track_node *sensor,
                                     uint64_t *out_expected_us) {
    if (out_expected_us) *out_expected_us = 0;
    if (!pos || !sensor || !pos->cur_sensor || pos->accel_start_us == 0) return 0;

    int32_t dist_mm = follow_dist(pos->cur_sensor, sensor, OFF_ROUTE_PATH_MAX_HOPS);
    if (dist_mm < 0) return 0;

    uint64_t dt_us = 0;
    if (!attr_travel_time_from_launch_us(pos, dist_mm, &dt_us)) return 0;

    if (out_expected_us) *out_expected_us = pos->accel_start_us + dt_us;
    return 1;
}

static int attr_expected_third_sensor_time(const train_pos_t *pos,
                                           uint64_t *out_expected_us,
                                           track_node **out_third_sensor) {
    if (out_expected_us) *out_expected_us = 0;
    if (out_third_sensor) *out_third_sensor = NULL;
    if (!pos || !pos->cur_sensor) return 0;

    int32_t dist_mm = -1;
    track_node *third = attr_nth_sensor_forward(pos->cur_sensor, 3, 120, &dist_mm);
    if (!third || dist_mm < 0) return 0;

    uint64_t dt_us = 0;
    if (!attr_travel_time_from_launch_us(pos, dist_mm, &dt_us)) return 0;

    if (out_expected_us) *out_expected_us = pos->accel_start_us + dt_us;
    if (out_third_sensor) *out_third_sensor = third;
    return 1;
}

static int attr_is_stale_onroute_candidate(const train_pos_t *pos,
                                           uint64_t now_us,
                                           uint8_t *out_revive_dead_track) {
    if (out_revive_dead_track) *out_revive_dead_track = 0;
    if (!pos || pos->train_num < 0 || pos->cur_sensor == NULL) return 0;
    if (pos->accel_start_us == 0 || pos->cur_sensor_time >= pos->accel_start_us) return 0;

    if (pos->route_state == TRAIN_STATE_ON_ROUTE) {
        if (out_revive_dead_track) *out_revive_dead_track = 0;
    } else if (pos->route_state == TRAIN_STATE_DEAD_TRACK &&
               pos->dead_track_recover.valid) {
        if (out_revive_dead_track) *out_revive_dead_track = 1;
    } else {
        return 0;
    }

    uint64_t expected_us = 0;
    if (!attr_expected_third_sensor_time(pos, &expected_us, NULL)) return 0;
    return now_us >= expected_us;
}

typedef struct {
    train_pos_t *pos;
    int          priority;
    int          terr_us;
    int32_t      pending_dist_mm;
    uint8_t      revive_dead_track;
} attr_pair_candidate_t;

static int attr_pair_candidate_better(const attr_pair_candidate_t *a,
                                      const attr_pair_candidate_t *b) {
    if (a->pos == NULL) return 0;
    if (b->pos == NULL) return 1;
    if (a->priority != b->priority) return a->priority > b->priority;
    if (a->terr_us != b->terr_us) return a->terr_us < b->terr_us;
    if (a->pending_dist_mm != b->pending_dist_mm) {
        return a->pending_dist_mm < b->pending_dist_mm;
    }
    return a->pos->train_num < b->pos->train_num;
}

static train_pos_t *attr_pick_stale_train_for_pair(track_node *pending_sensor,
                                                   track_node *current_hit,
                                                   uint64_t now_us,
                                                   int *out_ambiguous,
                                                   uint8_t *out_revive_dead_track) {
    if (out_ambiguous) *out_ambiguous = 0;
    if (out_revive_dead_track) *out_revive_dead_track = 0;

    attr_pair_candidate_t best = {0};
    attr_pair_candidate_t second = {0};

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        uint8_t revive = 0;
        if (!attr_is_stale_onroute_candidate(pos, now_us, &revive)) continue;

        int32_t pending_dist_mm =
            follow_dist(pos->cur_sensor, pending_sensor, OFF_ROUTE_PATH_MAX_HOPS);
        if (pending_dist_mm < 0) continue;
        if (follow_dist(pending_sensor, current_hit, OFF_ROUTE_PATH_MAX_HOPS) < 0) continue;

        uint64_t expected_hit_us = 0;
        if (!attr_expected_sensor_time(pos, current_hit, &expected_hit_us)) continue;

        attr_pair_candidate_t cand = {
            .pos = pos,
            .priority = (pos->route_state == TRAIN_STATE_ON_ROUTE) ? 2 : 1,
            .terr_us = abs_time_us(now_us, expected_hit_us),
            .pending_dist_mm = pending_dist_mm,
            .revive_dead_track = revive,
        };

        if (attr_pair_candidate_better(&cand, &best)) {
            second = best;
            best = cand;
        } else if (attr_pair_candidate_better(&cand, &second)) {
            second = cand;
        }
    }

    if (best.pos == NULL) return NULL;

    if (second.pos != NULL &&
        second.priority == best.priority &&
        abs64((int64_t)best.terr_us - (int64_t)second.terr_us) <= ATTR_RESCUE_AMBIG_US) {
        if (out_ambiguous) *out_ambiguous = 1;
        return NULL;
    }

    if (out_revive_dead_track) *out_revive_dead_track = best.revive_dead_track;
    return best.pos;
}

void traffic_attr_init(void) {
    confirmed_spurious_sensor_count = 0;
    ambiguous_sensor_count = 0;
    last_confirmed_spurious_sensor_id = 0;
    last_ambiguous_sensor_id = 0;
    last_confirmed_spurious_time_us = 0;
    last_ambiguous_time_us = 0;
    pending_spurious.sensor = NULL;
    pending_spurious.time_us = 0;
    pending_spurious.active = 0;
}

traffic_attr_result_t traffic_attribute_sensor(track_node *hit, uint64_t time_us) {
    traffic_attr_result_t result = traffic_attr_null_result();
    if (!hit || hit->type != NODE_SENSOR) return result;

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
        int uses_route_path = 0;
        int uses_last_hit_time = 0;
        int candidate_terr = 0x7fffffff;
        uint64_t candidate_gate_us = 0;

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
            int terr = 0x7fffffff;
            uint64_t gate_us = 0;
            int alt_skip = current_leg_alt_branch_skip_to_hit(pos, hit, &alt_dist);
            if (alt_skip >= 0 && alt_skip <= ATTR_MAX_SKIP &&
                attr_time_from_last_hit_ok(pos, alt_dist, alt_skip, time_us,
                                           &terr, &gate_us)) {
                score = 8250 - alt_skip * 550 - ((alt_dist > 0) ? (alt_dist / 20) : 0);
                conf = 2;
                has_candidate = 1;
                uses_alt_path = 1;
                uses_last_hit_time = 1;
                candidate_terr = terr;
                candidate_gate_us = gate_us;
            }
        }

        if (!has_candidate && attr_has_remaining_route_path(pos)) {
            int32_t alt_dist = -1;
            int skip = route_path_alt_branch_skip_to_hit(pos, hit, &alt_dist);
            int terr = 0x7fffffff;
            uint64_t gate_us = 0;
            if (skip >= 0 && skip <= ATTR_MAX_SKIP &&
                attr_time_from_last_hit_ok(pos, alt_dist, skip, time_us,
                                           &terr, &gate_us)) {
                score = 8000 - skip * 550 - ((alt_dist > 0) ? (alt_dist / 20) : 0);
                conf = 2;
                has_candidate = 1;
                uses_alt_path = 1;
                uses_route_path = 1;
                uses_last_hit_time = 1;
                candidate_terr = terr;
                candidate_gate_us = gate_us;
            }
        }

        if (!has_candidate && attr_has_remaining_route_path(pos)) {
            int32_t path_dist = -1;
            int skip = route_path_skip_to_hit(pos, hit, &path_dist);
            int terr = 0x7fffffff;
            uint64_t gate_us = 0;
            if (skip >= 0 && skip <= ATTR_MAX_SKIP &&
                attr_time_from_last_hit_ok(pos, path_dist, skip, time_us,
                                           &terr, &gate_us)) {
                score = 7600 - skip * 600 - ((path_dist > 0) ? (path_dist / 20) : 0);
                conf = 2;
                has_candidate = 1;
                uses_route_path = 1;
                uses_last_hit_time = 1;
                candidate_terr = terr;
                candidate_gate_us = gate_us;
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
        if (uses_last_hit_time) {
            terr = candidate_terr;
            if ((uint64_t)terr <= candidate_gate_us) {
                score += (int32_t)((candidate_gate_us - (uint64_t)terr) / 40000ULL);
            } else {
                continue;
            }
        } else if (pos->pred.trigger_time > 0) {
            terr = abs64((int64_t)time_us - (int64_t)pos->pred.trigger_time);
            if (!uses_alt_path && !uses_route_path &&
                (uint64_t)terr > ATTR_TIME_GATE_US &&
                pos->pred.next_sensor != hit) {
                continue;
            }
            if (uses_alt_path || uses_route_path) {
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
            attr_confirm_pending_spurious();
            result.owner = bootstrap;
            return result;
        }

        if (bootstrap_count > 1) {
            attr_confirm_pending_spurious();
            attr_mark_ambiguous(hit, time_us);
            return result;
        }

        if (pending_spurious.active) {
            int32_t pair_dist_mm = -1;
            if (sensors_form_adjacent_pair(pending_spurious.sensor, hit, &pair_dist_mm) &&
                pair_dist_mm >= 0) {
                uint8_t revive_dead_track = 0;
                train_pos_t *rescued = attr_pick_stale_train_for_pair(
                    pending_spurious.sensor, hit, time_us,
                    NULL, &revive_dead_track);
                if (rescued != NULL) {
                    attr_clear_pending_spurious();
                    result.owner = rescued;
                    result.rescued_from_pair = 1;
                    result.revive_dead_track = revive_dead_track;
                    return result;
                }
            }
            attr_confirm_pending_spurious();
        }

        attr_set_pending_spurious(hit, time_us);
        return result;
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

        attr_confirm_pending_spurious();
        attr_mark_ambiguous(hit, time_us);
        result.owner = chosen;
        return result;
    }

    attr_confirm_pending_spurious();
    result.owner = best;
    return result;
}

void traffic_get_sensor_stats(int *spurious, int *ambiguous) {
    if (spurious) {
        *spurious = confirmed_spurious_sensor_count + (pending_spurious.active ? 1 : 0);
    }
    if (ambiguous) *ambiguous = ambiguous_sensor_count;
}

void traffic_get_sensor_stats_ex(traffic_sensor_stats_t *out) {
    if (!out) return;
    out->spurious_count = confirmed_spurious_sensor_count +
                          (pending_spurious.active ? 1 : 0);
    out->ambiguous_count = ambiguous_sensor_count;
    out->last_spurious_sensor_id = pending_spurious.active && pending_spurious.sensor != NULL
        ? (uint16_t)((int)(pending_spurious.sensor - g_track) + 1)
        : last_confirmed_spurious_sensor_id;
    out->last_ambiguous_sensor_id = last_ambiguous_sensor_id;
    out->last_spurious_time_us = pending_spurious.active
        ? pending_spurious.time_us
        : last_confirmed_spurious_time_us;
    out->last_ambiguous_time_us = last_ambiguous_time_us;
}
