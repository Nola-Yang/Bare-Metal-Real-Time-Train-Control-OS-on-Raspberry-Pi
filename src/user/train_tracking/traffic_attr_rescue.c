#include "traffic_attr_internal.h"

int sensors_form_adjacent_pair(track_node *first,
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
    int32_t v_end = speed_table_get_v(pos->train_ind, pos->goto_speed);

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

train_pos_t *attr_pick_stale_train_for_pair(track_node *pending_sensor,
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
