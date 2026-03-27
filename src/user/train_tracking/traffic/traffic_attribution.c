#include "train_tracking/traffic_manager.h"
#include "traffic_attr_internal.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

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

static int attr_estimate_speed_mm_s(const train_pos_t *pos) {
    if (!pos) return 0;
    if (pos->effective_v > 0) return pos->effective_v;

    int user_speed = pos->user_speed;
    if (user_speed <= 0 || user_speed > 14) user_speed = pos->goto_speed;
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

static int attr_is_bootstrap_fallback_candidate(const train_pos_t *pos) {
    if (!should_consider_for_attr(pos)) return 0;
    if (!is_bootstrap_state(pos->route_state)) return 0;

    /* Dead-track recovery re-enters FIND_POS while preserving cur_sensor as a
     * stale reservation anchor. Those trains still need the unmatched-hit
     * bootstrap fallback to claim their first real sensor after relaunch. */
    if (pos->route_state == TRAIN_STATE_FIND_POS) return 1;

    return pos->cur_sensor == NULL;
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

typedef struct {
    train_pos_t *pos;
    int32_t      score;
    int          conf;
    int          terr_us;
} attr_ranked_candidate_t;

typedef struct {
    int32_t  score;
    int      conf;
    uint8_t  uses_alt_path;
    uint8_t  uses_route_path;
    uint8_t  uses_last_hit_time;
    int      terr_us;
    uint64_t gate_us;
} attr_candidate_eval_t;

static void attr_eval_init(attr_candidate_eval_t *eval) {
    if (!eval) return;
    *eval = (attr_candidate_eval_t){
        .terr_us = 0x7fffffff,
    };
}

static void attr_eval_set(attr_candidate_eval_t *eval, int32_t score, int conf,
                          uint8_t uses_alt_path, uint8_t uses_route_path,
                          uint8_t uses_last_hit_time, int terr_us,
                          uint64_t gate_us) {
    if (!eval) return;
    *eval = (attr_candidate_eval_t){
        .score = score,
        .conf = conf,
        .uses_alt_path = uses_alt_path,
        .uses_route_path = uses_route_path,
        .uses_last_hit_time = uses_last_hit_time,
        .terr_us = terr_us,
        .gate_us = gate_us,
    };
}

static int attr_try_direct_prediction_candidate(const train_pos_t *pos,
                                                track_node *hit,
                                                attr_candidate_eval_t *eval) {
    if (pos->pred.next_sensor == hit) {
        attr_eval_set(eval, 10000, 3, 0, 0, 0, 0x7fffffff, 0);
        return 1;
    }

    if (pos->pred.alt_sensor == hit) {
        attr_eval_set(eval, 8500, 2, 1, 0, 0, 0x7fffffff, 0);
        return 1;
    }

    return 0;
}

static int attr_try_last_hit_skip_candidate(const train_pos_t *pos,
                                            int32_t dist_mm,
                                            int skip,
                                            uint64_t time_us,
                                            int32_t base_score,
                                            int skip_penalty,
                                            uint8_t uses_alt_path,
                                            uint8_t uses_route_path,
                                            attr_candidate_eval_t *eval) {
    if (skip < 0 || skip > ATTR_MAX_SKIP) return 0;

    int terr = 0x7fffffff;
    uint64_t gate_us = 0;
    if (!attr_time_from_last_hit_ok(pos, dist_mm, skip, time_us, &terr, &gate_us)) return 0;

    attr_eval_set(eval,
                  base_score - skip * skip_penalty - ((dist_mm > 0) ? (dist_mm / 20) : 0),
                  2, uses_alt_path, uses_route_path, 1, terr, gate_us);
    return 1;
}

static int attr_try_current_leg_alt_candidate(const train_pos_t *pos,
                                              track_node *hit,
                                              uint64_t time_us,
                                              attr_candidate_eval_t *eval) {
    int32_t alt_dist = -1;
    int alt_skip = current_leg_alt_branch_skip_to_hit(pos, hit, &alt_dist);
    return attr_try_last_hit_skip_candidate(
        pos, alt_dist, alt_skip, time_us, 8250, 550, 1, 0, eval);
}

static int attr_try_route_path_alt_candidate(const train_pos_t *pos,
                                             track_node *hit,
                                             uint64_t time_us,
                                             attr_candidate_eval_t *eval) {
    int32_t alt_dist = -1;
    int skip = route_path_alt_branch_skip_to_hit(pos, hit, &alt_dist);
    return attr_try_last_hit_skip_candidate(
        pos, alt_dist, skip, time_us, 8000, 550, 1, 1, eval);
}

static int attr_try_route_path_candidate(const train_pos_t *pos,
                                         track_node *hit,
                                         uint64_t time_us,
                                         attr_candidate_eval_t *eval) {
    int32_t path_dist = -1;
    int skip = route_path_skip_to_hit(pos, hit, &path_dist);
    return attr_try_last_hit_skip_candidate(
        pos, path_dist, skip, time_us, 7600, 600, 0, 1, eval);
}

static int attr_try_predicted_forward_fallback_candidate(const train_pos_t *pos,
                                                         track_node *hit,
                                                         attr_candidate_eval_t *eval) {
    if (!pos->pred.next_sensor) return 0;

    int32_t pred_dist = follow_dist(pos->pred.next_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
    if (pred_dist < 0) return 0;

    int hops = sensor_hops_between(pos->pred.next_sensor, hit, 120);
    int skip = (hops >= 0) ? (hops - 1) : 99;
    if (skip > ATTR_MAX_SKIP) return 0;

    attr_eval_set(eval, 7400 - skip * 600 - pred_dist / 20, 2, 0, 0, 0, 0x7fffffff, 0);
    return 1;
}

static int attr_try_forward_progress_candidate(const train_pos_t *pos,
                                               track_node *hit,
                                               uint64_t time_us,
                                               attr_candidate_eval_t *eval) {
    if (attr_has_remaining_route_path(pos) &&
        attr_try_route_path_candidate(pos, hit, time_us, eval)) {
        return 1;
    }

    return attr_try_predicted_forward_fallback_candidate(pos, hit, eval);
}

#if 0
static int attr_try_current_sensor_candidate(const train_pos_t *pos,
                                             track_node *hit,
                                             attr_candidate_eval_t *eval) {
    if (!pos->cur_sensor) return 0;

    int32_t cur_dist = follow_dist(pos->cur_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
    if (cur_dist < 0) return 0;

    attr_eval_set(eval, 2800 - cur_dist / 25, 1, 0, 0, 0, 0x7fffffff, 0);
    return 1;
}
#endif

static int attr_try_near_prediction_candidate(const train_pos_t *pos,
                                              track_node *hit,
                                              attr_candidate_eval_t *eval) {
    if (pos->route_state != TRAIN_STATE_ON_ROUTE ||
        pos->pred.next_sensor == NULL ||
        pos->pred.trigger_time != 0) {
        return 0;
    }

    int32_t to_pred = follow_dist(hit, pos->pred.next_sensor, 10);
    if (to_pred < 0 || to_pred > 800) return 0;

    attr_eval_set(eval, 2600 - to_pred / 20, 1, 0, 0, 0, 0x7fffffff, 0);
    return 1;
}

static int attr_finalize_candidate_eval(train_pos_t *pos, track_node *hit,
                                        uint64_t time_us,
                                        const attr_candidate_eval_t *eval,
                                        attr_ranked_candidate_t *out) {
    if (!eval || eval->score <= 0) return 0;

    int32_t score = eval->score;
    int terr = 0x7fffffff;

    if (eval->uses_last_hit_time) {
        terr = eval->terr_us;
        if ((uint64_t)terr <= eval->gate_us) {
            score += (int32_t)((eval->gate_us - (uint64_t)terr) / 40000ULL);
        } else {
            return 0;
        }
    } else if (pos->pred.trigger_time > 0) {
        terr = abs64((int64_t)time_us - (int64_t)pos->pred.trigger_time);
        if (!eval->uses_alt_path && !eval->uses_route_path &&
            (uint64_t)terr > ATTR_TIME_GATE_US &&
            pos->pred.next_sensor != hit) {
            return 0;
        }
        if (eval->uses_alt_path || eval->uses_route_path) {
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

    if (out) {
        out->pos = pos;
        out->score = score;
        out->conf = eval->conf;
        out->terr_us = terr;
    }
    return 1;
}

static int attr_evaluate_pos_candidate(train_pos_t *pos, track_node *hit,
                                       uint64_t time_us,
                                       attr_ranked_candidate_t *out) {
    if (out) *out = (attr_ranked_candidate_t){0};
    if (!should_consider_for_attr(pos) || !hit || hit->type != NODE_SENSOR) return 0;

    attr_candidate_eval_t eval;
    attr_eval_init(&eval);

    if (!attr_try_direct_prediction_candidate(pos, hit, &eval) &&
        !attr_try_current_leg_alt_candidate(pos, hit, time_us, &eval) &&
        !(attr_has_remaining_route_path(pos) &&
          attr_try_route_path_alt_candidate(pos, hit, time_us, &eval)) &&
        !attr_try_forward_progress_candidate(pos, hit, time_us, &eval) &&
        !attr_try_near_prediction_candidate(pos, hit, &eval)) {
        return 0;
    }

    return attr_finalize_candidate_eval(pos, hit, time_us, &eval, out);
}

static void attr_rank_candidate(const attr_ranked_candidate_t *cand,
                                attr_ranked_candidate_t *best,
                                attr_ranked_candidate_t *second) {
    if (!cand || !cand->pos || !best || !second) return;

    if (cand->score > best->score) {
        *second = *best;
        *best = *cand;
    } else if (cand->score > second->score) {
        *second = *cand;
    }
}

static train_pos_t *attr_choose_ambiguous_owner(const attr_ranked_candidate_t *best,
                                                const attr_ranked_candidate_t *second) {
    if (!best || best->pos == NULL) return NULL;

    train_pos_t *chosen = best->pos;
    int chosen_conf = best->conf;
    int chosen_terr = best->terr_us;
    int32_t chosen_score = best->score;

    if (second && second->pos != NULL) {
        if (second->conf > chosen_conf) {
            chosen = second->pos;
            chosen_conf = second->conf;
            chosen_terr = second->terr_us;
            chosen_score = second->score;
        } else if (second->conf == chosen_conf) {
            if (second->terr_us < chosen_terr) {
                chosen = second->pos;
                chosen_terr = second->terr_us;
                chosen_score = second->score;
            } else if (second->terr_us == chosen_terr) {
                if (second->score > chosen_score ||
                    (second->score == chosen_score &&
                     second->pos->train_num < chosen->train_num)) {
                    chosen = second->pos;
                    chosen_score = second->score;
                }
            }
        }
    }

    return chosen;
}

static traffic_attr_result_t attr_handle_unmatched_hit(track_node *hit,
                                                       uint64_t time_us) {
    traffic_attr_result_t result = traffic_attr_null_result();
    train_pos_t *bootstrap = NULL;
    int bootstrap_count = 0;

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (!attr_is_bootstrap_fallback_candidate(pos)) continue;
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

    attr_ranked_candidate_t best = {.score = -1, .terr_us = 0x7fffffff};
    attr_ranked_candidate_t second = {.score = -1, .terr_us = 0x7fffffff};

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        attr_ranked_candidate_t cand = {.score = -1, .terr_us = 0x7fffffff};
        if (!attr_evaluate_pos_candidate(&g_pos[i], hit, time_us, &cand)) continue;
        attr_rank_candidate(&cand, &best, &second);
    }

    if (best.pos == NULL) {
        return attr_handle_unmatched_hit(hit, time_us);
    }

    if (second.score >= 0 && best.score - second.score <= ATTR_MARGIN) {
        attr_confirm_pending_spurious();
        attr_mark_ambiguous(hit, time_us);
        result.owner = attr_choose_ambiguous_owner(&best, &second);
        return result;
    }

    attr_confirm_pending_spurious();
    result.owner = best.pos;
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
