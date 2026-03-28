#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include <stddef.h>
#include <stdint.h>

static pos_route_eval_t g_pos_try_eval_candidate;
static pos_route_eval_t g_pos_try_eval_force_move;
static route_plan_t g_pos_try_launch_prefix;
static int g_pos_try_snapshot[TRACK_MAX];
static uint16_t g_pos_try_sorted_from_origin0[TRACK_MAX];
static uint16_t g_pos_try_sorted_from_origin1[TRACK_MAX];
static uint16_t g_pos_try_merged_candidates[TRACK_MAX];
static uint8_t g_pos_try_seen_candidate[TRACK_MAX];

static int pos_deadlock_same_physical_sensor(const track_node *a,
                                             const track_node *b) {
    if (!a || !b) return 0;
    return a == b || a->reverse == b || b->reverse == a;
}

static int pos_deadlock_candidate_in_yield_history(const train_pos_t *pos,
                                                   const track_node *cand) {
    if (!pos || !cand) return 0;

    for (int i = 0; i < pos->deadlock_recover.yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        if (pos_deadlock_same_physical_sensor(pos->deadlock_recover.yield_history[i],
                                              cand)) {
            return 1;
        }
    }
    return 0;
}

static int32_t pos_deadlock_candidate_sort_dist(track_node *origins[2],
                                                track_node *cand) {
    int32_t best = -1;

    if (!cand) return -1;

    for (int i = 0; i < 2; i++) {
        int32_t dist;
        if (!origins[i]) continue;
        dist = route_direct_sensor_dist(origins[i], cand);
        if (dist < 0) continue;
        if (best < 0 || dist < best) best = dist;
    }
    return best;
}

static int pos_deadlock_merge_sorted_candidates(track_node *origins[2]) {
    int count0 = 0;
    int count1 = 0;
    int i0 = 0;
    int i1 = 0;
    int out_count = 0;

    for (int i = 0; i < TRACK_MAX; i++) g_pos_try_seen_candidate[i] = 0;

    if (origins[0]) {
        count0 = route_fill_sorted_direct_sensor_candidates(
            origins[0], g_pos_try_sorted_from_origin0, TRACK_MAX);
    }
    if (origins[1]) {
        count1 = route_fill_sorted_direct_sensor_candidates(
            origins[1], g_pos_try_sorted_from_origin1, TRACK_MAX);
    }

    while (i0 < count0 || i1 < count1) {
        track_node *cand0 = NULL;
        track_node *cand1 = NULL;
        int32_t dist0 = -1;
        int32_t dist1 = -1;
        track_node *chosen;
        int idx;

        if (i0 < count0) {
            cand0 = &g_track[g_pos_try_sorted_from_origin0[i0]];
            dist0 = pos_deadlock_candidate_sort_dist(origins, cand0);
        }
        if (i1 < count1) {
            cand1 = &g_track[g_pos_try_sorted_from_origin1[i1]];
            dist1 = pos_deadlock_candidate_sort_dist(origins, cand1);
        }

        if (!cand1 ||
            (cand0 != NULL &&
             (dist1 < 0 || dist0 < dist1 ||
              (dist0 == dist1 &&
               g_pos_try_sorted_from_origin0[i0] <
                   g_pos_try_sorted_from_origin1[i1])))) {
            chosen = cand0;
            i0++;
        } else {
            chosen = cand1;
            i1++;
        }

        if (!chosen) continue;
        idx = (int)(chosen - g_track);
        if (idx < 0 || idx >= TRACK_MAX) continue;
        if (g_pos_try_seen_candidate[idx]) continue;

        g_pos_try_seen_candidate[idx] = 1;
        g_pos_try_merged_candidates[out_count++] = (uint16_t)idx;
    }

    return out_count;
}

static uint8_t pos_simulate_deadlock_unblocked_mask(train_pos_t *victim,
                                                    uint8_t cycle_mask,
                                                    track_node *yield_target) {
    uint8_t ready_mask = 0;
    uint8_t victim_bit;
    track_node *keep_end;

    if (!victim || !yield_target) return 0;

    victim_bit = pos_deadlock_train_bit(victim->train_num);
    keep_end = pos_release_keep_end(yield_target, NULL);
    traffic_snapshot_reservations(g_pos_try_snapshot);
    traffic_simulate_parked_train(victim->train_num, yield_target,
                                  TRAIN_BODY_MM, keep_end);

    for (int i = 0; i < 6; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        train_pos_t *other;
        track_node *target;

        if (!(cycle_mask & bit)) continue;
        if (pos_deadlock_index_to_train(i) == victim->train_num) continue;

        other = pos_get(pos_deadlock_index_to_train(i));
        if (!other || other->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        if (!(other->replan.blocker_mask & victim_bit)) continue;

        pos_restore_pending_target(other);
        target = pos_route_current_goal(other);
        if (!target) continue;

        if (pos_evaluate_target_ready_now(other, target, NULL) == POS_ROUTE_EVAL_READY) {
            ready_mask |= bit;
        }
    }

    traffic_restore_reservations(g_pos_try_snapshot);
    return ready_mask;
}

static int pos_deadlock_candidate_can_force_move(train_pos_t *pos,
                                                 track_node *target) {
    pos_route_eval_result_t result;
    int reserved_end_cursor = -1;
    int switch_blocker_owner = -1;

    if (!pos || !target) return 0;

    result = pos_evaluate_target_plan(pos, target, &g_pos_try_eval_force_move);
    if (result != POS_ROUTE_EVAL_READY) return 0;

    return pos_route_authority_prepare_launch_strict(
        pos, &g_pos_try_eval_force_move.plan,
        &g_pos_try_launch_prefix,
        &reserved_end_cursor,
        &switch_blocker_owner);
}

int pos_pick_deadlock_yield_target(train_pos_t *pos, uint8_t cycle_mask,
                                   track_node **out_target,
                                   uint8_t *out_unblocked_mask,
                                   pos_deadlock_pick_kind_t *out_kind) {
    track_node *origins[2];
    track_node *current_target;
    track_node *best_unlock_target = NULL;
    track_node *best_ready_reloc_target = NULL;
    uint8_t best_unblocked_mask = 0;
    uint8_t fallback_wait_mask;
    int merged_count;
    int32_t min_dist_mm;

    if (out_target) *out_target = NULL;
    if (out_unblocked_mask) *out_unblocked_mask = 0;
    if (out_kind) *out_kind = POS_DEADLOCK_PICK_NONE;
    if (!pos || !pos->cur_sensor) return 0;

    pos_route_fill_origins(pos, origins);
    if (!origins[0] && !origins[1]) return 0;

    current_target = pos_route_current_goal(pos);
    fallback_wait_mask = cycle_mask & (uint8_t)~pos_deadlock_train_bit(pos->train_num);
    min_dist_mm = pos_route_authority_min_mm(pos);
    merged_count = pos_deadlock_merge_sorted_candidates(origins);

    for (int i = 0; i < merged_count; i++) {
        track_node *cand = &g_track[g_pos_try_merged_candidates[i]];
        uint8_t unblocked_mask;
        int32_t sort_dist;

        sort_dist = pos_deadlock_candidate_sort_dist(origins, cand);
        if (sort_dist < 0 || sort_dist < min_dist_mm) continue;
        if (pos_deadlock_same_physical_sensor(cand, pos->cur_sensor)) continue;
        if (pos_deadlock_same_physical_sensor(cand, current_target)) continue;
        if (pos_deadlock_candidate_in_yield_history(pos, cand)) continue;

        if (pos_evaluate_target_ready_now(pos, cand,
                                          &g_pos_try_eval_candidate) != POS_ROUTE_EVAL_READY) {
            continue;
        }

        best_ready_reloc_target = cand;
        unblocked_mask = pos_simulate_deadlock_unblocked_mask(pos, cycle_mask, cand);
        if (unblocked_mask == 0) continue;

        best_unlock_target = cand;
        best_unblocked_mask = unblocked_mask;
        break;
    }

    if (best_unlock_target) {
        if (out_target) *out_target = best_unlock_target;
        if (out_unblocked_mask) *out_unblocked_mask = best_unblocked_mask;
        if (out_kind) *out_kind = POS_DEADLOCK_PICK_READY_UNLOCK;
        return 1;
    }

    if (best_ready_reloc_target) {
        if (out_target) *out_target = best_ready_reloc_target;
        if (out_unblocked_mask) *out_unblocked_mask = fallback_wait_mask;
        if (out_kind) *out_kind = POS_DEADLOCK_PICK_READY_RELOCATE;
        return 1;
    }

    for (int i = merged_count - 1; i >= 0; i--) {
        track_node *cand = &g_track[g_pos_try_merged_candidates[i]];
        int32_t sort_dist = pos_deadlock_candidate_sort_dist(origins, cand);

        if (sort_dist < 0 || sort_dist < min_dist_mm) continue;
        if (pos_deadlock_same_physical_sensor(cand, pos->cur_sensor)) continue;
        if (pos_deadlock_same_physical_sensor(cand, current_target)) continue;
        if (pos_deadlock_candidate_in_yield_history(pos, cand)) continue;
        if (!pos_deadlock_candidate_can_force_move(pos, cand)) continue;

        if (out_target) *out_target = cand;
        if (out_unblocked_mask) *out_unblocked_mask = fallback_wait_mask;
        if (out_kind) *out_kind = POS_DEADLOCK_PICK_FORCE_MOVE;
        return 1;
    }

    return 0;
}
