#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include <stddef.h>
#include <stdint.h>

#define DEADLOCK_YIELD_MAX_CANDIDATES 8

typedef struct {
    track_node *target;
    int32_t     total_dist_mm;
} pos_deadlock_yield_candidate_t;

static pos_deadlock_yield_candidate_t
    g_pos_try_deadlock_candidates[DEADLOCK_YIELD_MAX_CANDIDATES];
static pos_route_eval_t g_pos_try_eval_candidate;
static int g_pos_try_snapshot[TRACK_MAX];

static uint8_t pos_simulate_deadlock_unblocked_mask(train_pos_t *victim,
                                                    uint8_t cycle_mask,
                                                    track_node *yield_target) {
    uint8_t ready_mask = 0;
    track_node *keep_end;

    if (!victim || !yield_target) return 0;

    keep_end = pos_release_keep_end(yield_target, NULL);
    traffic_snapshot_reservations(g_pos_try_snapshot);
    traffic_simulate_parked_train(victim->train_num, yield_target, keep_end);

    for (int i = 0; i < 6; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        train_pos_t *other;
        track_node *target;

        if (!(cycle_mask & bit)) continue;
        if (pos_deadlock_index_to_train(i) == victim->train_num) continue;

        other = pos_get(pos_deadlock_index_to_train(i));
        if (!other || other->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

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

static int pos_deadlock_is_canonical_sensor(track_node *cand) {
    if (!cand || cand->type != NODE_SENSOR) return 0;
    if (!cand->reverse || cand->reverse->type != NODE_SENSOR) return 1;
    return cand < cand->reverse;
}

static int pos_deadlock_insert_yield_candidate(track_node *target,
                                               int32_t total_dist_mm) {
    pos_deadlock_yield_candidate_t entry;
    int count = 0;
    int insert_at = DEADLOCK_YIELD_MAX_CANDIDATES;

    if (!target) return 0;

    entry.target = target;
    entry.total_dist_mm = total_dist_mm;

    while (count < DEADLOCK_YIELD_MAX_CANDIDATES &&
           g_pos_try_deadlock_candidates[count].target != NULL) {
        if (insert_at == DEADLOCK_YIELD_MAX_CANDIDATES &&
            total_dist_mm < g_pos_try_deadlock_candidates[count].total_dist_mm) {
            insert_at = count;
        }
        count++;
    }

    if (insert_at == DEADLOCK_YIELD_MAX_CANDIDATES) {
        if (count >= DEADLOCK_YIELD_MAX_CANDIDATES) return count;
        g_pos_try_deadlock_candidates[count] = entry;
        return count + 1;
    }

    if (count < DEADLOCK_YIELD_MAX_CANDIDATES) count++;
    for (int i = count - 1; i > insert_at; i--) {
        g_pos_try_deadlock_candidates[i] = g_pos_try_deadlock_candidates[i - 1];
    }
    g_pos_try_deadlock_candidates[insert_at] = entry;
    return count;
}

int pos_pick_deadlock_yield_target(train_pos_t *pos, uint8_t cycle_mask,
                                   track_node **out_target,
                                   uint8_t *out_unblocked_mask) {
    track_node *current_target;
    track_node *best_target = NULL;
    track_node *fallback_target = NULL;
    uint8_t best_unblocked_mask = 0;
    uint8_t fallback_wait_mask;
    int32_t best_dist = 0;
    int32_t fallback_dist = 0;
    int candidate_count = 0;

    if (out_target) *out_target = NULL;
    if (out_unblocked_mask) *out_unblocked_mask = 0;
    if (!pos || !pos->cur_sensor) return 0;

    current_target = pos_route_current_goal(pos);
    fallback_wait_mask = cycle_mask & (uint8_t)~pos_deadlock_train_bit(pos->train_num);
    for (int i = 0; i < DEADLOCK_YIELD_MAX_CANDIDATES; i++) {
        g_pos_try_deadlock_candidates[i].target = NULL;
        g_pos_try_deadlock_candidates[i].total_dist_mm = 0;
    }

    /* Keep only the nearest ready-now park targets; simulating every sensor
     * against the whole wait cycle is too expensive on the hot path. */
    for (int i = 0; i < TRACK_MAX; i++) {
        int32_t cand_total_dist;
        track_node *cand = &g_track[i];

        if (!pos_deadlock_is_canonical_sensor(cand)) continue;
        if (cand == pos->cur_sensor) continue;
        if (cand == current_target) continue;
        if (pos_evaluate_target_ready_now(pos, cand,
                                          &g_pos_try_eval_candidate) != POS_ROUTE_EVAL_READY) {
            continue;
        }

        cand_total_dist = g_pos_try_eval_candidate.plan.total_dist_mm;
        if (fallback_target == NULL || cand_total_dist < fallback_dist) {
            fallback_target = cand;
            fallback_dist = cand_total_dist;
        }
        candidate_count = pos_deadlock_insert_yield_candidate(cand, cand_total_dist);
    }

    for (int i = 0; i < candidate_count; i++) {
        uint8_t unblocked_mask;
        track_node *cand = g_pos_try_deadlock_candidates[i].target;
        int32_t cand_total_dist = g_pos_try_deadlock_candidates[i].total_dist_mm;

        if (!cand) continue;
        unblocked_mask = pos_simulate_deadlock_unblocked_mask(pos, cycle_mask, cand);
        if (unblocked_mask == 0) continue;
        if (best_target == NULL ||
            pos_route_blocker_mask_bit_count(unblocked_mask) >
                pos_route_blocker_mask_bit_count(best_unblocked_mask) ||
            (pos_route_blocker_mask_bit_count(unblocked_mask) ==
                 pos_route_blocker_mask_bit_count(best_unblocked_mask) &&
             cand_total_dist < best_dist)) {
            best_target = cand;
            best_unblocked_mask = unblocked_mask;
            best_dist = cand_total_dist;
        }
    }

    if (best_target) {
        if (out_target) *out_target = best_target;
        if (out_unblocked_mask) *out_unblocked_mask = best_unblocked_mask;
        return 1;
    }

    /* No immediate unlock found: still move to the nearest viable park target
     * so the reservation graph changes and later retries can progress. */
    if (!fallback_target) return 0;
    if (out_target) *out_target = fallback_target;
    if (out_unblocked_mask) *out_unblocked_mask = fallback_wait_mask;
    return 1;
}
