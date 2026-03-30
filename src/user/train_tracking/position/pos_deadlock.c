#include "train_tracking/position_priv.h"
#include "train_tracking/traffic_manager.h"
#include "demo_manager.h"
#include <stddef.h>
#include <stdint.h>

#define DEADLOCK_TIMEOUT_TRIGGER_WINDOW_US 30000ULL

typedef enum {
    DEADLOCK_KIND_NONE = 0,
    DEADLOCK_KIND_WAIT_CYCLE = 1,
    DEADLOCK_KIND_STOPPED_BLOCKER = 2,
} deadlock_kind_t;

typedef struct {
    int count;
    int train_nums[DEADLOCK_MAX_TRAINS];
    uint8_t global_bits[DEADLOCK_MAX_TRAINS];
    uint8_t wait_mask;
    uint8_t stopped_mask;
} deadlock_participants_t;

static int deadlock_bit_count(uint8_t mask) {
    int count = 0;
    while (mask) {
        count += (mask & 1u);
        mask >>= 1;
    }
    return count;
}

static int deadlock_train_has_reserved_route(const train_pos_t *pos) {
    if (!pos) return 0;
    switch (pos->route_state) {
    case TRAIN_STATE_WAIT_SWITCH_SETTLE:
    case TRAIN_STATE_ON_ROUTE:
    case TRAIN_STATE_STOPPING:
    case TRAIN_STATE_RECOVERY_STOPPING:
    case TRAIN_STATE_STOPPING_GOTO:
    case TRAIN_STATE_FIND_POS:
        return 1;
    default:
        return 0;
    }
}

static int deadlock_train_is_manual_stop_blocker(const train_pos_t *pos) {
    if (!pos || pos->route_state != TRAIN_STATE_STOPPED) return 0;
    return !demo_is_auto_dispatching_targets();
}

static int deadlock_train_is_yield_stop_blocker(const train_pos_t *pos) {
    if (!pos || pos->route_state != TRAIN_STATE_STOPPED) return 0;
    if (!pos->deadlock_recover.valid) return 0;
    if (pos->deadlock_recover.resume_target == NULL) return 0;
    if (pos->deadlock_recover.yield_target == NULL) return 0;
    return pos->deadlock_recover.parked_at_yield != 0;
}

static int deadlock_train_is_stopped_blocker(const train_pos_t *pos) {
    return deadlock_train_is_manual_stop_blocker(pos) ||
           deadlock_train_is_yield_stop_blocker(pos);
}

static void deadlock_collect_participants(deadlock_participants_t *parts) {
    static const int train_order[6] = {13, 14, 15, 17, 18, 55};

    if (!parts) return;

    parts->count = 0;
    parts->wait_mask = 0;
    parts->stopped_mask = 0;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        parts->train_nums[i] = -1;
        parts->global_bits[i] = 0;
    }

    for (int i = 0; i < 6 && parts->count < DEADLOCK_MAX_TRAINS; i++) {
        train_pos_t *pos = pos_get(train_order[i]);
        uint8_t local_bit;

        if (!pos) continue;
        if (pos->route_state != TRAIN_STATE_WAIT_RESOURCE &&
            !deadlock_train_is_stopped_blocker(pos)) {
            continue;
        }

        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            pos->replan.blocker_mask = pos_wait_resource_current_blocker_mask(pos);
        }

        local_bit = (uint8_t)(1u << parts->count);
        parts->train_nums[parts->count] = train_order[i];
        parts->global_bits[parts->count] = pos_deadlock_train_bit(train_order[i]);
        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            parts->wait_mask |= local_bit;
        } else {
            parts->stopped_mask |= local_bit;
        }
        parts->count++;
    }
}

static int deadlock_participant_index(const deadlock_participants_t *parts,
                                      int train_num) {
    if (!parts) return -1;
    for (int i = 0; i < parts->count; i++) {
        if (parts->train_nums[i] == train_num) return i;
    }
    return -1;
}

static uint8_t deadlock_global_mask_from_local(const deadlock_participants_t *parts,
                                               uint8_t local_mask) {
    uint8_t global_mask = 0;

    if (!parts) return 0;
    for (int i = 0; i < parts->count; i++) {
        if (local_mask & (uint8_t)(1u << i)) {
            global_mask |= parts->global_bits[i];
        }
    }
    return global_mask;
}

static void deadlock_build_graph(const deadlock_participants_t *parts,
                                 uint8_t adj[DEADLOCK_MAX_TRAINS]) {
    if (!parts || !adj) return;

    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) adj[i] = 0;

    for (int i = 0; i < parts->count; i++) {
        train_pos_t *pos;

        if (!(parts->wait_mask & (uint8_t)(1u << i))) continue;

        pos = pos_get(parts->train_nums[i]);
        if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

        for (int j = 0; j < parts->count; j++) {
            if (i == j) continue;
            if (!(parts->wait_mask & (uint8_t)(1u << j))) continue;
            if (pos->replan.blocker_mask & parts->global_bits[j]) {
                adj[i] |= (uint8_t)(1u << j);
            }
        }
    }
}

static void deadlock_compute_reachability(const uint8_t adj[DEADLOCK_MAX_TRAINS],
                                          int count,
                                          uint8_t wait_mask,
                                          uint8_t reach[DEADLOCK_MAX_TRAINS]) {
    for (int i = 0; i < count; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        reach[i] = (wait_mask & bit) ? (adj[i] | bit) : 0;
    }

    for (int pass = 0; pass < count; pass++) {
        for (int i = 0; i < count; i++) {
            uint8_t expanded = reach[i];
            if (!(wait_mask & (uint8_t)(1u << i))) continue;
            for (int j = 0; j < count; j++) {
                if (expanded & (uint8_t)(1u << j)) expanded |= reach[j];
            }
            reach[i] = expanded & wait_mask;
        }
    }
}

static uint8_t deadlock_find_stopped_blocker_mask_for_train(
    int train_num, const deadlock_participants_t *parts) {
    train_pos_t *pos = pos_get(train_num);
    int start_idx = deadlock_participant_index(parts, train_num);
    uint8_t start_bit;
    uint8_t blockers = 0;

    if (!pos || !parts || start_idx < 0) return 0;
    if (pos->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    start_bit = (uint8_t)(1u << start_idx);
    for (int i = 0; i < parts->count; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        if (!(parts->stopped_mask & bit)) continue;
        if (pos->replan.blocker_mask & parts->global_bits[i]) blockers |= bit;
    }

    return blockers ? (uint8_t)(blockers | start_bit) : 0;
}

static uint8_t deadlock_find_mask_for_train(int train_num,
                                            deadlock_kind_t *out_kind,
                                            deadlock_participants_t *out_parts) {
    deadlock_participants_t parts;
    uint8_t adj[DEADLOCK_MAX_TRAINS];
    uint8_t reach[DEADLOCK_MAX_TRAINS];
    uint8_t cycle = 0;
    int start_idx;
    uint8_t stopped_blockers;

    if (out_kind) *out_kind = DEADLOCK_KIND_NONE;

    deadlock_collect_participants(&parts);
    if (out_parts) *out_parts = parts;

    start_idx = deadlock_participant_index(&parts, train_num);
    if (start_idx < 0) return 0;

    deadlock_build_graph(&parts, adj);
    if (parts.wait_mask & (uint8_t)(1u << start_idx)) {
        deadlock_compute_reachability(adj, parts.count, parts.wait_mask, reach);
        for (int i = 0; i < parts.count; i++) {
            uint8_t bit = (uint8_t)(1u << i);
            if (!(parts.wait_mask & bit)) continue;
            if ((reach[start_idx] & bit) && (reach[i] & (uint8_t)(1u << start_idx))) {
                cycle |= bit;
            }
        }

        if (deadlock_bit_count(cycle) >= 2) {
            if (out_kind) *out_kind = DEADLOCK_KIND_WAIT_CYCLE;
            return cycle;
        }
    }

    stopped_blockers = deadlock_find_stopped_blocker_mask_for_train(train_num, &parts);
    if (deadlock_bit_count(stopped_blockers) >= 2) {
        if (out_kind) *out_kind = DEADLOCK_KIND_STOPPED_BLOCKER;
        return stopped_blockers;
    }
    return 0;
}

static int deadlock_choose_victim(const deadlock_participants_t *parts,
                                  uint8_t cycle_mask,
                                  deadlock_kind_t kind) {
    if (!parts) return -1;

    if (kind == DEADLOCK_KIND_STOPPED_BLOCKER) {
        for (int i = 0; i < parts->count; i++) {
            train_pos_t *pos;
            if (!(cycle_mask & (uint8_t)(1u << i))) continue;
            pos = pos_get(parts->train_nums[i]);
            if (pos && pos->route_state == TRAIN_STATE_STOPPED) {
                return parts->train_nums[i];
            }
        }
    }

    for (int i = 0; i < parts->count; i++) {
        train_pos_t *pos;
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        pos = pos_get(parts->train_nums[i]);
        if (pos && pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            return parts->train_nums[i];
        }
    }

    for (int i = 0; i < parts->count; i++) {
        if (cycle_mask & (uint8_t)(1u << i)) return parts->train_nums[i];
    }
    return -1;
}

static int deadlock_apply_reroute(train_pos_t *victim,
                                  const deadlock_participants_t *parts,
                                  uint8_t cycle_mask,
                                  uint64_t now_us,
                                  int resume_after_yield);

static int deadlock_notice_matches_cycle(const pos_deadlock_notice_t *notice,
                                         const deadlock_participants_t *parts,
                                         uint8_t cycle_mask) {
    uint8_t notice_mask = 0;
    uint8_t cycle_global_mask;

    if (!notice || !notice->active || !notice->unresolved || !parts) return 0;

    for (int i = 0; i < notice->cycle_count && i < DEADLOCK_MAX_TRAINS; i++) {
        uint8_t bit = pos_deadlock_train_bit(notice->cycle_trains[i]);
        if (!bit) return 0;
        notice_mask |= bit;
    }
    if (deadlock_bit_count(notice_mask) < 2) return 0;

    cycle_global_mask = deadlock_global_mask_from_local(parts, cycle_mask);
    return cycle_global_mask == notice_mask;
}

static uint64_t deadlock_fallback_due_from_now(uint64_t now_us) {
    if (UINT64_MAX - now_us < DEADLOCK_FALLBACK_TIMEOUT_US) return UINT64_MAX;
    return now_us + DEADLOCK_FALLBACK_TIMEOUT_US;
}

static int deadlock_notice_timeout_due(const deadlock_participants_t *parts,
                                       uint8_t cycle_mask,
                                       uint64_t now_us) {
    pos_deadlock_notice_t notice;

    if (!parts || deadlock_bit_count(cycle_mask) < 2) return 0;

    pos_get_deadlock_notice(&notice);
    if (!deadlock_notice_matches_cycle(&notice, parts, cycle_mask)) return 0;
    if (notice.fallback_due_us == 0) return 0;
    return now_us >= notice.fallback_due_us;
}

static int deadlock_first_wait_cycle_index(const deadlock_participants_t *parts,
                                           uint8_t cycle_mask) {
    if (!parts) return -1;

    for (int i = 0; i < parts->count; i++) {
        train_pos_t *pos;
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        pos = pos_get(parts->train_nums[i]);
        if (pos && pos->route_state == TRAIN_STATE_WAIT_RESOURCE) return i;
    }
    return -1;
}

static int deadlock_wait_cycle_start_index(const deadlock_participants_t *parts,
                                           uint8_t cycle_mask) {
    pos_deadlock_notice_t notice;
    int base_idx;
    int notice_idx;

    if (!parts) return -1;

    base_idx = deadlock_first_wait_cycle_index(parts, cycle_mask);
    if (base_idx < 0) return -1;

    pos_get_deadlock_notice(&notice);
    if (!deadlock_notice_matches_cycle(&notice, parts, cycle_mask)) {
        return base_idx;
    }

    notice_idx = deadlock_participant_index(parts, notice.victim_train);
    if (notice_idx < 0 || !(cycle_mask & (uint8_t)(1u << notice_idx))) {
        return base_idx;
    }

    for (int offset = 1; offset <= parts->count; offset++) {
        int idx = (notice_idx + offset) % parts->count;
        train_pos_t *pos;

        if (!(cycle_mask & (uint8_t)(1u << idx))) continue;
        pos = pos_get(parts->train_nums[idx]);
        if (pos && pos->route_state == TRAIN_STATE_WAIT_RESOURCE) return idx;
    }

    return base_idx;
}

static void deadlock_fill_cycle_trains(pos_deadlock_notice_t *notice,
                                       const deadlock_participants_t *parts,
                                       uint8_t cycle_mask) {
    if (!notice) return;

    notice->cycle_count = 0;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        notice->cycle_trains[i] = -1;
    }
    if (!parts) return;

    for (int i = 0; i < parts->count; i++) {
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        if (notice->cycle_count >= DEADLOCK_MAX_TRAINS) break;
        notice->cycle_trains[notice->cycle_count++] = parts->train_nums[i];
    }
}

static track_node *deadlock_current_target(const train_pos_t *pos,
                                           int32_t *out_offset_mm) {
    if (out_offset_mm) *out_offset_mm = 0;
    if (!pos) return NULL;

    if (pos->orig_user_target) {
        if (out_offset_mm) *out_offset_mm = pos->orig_target_offset;
        return pos->orig_user_target;
    }
    if (pos->target_sensor) {
        if (out_offset_mm) *out_offset_mm = pos->target_offset_mm;
        return pos->target_sensor;
    }
    if (pos->pending_target) {
        if (out_offset_mm) *out_offset_mm = pos->pending_offset_mm;
        return pos->pending_target;
    }
    return NULL;
}

static int deadlock_notice_still_active(const pos_deadlock_notice_t *notice) {
    deadlock_participants_t parts;
    int first_train = -1;
    int wait_train = -1;
    uint8_t expected_mask = 0;
    uint8_t cycle_mask;
    uint8_t cycle_global_mask;

    if (!notice || !notice->active || !notice->unresolved) return 0;
    for (int i = 0; i < notice->cycle_count && i < DEADLOCK_MAX_TRAINS; i++) {
        int train_num = notice->cycle_trains[i];
        train_pos_t *pos;
        uint8_t bit = pos_deadlock_train_bit(train_num);

        if (!bit) return 0;
        if (first_train < 0) first_train = train_num;
        pos = pos_get(train_num);
        if (wait_train < 0 && pos && pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            wait_train = train_num;
        }
        expected_mask |= bit;
    }
    if ((expected_mask & pos_deadlock_train_bit(notice->victim_train)) &&
        pos_get(notice->victim_train) != NULL &&
        pos_get(notice->victim_train)->route_state == TRAIN_STATE_WAIT_RESOURCE) {
        first_train = notice->victim_train;
    } else if (wait_train >= 0) {
        first_train = wait_train;
    }
    if (first_train < 0 || deadlock_bit_count(expected_mask) < 2) return 0;

    cycle_mask = deadlock_find_mask_for_train(first_train, NULL, &parts);
    if (!cycle_mask) return 0;

    cycle_global_mask = deadlock_global_mask_from_local(&parts, cycle_mask);
    return (cycle_global_mask & expected_mask) == expected_mask;
}

void pos_deadlock_refresh_notice_state(uint64_t now_us) {
    pos_deadlock_notice_t notice;

    pos_get_deadlock_notice(&notice);
    if (!notice.active) return;

    if (notice.unresolved) {
        if (!deadlock_notice_still_active(&notice)) pos_clear_deadlock_notice();
        return;
    }

    if (notice.expire_us > 0 && now_us > notice.expire_us) {
        pos_clear_deadlock_notice();
    }
}

static void deadlock_write_notice(const deadlock_participants_t *parts,
                                  uint8_t cycle_mask,
                                  int victim_train,
                                  track_node *blocked_target,
                                  track_node *yield_target,
                                  track_node *resume_target,
                                  uint64_t detect_us,
                                  uint64_t fallback_due_us,
                                  int fallback_used,
                                  uint64_t expire_us,
                                  int unresolved) {
    pos_deadlock_notice_t notice;

    notice.active = 1;
    notice.unresolved = unresolved ? 1 : 0;
    notice.fallback_used = fallback_used ? 1 : 0;
    notice.victim_train = victim_train;
    deadlock_fill_cycle_trains(&notice, parts, cycle_mask);
    notice.blocked_target = blocked_target;
    notice.yield_target = yield_target;
    notice.resume_target = resume_target;
    notice.detect_us = detect_us;
    notice.fallback_due_us = fallback_due_us;
    notice.expire_us = expire_us;
    pos_set_deadlock_notice(&notice);
}

static void deadlock_note_detected(const deadlock_participants_t *parts,
                                   uint8_t cycle_mask,
                                   deadlock_kind_t kind,
                                   int preferred_victim_train,
                                   uint64_t now_us) {
    pos_deadlock_notice_t existing;
    int victim_train;
    uint64_t detect_us = now_us;
    uint64_t fallback_due_us = deadlock_fallback_due_from_now(now_us);

    if (!parts || deadlock_bit_count(cycle_mask) < 2) return;

    victim_train = preferred_victim_train;
    if (victim_train < 0) {
        victim_train = deadlock_choose_victim(parts, cycle_mask, kind);
    }
    if (victim_train < 0 && parts->count > 0) {
        for (int i = 0; i < parts->count; i++) {
            if (cycle_mask & (uint8_t)(1u << i)) {
                victim_train = parts->train_nums[i];
                break;
            }
        }
    }
    if (victim_train < 0) return;

    pos_get_deadlock_notice(&existing);
    if (deadlock_notice_matches_cycle(&existing, parts, cycle_mask)) {
        if (existing.detect_us > 0) detect_us = existing.detect_us;
        if (existing.fallback_due_us > 0) fallback_due_us = existing.fallback_due_us;
    }

    deadlock_write_notice(parts, cycle_mask, victim_train,
                          NULL, NULL, NULL,
                          detect_us, fallback_due_us, 0, 0, 1);
}

static int deadlock_apply_wait_cycle_reroute(
    uint8_t cycle_mask, const deadlock_participants_t *parts, uint64_t now_us,
    int *out_victim_train) {
    int start_idx;
    int last_attempted_train = -1;

    if (out_victim_train) *out_victim_train = -1;
    if (!parts) return 0;

    start_idx = deadlock_wait_cycle_start_index(parts, cycle_mask);
    if (start_idx < 0) return 0;

    for (int offset = 0; offset < parts->count; offset++) {
        int idx = (start_idx + offset) % parts->count;
        train_pos_t *pos;

        if (!(cycle_mask & (uint8_t)(1u << idx))) continue;

        pos = pos_get(parts->train_nums[idx]);
        if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

        last_attempted_train = parts->train_nums[idx];
        if (deadlock_apply_reroute(pos, parts, cycle_mask, now_us, 1)) {
            if (out_victim_train) *out_victim_train = parts->train_nums[idx];
            return 1;
        }
    }

    if (last_attempted_train >= 0) {
        deadlock_note_detected(parts, cycle_mask, DEADLOCK_KIND_WAIT_CYCLE,
                               last_attempted_train, now_us);
    }
    return 0;
}

static track_node *deadlock_actual_yield_target(const train_pos_t *pos,
                                                track_node *fallback) {
    if (!pos) return fallback;
    if (pos->midrev.active && pos->midrev.final_target != NULL) {
        return pos->midrev.final_target;
    }
    if (pos->target_sensor != NULL) return pos->target_sensor;
    return fallback;
}

static int deadlock_target_is_ready_now(train_pos_t *pos, track_node *target) {
    return pos != NULL &&
           target != NULL &&
           pos_evaluate_target_ready_now(pos, target, NULL) == POS_ROUTE_EVAL_READY;
}

static int deadlock_timeout_victim_is_eligible(const deadlock_participants_t *parts,
                                               uint8_t cycle_mask,
                                               int train_num) {
    int idx;
    train_pos_t *pos;

    if (!parts || train_num < 0) return 0;
    idx = deadlock_participant_index(parts, train_num);
    if (idx < 0 || !(cycle_mask & (uint8_t)(1u << idx))) return 0;

    pos = pos_get(train_num);
    return pos != NULL &&
           (pos->route_state == TRAIN_STATE_WAIT_RESOURCE ||
            pos->route_state == TRAIN_STATE_STOPPED);
}

static int deadlock_append_timeout_victim(int *order, int count, int cap,
                                          int train_num) {
    if (!order || count < 0 || count >= cap || train_num < 0) return count;
    for (int i = 0; i < count; i++) {
        if (order[i] == train_num) return count;
    }
    order[count++] = train_num;
    return count;
}

static int deadlock_build_timeout_victim_order(const deadlock_participants_t *parts,
                                               uint8_t cycle_mask,
                                               int preferred_train,
                                               int order[DEADLOCK_MAX_TRAINS]) {
    int count = 0;

    if (!parts || !order) return 0;

    if (deadlock_timeout_victim_is_eligible(parts, cycle_mask, preferred_train)) {
        count = deadlock_append_timeout_victim(order, count, DEADLOCK_MAX_TRAINS,
                                              preferred_train);
    }

    for (int i = 0; i < parts->count; i++) {
        train_pos_t *pos;
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        pos = pos_get(parts->train_nums[i]);
        if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        count = deadlock_append_timeout_victim(order, count, DEADLOCK_MAX_TRAINS,
                                              parts->train_nums[i]);
    }

    for (int i = 0; i < parts->count; i++) {
        train_pos_t *pos;
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        pos = pos_get(parts->train_nums[i]);
        if (!pos || pos->route_state != TRAIN_STATE_STOPPED) continue;
        count = deadlock_append_timeout_victim(order, count, DEADLOCK_MAX_TRAINS,
                                              parts->train_nums[i]);
    }

    return count;
}

static int deadlock_pick_timeout_target_for_victim(train_pos_t *victim,
                                                   track_node **out_blocked_target,
                                                   track_node **out_fallback_target,
                                                   int32_t *out_fallback_offset) {
    track_node *blocked_target;
    int32_t blocked_offset = 0;
    track_node *fallback_target = NULL;
    int32_t fallback_offset = 0;

    if (out_blocked_target) *out_blocked_target = NULL;
    if (out_fallback_target) *out_fallback_target = NULL;
    if (out_fallback_offset) *out_fallback_offset = 0;
    if (!victim) return 0;

    blocked_target = deadlock_current_target(victim, &blocked_offset);
    if (!blocked_target) return 0;

    if (blocked_target->type == NODE_SENSOR &&
        deadlock_target_is_ready_now(victim, blocked_target)) {
        fallback_target = blocked_target;
        fallback_offset = blocked_offset;
    } else if (!pos_pick_deadlock_timeout_fallback_target(victim,
                                                          &fallback_target)) {
        return 0;
    }

    if (out_blocked_target) *out_blocked_target = blocked_target;
    if (out_fallback_target) *out_fallback_target = fallback_target;
    if (out_fallback_offset) *out_fallback_offset = fallback_offset;
    return 1;
}

static int deadlock_find_notice_cycle(const pos_deadlock_notice_t *notice,
                                      deadlock_participants_t *out_parts,
                                      uint8_t *out_cycle_mask) {
    int preferred[DEADLOCK_MAX_TRAINS + 1];
    int preferred_count = 0;

    if (out_cycle_mask) *out_cycle_mask = 0;
    if (!notice || !notice->active || !notice->unresolved) return 0;

    if (notice->victim_train >= 0) {
        preferred[preferred_count++] = notice->victim_train;
    }
    for (int i = 0; i < notice->cycle_count && i < DEADLOCK_MAX_TRAINS; i++) {
        int train_num = notice->cycle_trains[i];
        int seen = 0;
        if (train_num < 0) continue;
        for (int j = 0; j < preferred_count; j++) {
            if (preferred[j] == train_num) {
                seen = 1;
                break;
            }
        }
        if (!seen) preferred[preferred_count++] = train_num;
    }

    for (int i = 0; i < preferred_count; i++) {
        deadlock_participants_t parts;
        uint8_t cycle_mask =
            deadlock_find_mask_for_train(preferred[i], NULL, &parts);
        if (!cycle_mask) continue;
        if (!deadlock_notice_matches_cycle(notice, &parts, cycle_mask)) continue;
        if (out_parts) *out_parts = parts;
        if (out_cycle_mask) *out_cycle_mask = cycle_mask;
        return 1;
    }

    return 0;
}

static int deadlock_apply_timeout_fallback(uint8_t cycle_mask,
                                           const deadlock_participants_t *parts,
                                           uint64_t now_us,
                                           int *out_victim_train) {
    pos_deadlock_notice_t notice;
    train_pos_t *victim;
    track_node *blocked_target;
    track_node *fallback_target = NULL;
    int victim_order[DEADLOCK_MAX_TRAINS];
    int victim_order_count;
    int victim_train;
    int32_t fallback_offset = 0;

    if (out_victim_train) *out_victim_train = -1;
    if (!parts) return 0;

    pos_get_deadlock_notice(&notice);
    victim_order_count = deadlock_build_timeout_victim_order(parts, cycle_mask,
                                                             notice.victim_train,
                                                             victim_order);
    if (victim_order_count <= 0) return 0;

    victim = NULL;
    blocked_target = NULL;
    victim_train = -1;
    for (int i = 0; i < victim_order_count; i++) {
        train_pos_t *candidate = pos_get(victim_order[i]);
        track_node *candidate_blocked = NULL;
        track_node *candidate_fallback = NULL;
        int32_t candidate_fallback_offset = 0;

        if (!deadlock_pick_timeout_target_for_victim(candidate,
                                                     &candidate_blocked,
                                                     &candidate_fallback,
                                                     &candidate_fallback_offset)) {
            continue;
        }

        victim = candidate;
        victim_train = victim_order[i];
        blocked_target = candidate_blocked;
        fallback_target = candidate_fallback;
        fallback_offset = candidate_fallback_offset;
        break;
    }
    if (!victim || !blocked_target || !fallback_target || victim_train < 0) return 0;

    pos_prepare_goto_request(victim, fallback_target, victim->goto_speed,
                             fallback_offset);
    if (!pos_try_direct_goto(victim)) return 0;
    if (victim->route_state != TRAIN_STATE_ON_ROUTE &&
        victim->route_state != TRAIN_STATE_WAIT_SWITCH_SETTLE) {
        return 0;
    }

    fallback_target = deadlock_actual_yield_target(victim, fallback_target);
    deadlock_write_notice(parts, cycle_mask, victim_train, blocked_target,
                          fallback_target, NULL, 0, 0, 1,
                          now_us + DEADLOCK_NOTICE_RESOLVED_US, 0);
    if (out_victim_train) *out_victim_train = victim_train;
    return 1;
}

static int deadlock_apply_reroute(train_pos_t *victim,
                                  const deadlock_participants_t *parts,
                                  uint8_t cycle_mask,
                                  uint64_t now_us,
                                  int resume_after_yield) {
    track_node *yield_target = NULL;
    track_node *blocked_target;
    track_node *resume_target;
    uint8_t global_cycle_mask;
    uint8_t unblocked_mask = 0;
    int32_t blocked_offset = 0;
    int had_resume = 0;
    pos_deadlock_pick_kind_t pick_kind = POS_DEADLOCK_PICK_NONE;

    if (!victim || !parts) return 0;

    blocked_target = deadlock_current_target(victim, &blocked_offset);
    if (!blocked_target) return 0;

    global_cycle_mask = deadlock_global_mask_from_local(parts, cycle_mask);
    had_resume = victim->deadlock_recover.valid &&
                 victim->deadlock_recover.resume_target != NULL;
    resume_target = resume_after_yield
                        ? (had_resume ? victim->deadlock_recover.resume_target
                                      : blocked_target)
                        : NULL;

    if (!pos_pick_deadlock_yield_target(victim, global_cycle_mask,
                                        &yield_target,
                                        &unblocked_mask, &pick_kind) ||
        !yield_target) {
        return 0;
    }

    if (resume_after_yield && unblocked_mask == 0) {
        unblocked_mask =
            global_cycle_mask & (uint8_t)~pos_deadlock_train_bit(victim->train_num);
    }

    if (resume_after_yield && !victim->deadlock_recover.valid) {
        victim->deadlock_recover.valid = 1;
        victim->deadlock_recover.resume_target = blocked_target;
        victim->deadlock_recover.resume_offset_mm = blocked_offset;
    }
    if (resume_after_yield) {
        victim->deadlock_recover.yield_target = yield_target;
        victim->deadlock_recover.wait_start_mask = unblocked_mask;
        victim->deadlock_recover.parked_at_yield = 0;
        victim->deadlock_recover.parked_since_us = 0;
    } else {
        pos_clear_deadlock_recover(victim);
    }

    victim->pending_target = yield_target;
    victim->pending_offset_mm = 0;
    victim->orig_user_target = yield_target;
    victim->orig_target_offset = 0;
    victim->target_sensor = yield_target;
    victim->target_offset_mm = 0;
    victim->dist_to_target_mm = 0;
    victim->parked_target_col = POS_TARGET_COL_NONE;
    pos_clear_committed_route(victim);

    if (!pos_try_direct_goto_strict(victim)) return 0;
    if (victim->route_state != TRAIN_STATE_ON_ROUTE &&
        victim->route_state != TRAIN_STATE_WAIT_SWITCH_SETTLE) {
        return 0;
    }

    yield_target = deadlock_actual_yield_target(victim, yield_target);
    if (resume_after_yield) {
        victim->deadlock_recover.yield_target = yield_target;
    }

    deadlock_write_notice(parts, cycle_mask, victim->train_num, blocked_target,
                          yield_target, resume_after_yield ? resume_target : NULL,
                          0, 0, 0,
                          now_us + DEADLOCK_NOTICE_RESOLVED_US, 0);
    return 1;
}

static int deadlock_apply_stopped_blocker_reroute(
    uint8_t cycle_mask, const deadlock_participants_t *parts, uint64_t now_us,
    int *out_victim_train) {
    if (out_victim_train) *out_victim_train = -1;
    if (!parts) return 0;

    for (int i = 0; i < parts->count; i++) {
        train_pos_t *pos;
        int keep_resume_target;

        if (!(cycle_mask & (uint8_t)(1u << i))) continue;

        pos = pos_get(parts->train_nums[i]);
        if (!pos || pos->route_state != TRAIN_STATE_STOPPED) continue;

        keep_resume_target =
            pos->deadlock_recover.valid &&
            pos->deadlock_recover.resume_target != NULL;
        if (deadlock_apply_reroute(pos, parts, cycle_mask, now_us,
                                   keep_resume_target)) {
            if (out_victim_train) *out_victim_train = parts->train_nums[i];
            return 1;
        }
    }

    return 0;
}

static int deadlock_resume_waiting_trains_reserved(uint8_t wait_start_mask) {
    int any_waiting = 0;

    for (int i = 0; i < 6; i++) {
        train_pos_t *other;
        uint8_t bit = (uint8_t)(1u << i);
        if (!(wait_start_mask & bit)) continue;

        other = pos_get(pos_deadlock_index_to_train(i));
        if (!other) continue;
        if (deadlock_train_has_reserved_route(other)) return 1;
        if (other->route_state == TRAIN_STATE_WAIT_RESOURCE) any_waiting = 1;
    }

    return !any_waiting;
}

int pos_deadlock_maybe_resume_after_yield(train_pos_t *pos, uint64_t now_us) {
    track_node *resume_target;
    int32_t resume_offset_mm;

    if (!pos || pos->route_state != TRAIN_STATE_STOPPED) return 0;
    if (!pos->deadlock_recover.valid || pos->deadlock_recover.resume_target == NULL) return 0;
    if (pos->deadlock_recover.yield_target == NULL) return 0;
    if (!pos->deadlock_recover.parked_at_yield) return 0;

    /* Demo gold should continue with a fresh auto-picked destination instead
     * of resuming the pre-deadlock target after yielding. */
    if (demo_is_auto_dispatching_targets()) {
        pos_clear_deadlock_recover(pos);
        return 0;
    }

    if (pos->deadlock_recover.parked_since_us == 0 ||
        now_us - pos->deadlock_recover.parked_since_us < DEADLOCK_RESUME_DELAY_US) {
        return 0;
    }

    if (pos->deadlock_recover.wait_start_mask != 0 &&
        !deadlock_resume_waiting_trains_reserved(pos->deadlock_recover.wait_start_mask)) {
        return 0;
    }

    resume_target = pos->deadlock_recover.resume_target;
    resume_offset_mm = pos->deadlock_recover.resume_offset_mm;
    pos_clear_deadlock_recover(pos);
    pos->pending_target = resume_target;
    pos->pending_offset_mm = resume_offset_mm;
    pos->orig_user_target = resume_target;
    pos->orig_target_offset = resume_offset_mm;
    pos->target_sensor = resume_target;
    pos->target_offset_mm = resume_offset_mm;
    pos->dist_to_target_mm = 0;
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos->replan.blocker_mask = 0;
    pos->replan.next_us = 0;
    pos_clear_committed_route(pos);
    return pos_try_direct_goto(pos);
}

void pos_deadlock_on_tick(uint64_t now_us) {
    pos_deadlock_notice_t notice;
    deadlock_participants_t parts;
    uint8_t cycle_mask = 0;

    pos_get_deadlock_notice(&notice);
    if (!notice.active || !notice.unresolved) return;
    if (notice.fallback_due_us == 0 || now_us < notice.fallback_due_us) return;
    if (now_us - notice.fallback_due_us >= DEADLOCK_TIMEOUT_TRIGGER_WINDOW_US) return;
    if (!deadlock_find_notice_cycle(&notice, &parts, &cycle_mask)) return;

    (void)deadlock_apply_timeout_fallback(cycle_mask, &parts, now_us, NULL);
}

int pos_deadlock_maybe_reroute_waiter(train_pos_t *pos, uint64_t now_us) {
    deadlock_participants_t parts;
    uint8_t cycle_mask = 0;
    int victim_train = -1;
    deadlock_kind_t deadlock_kind = DEADLOCK_KIND_NONE;

    if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    cycle_mask = deadlock_find_mask_for_train(pos->train_num, &deadlock_kind, &parts);
    if (!cycle_mask) return 0;

    if (deadlock_notice_timeout_due(&parts, cycle_mask, now_us)) {
        return deadlock_apply_timeout_fallback(cycle_mask, &parts, now_us,
                                               &victim_train);
    }

    if (deadlock_kind == DEADLOCK_KIND_STOPPED_BLOCKER) {
        deadlock_note_detected(&parts, cycle_mask, deadlock_kind, -1, now_us);
        return deadlock_apply_stopped_blocker_reroute(cycle_mask, &parts, now_us,
                                                      &victim_train);
    }

    return deadlock_apply_wait_cycle_reroute(cycle_mask, &parts, now_us,
                                             &victim_train);
}
