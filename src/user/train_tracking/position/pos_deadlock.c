#include "train_tracking/position_priv.h"
#include "train_tracking/traffic_manager.h"
#include "demo_manager.h"
#include "kassert.h"
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DEADLOCK_KIND_NONE = 0,
    DEADLOCK_KIND_WAIT_CYCLE = 1,
    DEADLOCK_KIND_STOPPED_BLOCKER = 2,
} deadlock_kind_t;

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

static void deadlock_build_graph(uint8_t adj[6], uint8_t *out_wait_mask,
                                 uint8_t *out_stopped_mask) {
    uint8_t wait_mask = 0;
    uint8_t stopped_mask = 0;

    for (int i = 0; i < 6; i++) adj[i] = 0;

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        int idx;
        uint8_t bit;
        if (pos->train_num < 0) continue;
        idx = pos_deadlock_train_to_index(pos->train_num);
        if (idx < 0) continue;
        bit = (uint8_t)(1u << idx);
        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            wait_mask |= bit;
        } else if (deadlock_train_is_stopped_blocker(pos)) {
            stopped_mask |= bit;
        }
    }

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        int idx;
        if (pos->train_num < 0) continue;
        if (pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        idx = pos_deadlock_train_to_index(pos->train_num);
        if (idx < 0) continue;
        adj[idx] = pos->replan.blocker_mask & wait_mask & (uint8_t)~(1u << idx);
    }

    if (out_wait_mask) *out_wait_mask = wait_mask;
    if (out_stopped_mask) *out_stopped_mask = stopped_mask;
}

static void deadlock_compute_reachability(const uint8_t adj[6], uint8_t wait_mask,
                                          uint8_t reach[6]) {
    for (int i = 0; i < 6; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        reach[i] = (wait_mask & bit) ? (adj[i] | bit) : 0;
    }

    for (int pass = 0; pass < 6; pass++) {
        for (int i = 0; i < 6; i++) {
            uint8_t expanded = reach[i];
            if (!(wait_mask & (uint8_t)(1u << i))) continue;
            for (int j = 0; j < 6; j++) {
                if (expanded & (uint8_t)(1u << j)) expanded |= reach[j];
            }
            reach[i] = expanded & wait_mask;
        }
    }
}

static uint8_t deadlock_find_stopped_blocker_mask_for_train(int train_num,
                                                            uint8_t stopped_mask) {
    train_pos_t *pos = pos_get(train_num);
    int start_idx = pos_deadlock_train_to_index(train_num);
    uint8_t start_bit;
    uint8_t blockers;

    if (!pos || start_idx < 0) return 0;
    if (pos->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    start_bit = (uint8_t)(1u << start_idx);
    blockers = pos->replan.blocker_mask & stopped_mask & (uint8_t)~start_bit;
    return blockers ? (uint8_t)(blockers | start_bit) : 0;
}

static uint8_t deadlock_find_mask_for_train(int train_num, deadlock_kind_t *out_kind) {
    uint8_t adj[6];
    uint8_t reach[6];
    uint8_t wait_mask = 0;
    uint8_t stopped_mask = 0;
    uint8_t cycle = 0;
    int start_idx = pos_deadlock_train_to_index(train_num);
    uint8_t stopped_blockers;

    if (out_kind) *out_kind = DEADLOCK_KIND_NONE;
    if (start_idx < 0) return 0;

    deadlock_build_graph(adj, &wait_mask, &stopped_mask);
    if (wait_mask & (uint8_t)(1u << start_idx)) {
        deadlock_compute_reachability(adj, wait_mask, reach);
        for (int i = 0; i < 6; i++) {
            uint8_t bit = (uint8_t)(1u << i);
            if (!(wait_mask & bit)) continue;
            if ((reach[start_idx] & bit) && (reach[i] & (uint8_t)(1u << start_idx))) {
                cycle |= bit;
            }
        }

        if (deadlock_bit_count(cycle) >= 2) {
            if (out_kind) *out_kind = DEADLOCK_KIND_WAIT_CYCLE;
            return cycle;
        }
    }

    stopped_blockers = deadlock_find_stopped_blocker_mask_for_train(train_num, stopped_mask);
    if (deadlock_bit_count(stopped_blockers) >= 2) {
        if (out_kind) *out_kind = DEADLOCK_KIND_STOPPED_BLOCKER;
        return stopped_blockers;
    }
    return 0;
}

static int deadlock_choose_victim(uint8_t cycle_mask, deadlock_kind_t kind) {
    if (kind == DEADLOCK_KIND_STOPPED_BLOCKER) {
        for (int i = 0; i < 6; i++) {
            int train_num;
            train_pos_t *pos;
            if (!(cycle_mask & (uint8_t)(1u << i))) continue;
            train_num = pos_deadlock_index_to_train(i);
            pos = pos_get(train_num);
            if (pos && pos->route_state == TRAIN_STATE_STOPPED) return train_num;
        }
    }

    for (int i = 0; i < 6; i++) {
        int train_num;
        train_pos_t *pos;
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        train_num = pos_deadlock_index_to_train(i);
        pos = pos_get(train_num);
        if (pos && pos->route_state == TRAIN_STATE_WAIT_RESOURCE) return train_num;
    }
    for (int i = 0; i < 6; i++) {
        if (cycle_mask & (uint8_t)(1u << i)) {
            return pos_deadlock_index_to_train(i);
        }
    }
    return -1;
}

static void deadlock_fill_cycle_trains(pos_deadlock_notice_t *notice,
                                       uint8_t cycle_mask) {
    if (!notice) return;
    notice->cycle_count = 0;
    for (int i = 0; i < 6; i++) notice->cycle_trains[i] = -1;
    for (int i = 0; i < 6; i++) {
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        notice->cycle_trains[notice->cycle_count++] = pos_deadlock_index_to_train(i);
    }
}

static track_node *deadlock_current_target(const train_pos_t *pos, int32_t *out_offset_mm) {
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
    int first_train = -1;
    int wait_train = -1;
    uint8_t expected_mask = 0;
    uint8_t cycle_mask;

    if (!notice || !notice->active || !notice->unresolved) return 0;
    for (int i = 0; i < notice->cycle_count && i < 6; i++) {
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

    cycle_mask = deadlock_find_mask_for_train(first_train, NULL);
    return (cycle_mask & expected_mask) == expected_mask;
}

void pos_deadlock_refresh_notice_state(void) {
    pos_deadlock_notice_t notice;
    pos_get_deadlock_notice(&notice);
    if (!notice.active || !notice.unresolved) return;
    if (!deadlock_notice_still_active(&notice)) pos_clear_deadlock_notice();
}

static void deadlock_write_notice(uint8_t cycle_mask, int victim_train,
                                  track_node *blocked_target,
                                  track_node *yield_target,
                                  track_node *resume_target,
                                  uint64_t expire_us,
                                  int unresolved) {
    pos_deadlock_notice_t notice;
    notice.active = 1;
    notice.unresolved = unresolved ? 1 : 0;
    notice.victim_train = victim_train;
    deadlock_fill_cycle_trains(&notice, cycle_mask);
    notice.blocked_target = blocked_target;
    notice.yield_target = yield_target;
    notice.resume_target = resume_target;
    notice.expire_us = expire_us;
    pos_set_deadlock_notice(&notice);
}

static void deadlock_write_detected_notice(uint8_t cycle_mask, int victim_train) {
    train_pos_t *victim;
    track_node *blocked_target = NULL;
    track_node *resume_target = NULL;

    if (victim_train < 0) return;

    victim = pos_get(victim_train);
    if (victim) {
        blocked_target = deadlock_current_target(victim, NULL);
        resume_target = (victim->deadlock_recover.valid &&
                         victim->deadlock_recover.resume_target != NULL)
                        ? victim->deadlock_recover.resume_target
                        : blocked_target;
    }

    deadlock_write_notice(cycle_mask, victim_train, blocked_target, NULL,
                          resume_target, 0, 1);
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

static int deadlock_apply_reroute(train_pos_t *victim, uint8_t cycle_mask,
                                  uint64_t now_us, int resume_after_yield) {
    track_node *yield_target = NULL;
    track_node *blocked_target;
    track_node *resume_target;
    uint8_t unblocked_mask = 0;
    int32_t blocked_offset = 0;
    int had_resume = 0;

    if (!victim) return 0;

    blocked_target = deadlock_current_target(victim, &blocked_offset);
    if (!blocked_target) {
        deadlock_write_notice(cycle_mask, victim->train_num, NULL, NULL, NULL, 0, 1);
        return 0;
    }

    had_resume = victim->deadlock_recover.valid && victim->deadlock_recover.resume_target != NULL;
    resume_target = resume_after_yield
                    ? (had_resume ? victim->deadlock_recover.resume_target : blocked_target)
                    : NULL;

    if (!pos_pick_deadlock_yield_target(victim, cycle_mask, &yield_target,
                                        &unblocked_mask) ||
        !yield_target) {
        deadlock_write_notice(cycle_mask, victim->train_num, blocked_target, NULL,
                              resume_target, 0, 1);
        return 0;
    }

    if (resume_after_yield && unblocked_mask == 0) {
        unblocked_mask = cycle_mask & (uint8_t)~pos_deadlock_train_bit(victim->train_num);
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
    pos_clear_committed_route(victim);

    if (!pos_try_direct_goto_strict(victim) ||
        (victim->route_state != TRAIN_STATE_ON_ROUTE &&
         victim->route_state != TRAIN_STATE_WAIT_SWITCH_SETTLE)) {
        deadlock_write_notice(cycle_mask, victim->train_num, blocked_target, NULL,
                              resume_after_yield ? victim->deadlock_recover.resume_target : NULL,
                              0, 1);
        return 0;
    }

    yield_target = deadlock_actual_yield_target(victim, yield_target);
    if (resume_after_yield) {
        victim->deadlock_recover.yield_target = yield_target;
    }

    deadlock_write_notice(cycle_mask, victim->train_num, blocked_target, yield_target,
                          resume_after_yield ? victim->deadlock_recover.resume_target : NULL,
                          now_us + DEADLOCK_NOTICE_RESOLVED_US, 0);
    return 1;
}

static int deadlock_apply_stopped_blocker_reroute(uint8_t cycle_mask,
                                                  uint64_t now_us,
                                                  int *out_victim_train) {
    int fallback_victim = -1;

    if (out_victim_train) *out_victim_train = -1;

    for (int i = 0; i < 6; i++) {
        int train_num;
        train_pos_t *pos;
        int keep_resume_target;

        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        train_num = pos_deadlock_index_to_train(i);
        pos = pos_get(train_num);
        if (!pos || pos->route_state != TRAIN_STATE_STOPPED) continue;

        if (fallback_victim < 0) fallback_victim = train_num;
        keep_resume_target =
            pos->deadlock_recover.valid &&
            pos->deadlock_recover.resume_target != NULL;
        if (deadlock_apply_reroute(pos, cycle_mask, now_us, keep_resume_target)) {
            if (out_victim_train) *out_victim_train = train_num;
            return 1;
        }
    }

    if (out_victim_train) *out_victim_train = fallback_victim;
    if (fallback_victim >= 0) {
        deadlock_write_detected_notice(cycle_mask, fallback_victim);
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
        /* Once a blocked peer has reserved a route, it can make progress;
         * the yielded train does not need to wait for physical movement. */
        if (deadlock_train_has_reserved_route(other)) return 1;
        if (other->route_state == TRAIN_STATE_WAIT_RESOURCE) any_waiting = 1;
    }

    return !any_waiting;
}

int pos_deadlock_maybe_resume_after_yield(train_pos_t *pos) {
    track_node *resume_target;
    int32_t resume_offset_mm;

    if (!pos || pos->route_state != TRAIN_STATE_STOPPED) return 0;
    if (!pos->deadlock_recover.valid || pos->deadlock_recover.resume_target == NULL) return 0;
    if (pos->deadlock_recover.yield_target == NULL) return 0;
    if (!pos->deadlock_recover.parked_at_yield) return 0;
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
    pos->replan.blocker_mask = 0;
    pos->replan.next_us = 0;
    pos_clear_committed_route(pos);
    KASSERT(pos_try_direct_goto(pos));
    return 1;
}

int pos_deadlock_maybe_reroute_waiter(train_pos_t *pos, uint64_t now_us) {
    uint8_t cycle_mask = 0;
    int victim_train = -1;
    deadlock_kind_t deadlock_kind = DEADLOCK_KIND_NONE;

    if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    cycle_mask = deadlock_find_mask_for_train(pos->train_num, &deadlock_kind);
    if (cycle_mask) {
        if (deadlock_kind == DEADLOCK_KIND_STOPPED_BLOCKER) {
            (void)deadlock_apply_stopped_blocker_reroute(cycle_mask, now_us,
                                                         &victim_train);
            return 1;
        }

        victim_train = deadlock_choose_victim(cycle_mask, deadlock_kind);
        deadlock_write_detected_notice(cycle_mask, victim_train);
        if (victim_train != pos->train_num) return 1;
        (void)deadlock_apply_reroute(pos, cycle_mask, now_us, 1);
        return 1;
    }
    return 0;
}
