#include "train_tracking/position_priv.h"
#include "train_tracking/deadlock.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "timer.h"
#include "demo_manager.h"
#include <stddef.h>
#include <stdint.h>

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

static int deadlock_node_index(track_node *node) {
    int idx;

    if (!node) return -1;
    idx = (int)(node - g_track);
    return (idx >= 0 && idx < TRACK_MAX) ? idx : -1;
}

static track_node *deadlock_node_from_index(int idx) {
    if (idx < 0 || idx >= TRACK_MAX) return NULL;
    return &g_track[idx];
}

static int deadlock_same_physical_sensor(track_node *a, track_node *b) {
    if (!a || !b) return 0;
    return a == b || a->reverse == b || b->reverse == a;
}

static void deadlock_record_yield_history(train_pos_t *pos, track_node *yield_target) {
    pos_deadlock_recover_t *recover;

    if (!pos || !yield_target) return;
    recover = &pos->deadlock_recover;

    for (int i = 0; i < recover->yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        if (deadlock_same_physical_sensor(recover->yield_history[i], yield_target)) {
            return;
        }
    }

    if (recover->yield_history_count < DEADLOCK_YIELD_HISTORY_MAX) {
        recover->yield_history[recover->yield_history_count++] = yield_target;
        return;
    }

    for (int i = 1; i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        recover->yield_history[i - 1] = recover->yield_history[i];
    }
    recover->yield_history[DEADLOCK_YIELD_HISTORY_MAX - 1] = yield_target;
}

static void deadlock_route_plan_from_serialized(route_plan_t *out,
                                                const deadlock_route_plan_t *plan) {
    if (!out) return;
    *out = (route_plan_t){0};
    if (!plan) return;

    out->sw_count = plan->sw_count;
    for (int i = 0; i < plan->sw_count; i++) {
        out->sw_nums[i] = plan->sw_nums[i];
        out->sw_dirs[i] = plan->sw_dirs[i];
    }

    out->total_dist_mm = plan->total_dist_mm;
    out->chosen_target = deadlock_node_from_index(plan->chosen_target_idx);
    out->has_reversal = plan->has_reversal;
    out->reversal_sensor = deadlock_node_from_index(plan->reversal_sensor_idx);
    out->dist_to_reversal_mm = plan->dist_to_reversal_mm;
    out->sw_count2 = plan->sw_count2;
    for (int i = 0; i < plan->sw_count2; i++) {
        out->sw_nums2[i] = plan->sw_nums2[i];
        out->sw_dirs2[i] = plan->sw_dirs2[i];
    }
    out->dist_after_reversal_mm = plan->dist_after_reversal_mm;
    out->path_count = plan->path_count;
    for (int i = 0; i < plan->path_count; i++) {
        out->path_nodes[i] = plan->path_nodes[i];
    }
    out->path_count2 = plan->path_count2;
    for (int i = 0; i < plan->path_count2; i++) {
        out->path_nodes2[i] = plan->path_nodes2[i];
    }
}

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
                                  uint64_t expire_us,
                                  int unresolved) {
    pos_deadlock_notice_t notice;

    notice.active = 1;
    notice.unresolved = unresolved ? 1 : 0;
    notice.victim_train = victim_train;
    deadlock_fill_cycle_trains(&notice, parts, cycle_mask);
    notice.blocked_target = blocked_target;
    notice.yield_target = yield_target;
    notice.resume_target = resume_target;
    notice.expire_us = expire_us;
    pos_set_deadlock_notice(&notice);
}

static void deadlock_note_detected(const deadlock_participants_t *parts,
                                   uint8_t cycle_mask,
                                   deadlock_kind_t kind) {
    int victim_train;

    if (!parts || deadlock_bit_count(cycle_mask) < 2) return;

    victim_train = deadlock_choose_victim(parts, cycle_mask, kind);
    if (victim_train < 0 && parts->count > 0) {
        for (int i = 0; i < parts->count; i++) {
            if (cycle_mask & (uint8_t)(1u << i)) {
                victim_train = parts->train_nums[i];
                break;
            }
        }
    }
    if (victim_train < 0) return;

    deadlock_write_notice(parts, cycle_mask, victim_train,
                          NULL, NULL, NULL, 0, 1);
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

    if (!pos_pick_deadlock_yield_target(victim, global_cycle_mask, &yield_target,
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

    if (pick_kind == POS_DEADLOCK_PICK_FORCE_MOVE) {
        if (!pos_try_direct_goto(victim)) return 0;
    } else {
        if (!pos_try_direct_goto_strict(victim)) return 0;
    }
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

static void deadlock_fill_participant_snapshot(
    deadlock_participant_snapshot_t *out, const train_pos_t *pos) {
    track_node *origins[2] = {0};
    track_node *goal;
    int32_t goal_offset = 0;

    if (!out) return;
    *out = (deadlock_participant_snapshot_t){0};
    out->train_num = -1;
    out->cur_sensor_idx = -1;
    out->origin0_idx = -1;
    out->origin1_idx = -1;
    out->goal_idx = -1;
    out->resume_target_idx = -1;
    out->yield_target_idx = -1;
    for (int i = 0; i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        out->yield_history_idx[i] = -1;
    }

    if (!pos) return;

    pos_route_fill_origins(pos, origins);
    goal = deadlock_current_target(pos, &goal_offset);

    out->train_num = pos->train_num;
    out->train_ind = pos->train_ind;
    out->route_state = pos->route_state;
    out->goto_speed = pos->goto_speed;
    out->cur_sensor_idx = deadlock_node_index(pos->cur_sensor);
    out->origin0_idx = deadlock_node_index(origins[0]);
    out->origin1_idx = deadlock_node_index(origins[1]);
    out->goal_idx = deadlock_node_index(goal);
    out->goal_offset_mm = goal_offset;
    out->blocker_mask = pos->replan.blocker_mask;
    out->resume_target_idx = deadlock_node_index(pos->deadlock_recover.resume_target);
    out->resume_offset_mm = pos->deadlock_recover.resume_offset_mm;
    out->yield_target_idx = deadlock_node_index(pos->deadlock_recover.yield_target);
    out->parked_at_yield = pos->deadlock_recover.parked_at_yield;
    out->wait_start_mask = pos->deadlock_recover.wait_start_mask;
    out->yield_history_count = pos->deadlock_recover.yield_history_count;
    for (int i = 0; i < pos->deadlock_recover.yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        out->yield_history_idx[i] =
            deadlock_node_index(pos->deadlock_recover.yield_history[i]);
    }
}

void pos_deadlock_build_snapshot(deadlock_snapshot_t *out, uint64_t now_us) {
    deadlock_participants_t parts;
    const switch_entry_t *switch_state = track_get_switch_state();

    if (!out) return;

    *out = (deadlock_snapshot_t){0};
    out->now_us = now_us;
    out->traffic_generation = traffic_get_change_generation();
    out->switch_generation = track_get_switch_generation();
    out->auto_dispatching_targets = demo_is_auto_dispatching_targets() ? 1 : 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        out->reservation_owner[i] = -1;
    }
    for (int i = 0; i < MAX_SWITCHES; i++) {
        out->switch_state[i] = switch_state[i].state;
    }
    traffic_snapshot_reservations(out->reservation_owner);

    deadlock_collect_participants(&parts);
    out->participant_count = parts.count;
    for (int i = 0; i < parts.count; i++) {
        deadlock_fill_participant_snapshot(&out->participants[i],
                                           pos_get(parts.train_nums[i]));
    }
}

static int deadlock_participant_snapshot_matches(
    const deadlock_participant_snapshot_t *a,
    const deadlock_participant_snapshot_t *b) {
    if (!a || !b) return 0;
    return a->train_num == b->train_num &&
           a->train_ind == b->train_ind &&
           a->route_state == b->route_state &&
           a->goto_speed == b->goto_speed &&
           a->cur_sensor_idx == b->cur_sensor_idx &&
           a->origin0_idx == b->origin0_idx &&
           a->origin1_idx == b->origin1_idx &&
           a->goal_idx == b->goal_idx &&
           a->goal_offset_mm == b->goal_offset_mm &&
           a->blocker_mask == b->blocker_mask &&
           a->resume_target_idx == b->resume_target_idx &&
           a->resume_offset_mm == b->resume_offset_mm &&
           a->yield_target_idx == b->yield_target_idx &&
           a->parked_at_yield == b->parked_at_yield &&
           a->wait_start_mask == b->wait_start_mask &&
           a->yield_history_count == b->yield_history_count;
}

static int deadlock_participant_snapshot_history_matches(
    const deadlock_participant_snapshot_t *a,
    const deadlock_participant_snapshot_t *b) {
    if (!a || !b) return 0;
    for (int i = 0; i < a->yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        if (a->yield_history_idx[i] != b->yield_history_idx[i]) return 0;
    }
    return 1;
}

static int deadlock_result_snapshot_is_current(const deadlock_result_t *result) {
    deadlock_snapshot_t current;

    if (!result) return 0;

    pos_deadlock_build_snapshot(&current, read_timer());
    if (current.traffic_generation != result->traffic_generation) return 0;
    if (current.switch_generation != result->switch_generation) return 0;
    if (current.auto_dispatching_targets != result->auto_dispatching_targets) return 0;
    if (current.participant_count != result->participant_count) return 0;

    for (int i = 0; i < result->participant_count; i++) {
        if (!deadlock_participant_snapshot_matches(&current.participants[i],
                                                   &result->participants[i]) ||
            !deadlock_participant_snapshot_history_matches(&current.participants[i],
                                                           &result->participants[i])) {
            return 0;
        }
    }
    return 1;
}

static void deadlock_write_notice_from_result(const deadlock_result_t *result,
                                              int unresolved,
                                              track_node *yield_target_override) {
    pos_deadlock_notice_t notice;

    if (!result) return;

    notice = (pos_deadlock_notice_t){0};
    notice.active = 1;
    notice.unresolved = unresolved ? 1 : 0;
    notice.victim_train = result->victim_train;
    notice.cycle_count = 0;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        notice.cycle_trains[i] = -1;
    }
    for (int i = 0; i < result->cycle_count && i < DEADLOCK_MAX_TRAINS; i++) {
        notice.cycle_trains[notice.cycle_count++] = result->cycle_trains[i];
    }
    notice.blocked_target = deadlock_node_from_index(result->blocked_target_idx);
    notice.yield_target = yield_target_override
                              ? yield_target_override
                              : deadlock_node_from_index(result->yield_target_idx);
    notice.resume_target = deadlock_node_from_index(result->resume_target_idx);
    notice.expire_us = unresolved ? 0 : read_timer() + DEADLOCK_NOTICE_RESOLVED_US;
    pos_set_deadlock_notice(&notice);
}

int pos_deadlock_apply_result(const deadlock_result_t *result) {
    train_pos_t *victim;
    track_node *yield_target;
    track_node *resume_target;
    track_node *launch_origin;
    route_plan_t route_plan;
    track_node *actual_yield_target;

    if (!result || result->action == DEADLOCK_RESULT_NONE) return 0;
    if (!deadlock_result_snapshot_is_current(result)) return 0;

    if (result->action == DEADLOCK_RESULT_NOTICE_ONLY) {
        deadlock_write_notice_from_result(result, 1, NULL);
        return 1;
    }

    victim = pos_get(result->victim_train);
    if (!victim) return 0;

    yield_target = deadlock_node_from_index(result->yield_target_idx);
    resume_target = deadlock_node_from_index(result->resume_target_idx);
    launch_origin = deadlock_node_from_index(result->chosen_origin_idx);
    if (!yield_target || !launch_origin) return 0;

    deadlock_route_plan_from_serialized(&route_plan, &result->route_plan);

    if (resume_target != NULL) {
        victim->deadlock_recover.valid = 1;
        victim->deadlock_recover.resume_target = resume_target;
        victim->deadlock_recover.resume_offset_mm = result->resume_offset_mm;
        victim->deadlock_recover.yield_target = yield_target;
        victim->deadlock_recover.wait_start_mask = result->wait_start_mask;
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
    victim->parked_target_col = POS_TARGET_COL_NONE;
    pos_clear_committed_route(victim);

    if (!pos_launch_preplanned_route(victim, &route_plan, launch_origin,
                                     result->need_initial_reverse, 0,
                                     read_timer())) {
        return 0;
    }

    if (victim->route_state != TRAIN_STATE_ON_ROUTE &&
        victim->route_state != TRAIN_STATE_WAIT_SWITCH_SETTLE) {
        return 0;
    }

    actual_yield_target = deadlock_actual_yield_target(victim, yield_target);
    if (resume_target != NULL) {
        victim->deadlock_recover.yield_target = actual_yield_target;
        deadlock_record_yield_history(victim, actual_yield_target);
    }

    deadlock_write_notice_from_result(result, 0, actual_yield_target);
    return 1;
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
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos->replan.blocker_mask = 0;
    pos->replan.next_us = 0;
    pos_clear_committed_route(pos);
    return pos_try_direct_goto(pos);
}

int pos_deadlock_maybe_reroute_waiter(train_pos_t *pos, uint64_t now_us) {
    deadlock_participants_t parts;
    uint8_t cycle_mask = 0;
    int victim_train = -1;
    train_pos_t *victim;
    deadlock_kind_t deadlock_kind = DEADLOCK_KIND_NONE;

    if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    cycle_mask = deadlock_find_mask_for_train(pos->train_num, &deadlock_kind, &parts);
    if (!cycle_mask) return 0;

    deadlock_note_detected(&parts, cycle_mask, deadlock_kind);

    if (deadlock_kind == DEADLOCK_KIND_STOPPED_BLOCKER) {
        return deadlock_apply_stopped_blocker_reroute(cycle_mask, &parts, now_us,
                                                      &victim_train);
    }

    victim_train = deadlock_choose_victim(&parts, cycle_mask, deadlock_kind);
    if (victim_train < 0) return 0;

    victim = pos_get(victim_train);
    if (!victim) return 0;

    return deadlock_apply_reroute(victim, &parts, cycle_mask, now_us, 1);
}
