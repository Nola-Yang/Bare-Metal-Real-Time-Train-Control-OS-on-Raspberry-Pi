#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "kassert.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

static route_plan_t g_midrev_second_leg_plan;

static uint8_t deadlock_blocker_mask_from_trains(int requester_train,
                                                 const int *trains, int count) {
    uint8_t mask = 0;
    for (int i = 0; i < count; i++) {
        if (trains[i] == requester_train) continue;
        mask |= pos_deadlock_train_bit(trains[i]);
    }
    return mask;
}

static uint8_t deadlock_blocker_mask_from_plan(int requester_train,
                                               const route_plan_t *plan) {
    int blockers[6];
    int count = traffic_collect_plan_blockers(requester_train, plan, blockers, 6);
    return deadlock_blocker_mask_from_trains(requester_train, blockers, count);
}

static int event_route_switch_needs_change(int sw_num, char desired_dir) {
    int sw_idx = track_switch_to_index(sw_num);
    if (sw_idx < 0) return 1;
    return track_get_switch_state()[sw_idx].state != desired_dir;
}

static uint8_t deadlock_blocker_mask_from_switches(const int *sw_nums,
                                                   const char *sw_dirs,
                                                   int sw_count,
                                                   int requester_train) {
    uint8_t mask = 0;
    for (int i = 0; i < sw_count; i++) {
        int blockers[6];
        int count;
        if (!event_route_switch_needs_change(sw_nums[i], sw_dirs[i])) continue;
        count = traffic_collect_switch_envelope_blockers(sw_nums[i], blockers, 6);
        mask |= deadlock_blocker_mask_from_trains(requester_train, blockers, count);
    }
    return mask;
}

static int deadlock_bit_count(uint8_t mask) {
    int count = 0;
    while (mask) {
        count += (mask & 1u);
        mask >>= 1;
    }
    return count;
}

static int deadlock_train_has_started(const train_pos_t *pos) {
    if (!pos) return 0;
    switch (pos->route_state) {
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

static void deadlock_build_graph(uint8_t adj[6], uint8_t *out_wait_mask) {
    uint8_t wait_mask = 0;

    for (int i = 0; i < 6; i++) adj[i] = 0;

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        int idx;
        if (pos->train_num < 0) continue;
        if (pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        idx = pos_deadlock_train_to_index(pos->train_num);
        if (idx < 0) continue;
        wait_mask |= (uint8_t)(1u << idx);
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

static uint8_t deadlock_find_cycle_mask_for_train(int train_num) {
    uint8_t adj[6];
    uint8_t reach[6];
    uint8_t wait_mask = 0;
    uint8_t cycle = 0;
    int start_idx = pos_deadlock_train_to_index(train_num);
    if (start_idx < 0) return 0;

    deadlock_build_graph(adj, &wait_mask);
    if (!(wait_mask & (uint8_t)(1u << start_idx))) return 0;

    deadlock_compute_reachability(adj, wait_mask, reach);
    for (int i = 0; i < 6; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        if (!(wait_mask & bit)) continue;
        if ((reach[start_idx] & bit) && (reach[i] & (uint8_t)(1u << start_idx))) {
            cycle |= bit;
        }
    }

    return (deadlock_bit_count(cycle) >= 2) ? cycle : 0;
}

static int deadlock_choose_victim(uint8_t cycle_mask) {
    for (int i = 0; i < 6; i++) {
        if (cycle_mask & (uint8_t)(1u << i)) return pos_deadlock_index_to_train(i);
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
    uint8_t expected_mask = 0;
    uint8_t cycle_mask;

    if (!notice || !notice->active || !notice->unresolved) return 0;
    for (int i = 0; i < notice->cycle_count && i < 6; i++) {
        int train_num = notice->cycle_trains[i];
        uint8_t bit = pos_deadlock_train_bit(train_num);
        if (!bit) return 0;
        if (first_train < 0) first_train = train_num;
        expected_mask |= bit;
    }
    if (first_train < 0 || deadlock_bit_count(expected_mask) < 2) return 0;

    cycle_mask = deadlock_find_cycle_mask_for_train(first_train);
    return (cycle_mask & expected_mask) == expected_mask;
}

static void deadlock_refresh_notice_state(void) {
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

static void deadlock_reset_midrev_for_reroute(train_pos_t *pos) {
    if (!pos) return;
    pos->midrev.active = 0;
    pos->midrev.sensor = NULL;
    pos->midrev.final_target = NULL;
    pos->midrev.final_offset = 0;
    pos->midrev.sw_count = 0;
    pos->midrev.dist_after = 0;
    pos->midrev.path2_count = 0;
}

static void deadlock_apply_reroute(train_pos_t *victim, uint8_t cycle_mask,
                                   uint64_t now_us) {
    track_node *yield_target = NULL;
    track_node *blocked_target;
    track_node *resume_target;
    uint8_t unblocked_mask = 0;
    int32_t blocked_offset = 0;
    int had_resume = 0;

    if (!victim) return;

    blocked_target = deadlock_current_target(victim, &blocked_offset);
    if (!blocked_target) {
        deadlock_write_notice(cycle_mask, victim->train_num, NULL, NULL, NULL, 0, 1);
        return;
    }

    had_resume = victim->deadlock_recover.valid && victim->deadlock_recover.resume_target != NULL;
    resume_target = had_resume ? victim->deadlock_recover.resume_target : blocked_target;

    if (!pos_pick_deadlock_yield_target(victim, cycle_mask, &yield_target,
                                        &unblocked_mask) ||
        !yield_target || unblocked_mask == 0) {
        deadlock_write_notice(cycle_mask, victim->train_num, blocked_target, NULL,
                              resume_target, 0, 1);
        return;
    }

    if (!victim->deadlock_recover.valid) {
        victim->deadlock_recover.valid = 1;
        victim->deadlock_recover.resume_target = blocked_target;
        victim->deadlock_recover.resume_offset_mm = blocked_offset;
    }
    victim->deadlock_recover.yield_target = yield_target;
    victim->deadlock_recover.wait_start_mask = unblocked_mask;
    victim->deadlock_recover.parked_at_yield = 0;

    victim->pending_target = yield_target;
    victim->pending_offset_mm = 0;
    victim->orig_user_target = yield_target;
    victim->orig_target_offset = 0;
    victim->target_sensor = yield_target;
    victim->target_offset_mm = 0;
    victim->dist_to_target_mm = 0;
    victim->route_path_count = 0;
    victim->route_path_cursor = 0;
    victim->route_rem_tick_us = 0;
    deadlock_reset_midrev_for_reroute(victim);

    if (!pos_try_direct_goto(victim) ||
        (victim->route_state != TRAIN_STATE_ON_ROUTE &&
         victim->route_state != TRAIN_STATE_WAIT_SWITCH_SETTLE)) {
        deadlock_write_notice(cycle_mask, victim->train_num, blocked_target, NULL,
                              victim->deadlock_recover.resume_target, 0, 1);
        return;
    }

    deadlock_write_notice(cycle_mask, victim->train_num, blocked_target, yield_target,
                          victim->deadlock_recover.resume_target,
                          now_us + DEADLOCK_NOTICE_RESOLVED_US, 0);
}

static int deadlock_resume_waiting_trains_started(uint8_t wait_start_mask) {
    int any_waiting = 0;

    for (int i = 0; i < 6; i++) {
        train_pos_t *other;
        uint8_t bit = (uint8_t)(1u << i);
        if (!(wait_start_mask & bit)) continue;

        other = pos_get(pos_deadlock_index_to_train(i));
        if (!other) continue;
        if (deadlock_train_has_started(other)) return 1;
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
        !deadlock_resume_waiting_trains_started(pos->deadlock_recover.wait_start_mask)) {
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
    KASSERT(pos_try_direct_goto(pos));
    return 1;
}

void pos_replan_on_tick(uint64_t now_us) {
    /* Higher train numbers win WAIT_RESOURCE replans so a yielded smaller
     * train does not immediately reclaim the route before the blocked peer
     * can plan and launch. */
    static const int ORDER[6] = {55, 18, 17, 15, 14, 13};
    deadlock_refresh_notice_state();
    for (int wi = 0; wi < 6; wi++) {
        train_pos_t *pos = pos_get(ORDER[wi]);
        uint8_t cycle_mask = 0;
        int victim_train = -1;
        if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

        uint32_t generation = traffic_get_change_generation();
        int woke_on_change = 0;
        if (generation != pos->replan.seen_generation) {
            pos->replan.seen_generation = generation;
            pos->replan.retry_count = 0;
            woke_on_change = 1;
        }
        if (!woke_on_change &&
            pos->replan.next_us > 0 && now_us < pos->replan.next_us) {
            continue;
        }

        int backoff_exp = pos->replan.retry_count;
        if (backoff_exp > REPLAN_MAX_BACKOFF) backoff_exp = REPLAN_MAX_BACKOFF;
        uint64_t backoff_us = REPLAN_INTERVAL_US << backoff_exp;

        pos->replan.rand_state = pos->replan.rand_state * 1664525u + 1013904223u;
        uint64_t jitter_us = (pos->replan.rand_state >> 16) % (uint32_t)REPLAN_INTERVAL_US;
        pos->replan.next_us = now_us + backoff_us + jitter_us;
        pos->replan.retry_count++;

        pos_restore_pending_target(pos);

        if (pos->pending_target == NULL) continue;

        cycle_mask = deadlock_find_cycle_mask_for_train(pos->train_num);
        if (cycle_mask) {
            victim_train = deadlock_choose_victim(cycle_mask);
            deadlock_write_detected_notice(cycle_mask, victim_train);
            if (victim_train != pos->train_num) continue;
            deadlock_apply_reroute(pos, cycle_mask, now_us);
            continue;
        }

        int ok = pos_try_direct_goto(pos);
        KASSERT(ok);
    }
}

/* A midpoint reversal may stop at the reversal sensor and then block before the
 * second leg can be reserved. Collapse that half-complete midrev into a plain
 * WAIT on the final target. */
static void collapse_midrev_wait_target(train_pos_t *pos) {
    if (!pos || !pos->midrev.active) return;

    track_node *final_target = pos->midrev.final_target;
    int32_t final_offset = pos->midrev.final_offset;

    pos->target_sensor = final_target;
    pos->target_offset_mm = final_offset;
    pos->pending_target = final_target;
    pos->pending_offset_mm = final_offset;
    pos->orig_user_target = final_target;
    pos->orig_target_offset = final_offset;
    pos->dist_to_target_mm = 0;

    pos->midrev.active = 0;
    pos->midrev.sensor = NULL;
    pos->midrev.final_target = NULL;
    pos->midrev.final_offset = 0;
    pos->midrev.sw_count = 0;
    pos->midrev.dist_after = 0;
    pos->midrev.path2_count = 0;
}

int pos_handle_midrev_resume(train_pos_t *pos, uint64_t now_us) {
    route_plan_t *second_leg_plan = &g_midrev_second_leg_plan;

    *second_leg_plan = (route_plan_t){0};
    second_leg_plan->path_count = pos->midrev.path2_count;
    for (int j = 0; j < pos->midrev.path2_count; j++) {
        second_leg_plan->path_nodes[j] = pos->midrev.path2[j];
    }
    if (!traffic_can_reserve_plan(pos->train_num, second_leg_plan)) {
        uint8_t blocker_mask = deadlock_blocker_mask_from_plan(pos->train_num,
                                                               second_leg_plan);
        collapse_midrev_wait_target(pos);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 0;
    }
    int sw_owner = pos_route_switch_blocker(pos->midrev.sw_nums, pos->midrev.sw_dirs,
                                            pos->midrev.sw_count, pos->train_num);
    if (sw_owner == pos->train_num) {
        track_node *final_target = pos->midrev.final_target;
        int32_t final_offset = pos->midrev.final_offset;

        traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                        TRAIN_BODY_MM,
                                        pos_release_keep_end(pos->cur_sensor,
                                                             pos->pred.next_sensor));
        pos->midrev.active = 0;
        pos->pending_target = final_target;
        pos->pending_offset_mm = final_offset;
        pos->orig_user_target = final_target;
        pos->orig_target_offset = final_offset;
        KASSERT(pos_try_direct_goto(pos));
        return 1;
    }
    if (sw_owner >= 0) {
        uint8_t blocker_mask = deadlock_blocker_mask_from_switches(pos->midrev.sw_nums,
                                                                   pos->midrev.sw_dirs,
                                                                   pos->midrev.sw_count,
                                                                   pos->train_num);
        collapse_midrev_wait_target(pos);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 0;
    }
    if (!pos_apply_route_switches_safe(pos->midrev.sw_nums, pos->midrev.sw_dirs,
                                       pos->midrev.sw_count, pos->train_num)) {
        uint8_t blocker_mask = deadlock_blocker_mask_from_switches(pos->midrev.sw_nums,
                                                                   pos->midrev.sw_dirs,
                                                                   pos->midrev.sw_count,
                                                                   pos->train_num);
        collapse_midrev_wait_target(pos);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 0;
    }
    if (!traffic_reserve_plan(pos->train_num, pos->cur_sensor, second_leg_plan)) {
        uint8_t blocker_mask = deadlock_blocker_mask_from_plan(pos->train_num,
                                                               second_leg_plan);
        collapse_midrev_wait_target(pos);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 0;
    }
    if (pos->midrev.sw_count > 0) ui_mark_switches_dirty();

    pos->midrev.active = 0;

    track_reverse(pos->train_num);
    pos->going_forward = !pos->going_forward;
    if (pos->cur_sensor && pos->cur_sensor->reverse)
        pos->cur_sensor = pos->cur_sensor->reverse;

    pos->target_sensor = pos->midrev.final_target;
    pos->target_offset_mm = pos->midrev.final_offset;

    /* Switch active path to the stored second leg. */
    pos->route_path_count = pos->midrev.path2_count;
    for (int j = 0; j < pos->midrev.path2_count; j++)
        pos->route_path[j] = pos->midrev.path2[j];
    pos->route_path_cursor = 0;

    int32_t pd2 = route_path_dist_from(pos->route_path, 0, pos->route_path_count);
    int32_t d2 = (pd2 >= 0) ? pd2 : pos->midrev.dist_after;
    pos->dist_to_target_mm = d2 + pos->midrev.final_offset;
    if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;

    pos_arm_switch_settle(pos, pos->midrev.sw_count,
                          POS_SWITCH_SETTLE_REVERSED, now_us);
    return 1;
}
