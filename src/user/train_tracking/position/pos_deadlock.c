#include "train_tracking/position_priv.h"
#include "train_tracking/deadlock.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "timer.h"
#include "demo_manager.h"
#include <stddef.h>
#include <stdint.h>

static const int g_deadlock_train_order[6] = {13, 14, 15, 17, 18, 55};

static void deadlock_record_yield_history(train_pos_t *pos, track_node *yield_target) {
    pos_deadlock_recover_t *recover;

    if (!pos || !yield_target) return;
    recover = &pos->deadlock_recover;

    for (int i = 0; i < recover->yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        if (planner_same_physical_sensor(recover->yield_history[i], yield_target)) {
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
    out->chosen_target = planner_node_from_index(plan->chosen_target_idx);
    out->has_reversal = plan->has_reversal;
    out->reversal_sensor = planner_node_from_index(plan->reversal_sensor_idx);
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

static void deadlock_fill_snapshot_views(
    const deadlock_snapshot_t *snapshot,
    planner_train_view_t views[DEADLOCK_MAX_TRAINS],
    const planner_train_view_t *view_ptrs[DEADLOCK_MAX_TRAINS]) {
    if (!snapshot || !views || !view_ptrs) return;

    for (int i = 0; i < snapshot->participant_count && i < DEADLOCK_MAX_TRAINS; i++) {
        const deadlock_participant_snapshot_t *part = &snapshot->participants[i];
        planner_train_view_t *view = &views[i];

        *view = (planner_train_view_t){0};
        view->train_num = part->train_num;
        view->train_ind = part->train_ind;
        view->route_state = part->route_state;
        view->goto_speed = part->goto_speed;
        view->cur_sensor = planner_node_from_index(part->cur_sensor_idx);
        view->origins[0] = planner_node_from_index(part->origin0_idx);
        view->origins[1] = planner_node_from_index(part->origin1_idx);
        view->goal = planner_node_from_index(part->goal_idx);
        view->goal_offset_mm = part->goal_offset_mm;
        view->blocker_mask = part->blocker_mask;
        view->resume_target = planner_node_from_index(part->resume_target_idx);
        view->resume_offset_mm = part->resume_offset_mm;
        view->yield_target = planner_node_from_index(part->yield_target_idx);
        view->parked_at_yield = part->parked_at_yield;
        view->wait_start_mask = part->wait_start_mask;
        view->yield_history_count = part->yield_history_count;
        for (int j = 0; j < part->yield_history_count &&
                        j < DEADLOCK_YIELD_HISTORY_MAX; j++) {
            view->yield_history[j] =
                planner_node_from_index(part->yield_history_idx[j]);
        }
        view_ptrs[i] = view;
    }
}

static int deadlock_notice_still_active(const pos_deadlock_notice_t *notice) {
    deadlock_snapshot_t snapshot;
    planner_env_t env;
    planner_train_view_t views[DEADLOCK_MAX_TRAINS];
    const planner_train_view_t *view_ptrs[DEADLOCK_MAX_TRAINS] = {0};
    planner_deadlock_participants_t parts;
    planner_deadlock_kind_t kind = PLANNER_DEADLOCK_KIND_NONE;
    int first_train = -1;
    int wait_train = -1;
    uint8_t expected_mask = 0;
    uint8_t cycle_mask;
    uint8_t cycle_global_mask;

    if (!notice || !notice->active || !notice->unresolved) return 0;
    pos_deadlock_build_snapshot(&snapshot, read_timer());

    for (int i = 0; i < notice->cycle_count && i < DEADLOCK_MAX_TRAINS; i++) {
        int train_num = notice->cycle_trains[i];
        uint8_t bit = planner_train_bit(train_num);

        if (!bit) return 0;
        if (first_train < 0) first_train = train_num;
        for (int j = 0; j < snapshot.participant_count; j++) {
            const deadlock_participant_snapshot_t *part = &snapshot.participants[j];
            if (part->train_num == train_num &&
                part->route_state == TRAIN_STATE_WAIT_RESOURCE &&
                wait_train < 0) {
                wait_train = train_num;
            }
        }
        expected_mask |= bit;
    }

    for (int i = 0; i < snapshot.participant_count; i++) {
        const deadlock_participant_snapshot_t *part = &snapshot.participants[i];
        if (part->train_num == notice->victim_train &&
            part->route_state == TRAIN_STATE_WAIT_RESOURCE &&
            (expected_mask & planner_train_bit(notice->victim_train))) {
            first_train = notice->victim_train;
            break;
        }
    }

    if ((expected_mask & planner_train_bit(notice->victim_train)) &&
        first_train == notice->victim_train) {
        first_train = notice->victim_train;
    } else if (wait_train >= 0) {
        first_train = wait_train;
    }
    if (first_train < 0 ||
        planner_route_blocker_mask_bit_count(expected_mask) < 2 ||
        snapshot.participant_count < 2) {
        return 0;
    }

    env.reservation_owner = snapshot.reservation_owner;
    env.switch_state = snapshot.switch_state;
    env.auto_dispatching_targets = snapshot.auto_dispatching_targets;
    deadlock_fill_snapshot_views(&snapshot, views, view_ptrs);

    cycle_mask = planner_detect_deadlock(&env, view_ptrs, snapshot.participant_count,
                                         first_train, &kind, &parts);
    if (!cycle_mask) return 0;

    cycle_global_mask = planner_deadlock_global_mask_from_local(&parts, cycle_mask);
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

static track_node *deadlock_actual_yield_target(const train_pos_t *pos,
                                                track_node *fallback) {
    if (!pos) return fallback;
    if (pos->midrev.active && pos->midrev.final_target != NULL) {
        return pos->midrev.final_target;
    }
    if (pos->target_sensor != NULL) return pos->target_sensor;
    return fallback;
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
    goal = pos_planner_current_goal(pos, &goal_offset);

    out->train_num = pos->train_num;
    out->train_ind = pos->train_ind;
    out->route_state = pos->route_state;
    out->goto_speed = pos->goto_speed;
    out->cur_sensor_idx = planner_node_index(pos->cur_sensor);
    out->origin0_idx = planner_node_index(origins[0]);
    out->origin1_idx = planner_node_index(origins[1]);
    out->goal_idx = planner_node_index(goal);
    out->goal_offset_mm = goal_offset;
    out->blocker_mask = pos->replan.blocker_mask;
    out->resume_target_idx = planner_node_index(pos->deadlock_recover.resume_target);
    out->resume_offset_mm = pos->deadlock_recover.resume_offset_mm;
    out->yield_target_idx = planner_node_index(pos->deadlock_recover.yield_target);
    out->parked_at_yield = pos->deadlock_recover.parked_at_yield;
    out->wait_start_mask = pos->deadlock_recover.wait_start_mask;
    out->yield_history_count = pos->deadlock_recover.yield_history_count;
    for (int i = 0; i < pos->deadlock_recover.yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        out->yield_history_idx[i] =
            planner_node_index(pos->deadlock_recover.yield_history[i]);
    }
}

void pos_deadlock_build_snapshot(deadlock_snapshot_t *out, uint64_t now_us) {
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

    out->participant_count = 0;
    for (int i = 0; i < 6 && out->participant_count < DEADLOCK_MAX_TRAINS; i++) {
        train_pos_t *pos = pos_get(g_deadlock_train_order[i]);

        if (!pos) continue;
        if (pos->route_state != TRAIN_STATE_WAIT_RESOURCE &&
            !deadlock_train_is_stopped_blocker(pos)) {
            continue;
        }

        deadlock_fill_participant_snapshot(
            &out->participants[out->participant_count], pos);
        out->participant_count++;
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
    notice.blocked_target = planner_node_from_index(result->blocked_target_idx);
    notice.yield_target = yield_target_override
                              ? yield_target_override
                              : planner_node_from_index(result->yield_target_idx);
    notice.resume_target = planner_node_from_index(result->resume_target_idx);
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

    yield_target = planner_node_from_index(result->yield_target_idx);
    resume_target = planner_node_from_index(result->resume_target_idx);
    launch_origin = planner_node_from_index(result->chosen_origin_idx);
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
