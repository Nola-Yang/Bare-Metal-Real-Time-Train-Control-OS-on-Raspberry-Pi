#include "train_tracking/deadlock.h"
#include "train_tracking/planner_core.h"
#include "track.h"
#include <stddef.h>
#include <stdint.h>

static planner_workspace_t g_snapshot_planner_ws;

static void snapshot_deadlock_route_plan_clear(deadlock_route_plan_t *out) {
    if (!out) return;
    *out = (deadlock_route_plan_t){0};
    out->chosen_target_idx = -1;
    out->reversal_sensor_idx = -1;
}

static void snapshot_deadlock_result_clear(deadlock_result_t *out) {
    if (!out) return;
    *out = (deadlock_result_t){0};
    out->action = DEADLOCK_RESULT_NONE;
    out->victim_train = -1;
    out->blocked_target_idx = -1;
    out->yield_target_idx = -1;
    out->resume_target_idx = -1;
    out->chosen_origin_idx = -1;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        out->cycle_trains[i] = -1;
        out->participants[i].train_num = -1;
        for (int j = 0; j < DEADLOCK_YIELD_HISTORY_MAX; j++) {
            out->participants[i].yield_history_idx[j] = -1;
        }
    }
    snapshot_deadlock_route_plan_clear(&out->route_plan);
}

static void snapshot_serialize_route_plan(deadlock_route_plan_t *out,
                                          const route_plan_t *plan) {
    if (!out) return;

    snapshot_deadlock_route_plan_clear(out);
    if (!plan) return;

    out->sw_count = plan->sw_count;
    for (int i = 0; i < plan->sw_count; i++) {
        out->sw_nums[i] = plan->sw_nums[i];
        out->sw_dirs[i] = plan->sw_dirs[i];
    }
    out->total_dist_mm = plan->total_dist_mm;
    out->chosen_target_idx = planner_node_index(plan->chosen_target);
    out->has_reversal = plan->has_reversal;
    out->reversal_sensor_idx = planner_node_index(plan->reversal_sensor);
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

static void snapshot_fill_view(const deadlock_participant_snapshot_t *part,
                               planner_train_view_t *out) {
    if (!out) return;
    *out = (planner_train_view_t){0};
    if (!part) return;

    out->train_num = part->train_num;
    out->train_ind = part->train_ind;
    out->route_state = part->route_state;
    out->goto_speed = part->goto_speed;
    out->cur_sensor = planner_node_from_index(part->cur_sensor_idx);
    out->origins[0] = planner_node_from_index(part->origin0_idx);
    out->origins[1] = planner_node_from_index(part->origin1_idx);
    out->goal = planner_node_from_index(part->goal_idx);
    out->goal_offset_mm = part->goal_offset_mm;
    out->blocker_mask = part->blocker_mask;
    out->resume_target = planner_node_from_index(part->resume_target_idx);
    out->resume_offset_mm = part->resume_offset_mm;
    out->yield_target = planner_node_from_index(part->yield_target_idx);
    out->parked_at_yield = part->parked_at_yield;
    out->wait_start_mask = part->wait_start_mask;
    out->yield_history_count = part->yield_history_count;
    for (int i = 0; i < part->yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        out->yield_history[i] = planner_node_from_index(part->yield_history_idx[i]);
    }
}

static void snapshot_fill_cycle_trains(deadlock_result_t *out,
                                       const planner_deadlock_participants_t *parts,
                                       uint8_t cycle_mask) {
    if (!out) return;

    out->cycle_count = 0;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        out->cycle_trains[i] = -1;
    }
    if (!parts) return;

    for (int i = 0; i < parts->count; i++) {
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        if (out->cycle_count >= DEADLOCK_MAX_TRAINS) break;
        out->cycle_trains[out->cycle_count++] = parts->train_nums[i];
    }
}

static void snapshot_copy_participants(deadlock_result_t *out,
                                       const deadlock_snapshot_t *snapshot) {
    if (!out || !snapshot) return;

    out->snapshot_now_us = snapshot->now_us;
    out->traffic_generation = snapshot->traffic_generation;
    out->switch_generation = snapshot->switch_generation;
    out->auto_dispatching_targets = snapshot->auto_dispatching_targets;
    out->participant_count = snapshot->participant_count;
    for (int i = 0; i < snapshot->participant_count && i < DEADLOCK_MAX_TRAINS; i++) {
        out->participants[i] = snapshot->participants[i];
    }
}

static int snapshot_plan_reroute(const planner_env_t *env,
                                 const planner_deadlock_participants_t *parts,
                                 const planner_train_view_t *victim,
                                 const planner_train_view_t *const *views,
                                 int view_count,
                                 uint8_t cycle_mask,
                                 int resume_after_yield,
                                 deadlock_result_t *out) {
    planner_eval_t eval;
    uint8_t global_cycle_mask;
    uint8_t unblocked_mask = 0;
    track_node *yield_target = NULL;
    int blocked_target_idx;

    if (!env || !parts || !victim || !out) return 0;

    blocked_target_idx = planner_node_index(victim->goal);
    if (blocked_target_idx < 0) return 0;

    global_cycle_mask = planner_deadlock_global_mask_from_local(parts, cycle_mask);
    if (!planner_pick_yield_target(env, victim, views, view_count,
                                   global_cycle_mask, &g_snapshot_planner_ws,
                                   &yield_target, &unblocked_mask, &eval,
                                   NULL) ||
        !yield_target || !eval.chosen_origin) {
        return 0;
    }

    out->action = DEADLOCK_RESULT_REROUTE;
    out->victim_train = victim->train_num;
    out->blocked_target_idx = blocked_target_idx;
    out->yield_target_idx = planner_node_index(yield_target);
    out->resume_target_idx = -1;
    out->resume_offset_mm = 0;
    out->wait_start_mask = resume_after_yield ? unblocked_mask : 0;
    out->chosen_origin_idx = planner_node_index(eval.chosen_origin);
    out->need_initial_reverse = eval.need_initial_reverse;
    snapshot_serialize_route_plan(&out->route_plan, &eval.plan);

    if (resume_after_yield) {
        if (victim->resume_target != NULL) {
            out->resume_target_idx = planner_node_index(victim->resume_target);
            out->resume_offset_mm = victim->resume_offset_mm;
        } else {
            out->resume_target_idx = blocked_target_idx;
            out->resume_offset_mm = victim->goal_offset_mm;
        }
    }

    snapshot_fill_cycle_trains(out, parts, cycle_mask);
    return 1;
}

static int snapshot_plan_notice_only(const planner_deadlock_participants_t *parts,
                                     uint8_t cycle_mask,
                                     int victim_train,
                                     deadlock_result_t *out) {
    if (!parts || !out || victim_train < 0) return 0;

    out->action = DEADLOCK_RESULT_NOTICE_ONLY;
    out->victim_train = victim_train;
    snapshot_fill_cycle_trains(out, parts, cycle_mask);
    return 1;
}

int deadlock_plan_from_snapshot(const deadlock_snapshot_t *snapshot,
                                deadlock_result_t *out) {
    planner_env_t env;
    planner_train_view_t live_views[DEADLOCK_MAX_TRAINS];
    const planner_train_view_t *view_ptrs[DEADLOCK_MAX_TRAINS];
    planner_deadlock_participants_t parts;
    planner_deadlock_kind_t kind = PLANNER_DEADLOCK_KIND_NONE;
    uint8_t cycle_mask = 0;
    int victim_train = -1;

    snapshot_deadlock_result_clear(out);
    if (!snapshot || !out) return 0;

    snapshot_copy_participants(out, snapshot);
    env.reservation_owner = snapshot->reservation_owner;
    env.switch_state = snapshot->switch_state;
    env.auto_dispatching_targets = snapshot->auto_dispatching_targets;

    for (int i = 0; i < snapshot->participant_count && i < DEADLOCK_MAX_TRAINS; i++) {
        snapshot_fill_view(&snapshot->participants[i], &live_views[i]);
        view_ptrs[i] = &live_views[i];
    }

    if (snapshot->participant_count < 2) return 0;

    for (int i = 0; i < snapshot->participant_count && i < DEADLOCK_MAX_TRAINS; i++) {
        const planner_train_view_t *view = view_ptrs[i];
        if (!view || view->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        cycle_mask = planner_detect_deadlock(&env, view_ptrs,
                                             snapshot->participant_count,
                                             view->train_num, &kind, &parts);
        if (cycle_mask != 0) break;
    }
    if (cycle_mask == 0) return 0;

    victim_train = planner_choose_victim(&parts, cycle_mask, kind);
    if (victim_train < 0) return 0;

    if (kind == PLANNER_DEADLOCK_KIND_STOPPED_BLOCKER) {
        for (int i = 0; i < parts.count; i++) {
            const planner_train_view_t *victim = parts.views[i];
            int keep_resume;

            if (!(cycle_mask & (uint8_t)(1u << i))) continue;
            if (!victim || victim->route_state != TRAIN_STATE_STOPPED) continue;

            keep_resume = victim->resume_target != NULL;
            if (snapshot_plan_reroute(&env, &parts, victim, view_ptrs,
                                      snapshot->participant_count, cycle_mask,
                                      keep_resume, out)) {
                return 1;
            }
        }
        snapshot_plan_notice_only(&parts, cycle_mask, victim_train, out);
        return (out->action != DEADLOCK_RESULT_NONE);
    }

    for (int i = 0; i < parts.count; i++) {
        const planner_train_view_t *victim = parts.views[i];
        if (!victim || victim->train_num != victim_train) continue;
        if (snapshot_plan_reroute(&env, &parts, victim, view_ptrs,
                                  snapshot->participant_count, cycle_mask, 1,
                                  out)) {
            return 1;
        }
        break;
    }

    snapshot_plan_notice_only(&parts, cycle_mask, victim_train, out);
    return (out->action != DEADLOCK_RESULT_NONE);
}
