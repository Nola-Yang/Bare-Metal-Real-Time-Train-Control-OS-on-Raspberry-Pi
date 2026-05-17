#include "train_tracking/position.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "../traffic/traffic_window_internal.h"
#include "track.h"
#include <stddef.h>

static int route_switch_needs_change(int sw_num, char desired_dir) {
    int sw_idx = track_switch_to_index(sw_num);
    if (sw_idx < 0) return 1;
    return track_get_switch_state()[sw_idx].state != desired_dir;
}

static void pos_wait_info_reset(pos_wait_info_t *out) {
    if (!out) return;
    out->active = 0;
    out->blocked_by_switch = 0;
    out->node = NULL;
    out->blocker_train = -1;
    out->switch_num = -1;
}

static void pos_wait_info_set(pos_wait_info_t *out, track_node *node,
                              int blocker_train, int blocked_by_switch,
                              int switch_num) {
    if (!out) return;
    out->active = 1;
    out->blocked_by_switch = blocked_by_switch ? 1 : 0;
    out->node = node;
    out->blocker_train = blocker_train;
    out->switch_num = switch_num;
}

static int pos_query_build_wait_plan(const train_pos_t *pos, route_plan_t *out_plan) {
    int start_cursor;

    if (!pos || !out_plan) return 0;
    if (pos->route_path_count <= 0) return 0;

    start_cursor = (pos->replan.wait_mode == POS_WAIT_RESUME_ROUTE)
                       ? pos->route_path_cursor
                       : 0;
    if (start_cursor < 0) start_cursor = 0;
    if (start_cursor >= pos->route_path_count) return 0;

    return traffic_window_build_prefix_plan(pos->route_path, pos->route_path_count,
                                            start_cursor, pos->route_path_count - 1,
                                            out_plan);
}

static int pos_query_build_midrev_second_leg_plan(const train_pos_t *pos,
                                                  route_plan_t *out_plan) {
    if (!pos || !out_plan || pos->midrev.path2_count <= 0) return 0;

    *out_plan = (route_plan_t){0};
    out_plan->path_count = pos->midrev.path2_count;
    for (int i = 0; i < pos->midrev.path2_count; i++) {
        out_plan->path_nodes[i] = pos->midrev.path2[i];
    }
    return 1;
}

static int pos_query_first_blocking_switch(const int *sw_nums, const char *sw_dirs,
                                           int sw_count, int requester_train,
                                           int *out_owner) {
    if (out_owner) *out_owner = -1;

    for (int i = sw_count - 1; i >= 0; i--) {
        int owner;
        if (!route_switch_needs_change(sw_nums[i], sw_dirs[i])) continue;
        owner = traffic_can_set_switch_for_plan(sw_nums[i], requester_train);
        if (owner < 0) continue;
        if (out_owner) *out_owner = owner;
        return sw_nums[i];
    }

    return -1;
}

static track_node *pos_query_find_authority_blocking_node(train_pos_t *pos,
                                                          const route_plan_t *full_plan,
                                                          int *out_owner,
                                                          int *out_switch_num) {
    route_plan_t prefix;

    if (out_owner) *out_owner = -1;
    if (out_switch_num) *out_switch_num = -1;
    if (!pos || !full_plan || full_plan->path_count <= 0) return NULL;

    for (int end_cursor = 0; end_cursor < full_plan->path_count; end_cursor++) {
        int32_t dist_mm;
        int switch_owner = -1;
        int switch_num;

        if (!traffic_window_build_prefix_plan(full_plan->path_nodes,
                                              full_plan->path_count,
                                              0,
                                              end_cursor,
                                              &prefix)) {
            break;
        }

        dist_mm = route_path_dist_from(full_plan->path_nodes, 0, end_cursor + 1);
        if (dist_mm < 0) break;
        if (dist_mm <= 0) continue;

        if (end_cursor != full_plan->path_count - 1 &&
            g_track[full_plan->path_nodes[end_cursor]].type != NODE_SENSOR) {
            continue;
        }

        switch_num = pos_query_first_blocking_switch(prefix.sw_nums, prefix.sw_dirs,
                                                     prefix.sw_count, pos->train_num,
                                                     &switch_owner);
        if (switch_num > 0) {
            if (out_switch_num) *out_switch_num = switch_num;
            return traffic_find_switch_blocking_node(switch_num, out_owner);
        }

        if (!traffic_can_reserve_plan(pos->train_num, &prefix)) {
            return traffic_find_plan_blocking_node(pos->train_num, &prefix, out_owner);
        }

        if (dist_mm >= pos_route_authority_target_mm(pos)) {
            return NULL;
        }
    }

    return NULL;
}

pos_target_query_status_t pos_query_target(int train_num, track_node *target,
                                           pos_target_query_t *out) {
    train_pos_t *pos = pos_find_slot(train_num);
    pos_route_eval_t eval;
    pos_route_eval_result_t result;

    if (out) {
        out->status = POS_TARGET_UNREACHABLE;
        out->plan = (route_plan_t){0};
        out->blocker_mask = 0;
    }

    if (!pos || !target) return POS_TARGET_UNREACHABLE;

    result = pos_evaluate_target_ready_now(pos, target, &eval);
    if (out) {
        out->plan = eval.plan;
        out->blocker_mask = eval.blocker_mask;
    }

    switch (result) {
    case POS_ROUTE_EVAL_READY:
        if (out) out->status = POS_TARGET_READY;
        return POS_TARGET_READY;
    case POS_ROUTE_EVAL_BLOCKED:
        if (out) out->status = POS_TARGET_BLOCKED;
        return POS_TARGET_BLOCKED;
    case POS_ROUTE_EVAL_UNREACHABLE:
    default:
        return POS_TARGET_UNREACHABLE;
    }
}

void pos_get_wait_info(int train_num, pos_wait_info_t *out) {
    train_pos_t *pos = pos_find_slot(train_num);
    route_plan_t plan;
    route_plan_t authority_plan;
    pos_route_eval_t eval;
    track_node *node = NULL;
    track_node *target = NULL;
    int blocker_train = -1;
    int switch_num = -1;
    int reserved_end_cursor = -1;
    int switch_blocker_owner = -1;
    uint8_t blocker_mask = 0;

    pos_wait_info_reset(out);
    if (!out || !pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) return;

    switch ((pos_wait_mode_t)pos->replan.wait_mode) {
    case POS_WAIT_PRELAUNCH_ROUTE:
    case POS_WAIT_RESUME_ROUTE:
        if (!pos_query_build_wait_plan(pos, &plan)) return;

        if (!pos_route_authority_prepare_launch(pos, &plan, &authority_plan,
                                                &reserved_end_cursor,
                                                &switch_blocker_owner,
                                                &blocker_mask)) {
            node = pos_query_find_authority_blocking_node(pos, &plan,
                                                          &blocker_train,
                                                          &switch_num);
            if (node || switch_num > 0) {
                pos_wait_info_set(out, node, blocker_train, switch_num > 0, switch_num);
            }
            return;
        }

        switch_num = pos_query_first_blocking_switch(authority_plan.sw_nums,
                                                     authority_plan.sw_dirs,
                                                     authority_plan.sw_count,
                                                     pos->train_num,
                                                     &blocker_train);
        if (switch_num > 0) {
            node = traffic_find_switch_blocking_node(switch_num, &blocker_train);
            pos_wait_info_set(out, node, blocker_train, 1, switch_num);
            return;
        }

        if (!traffic_can_reserve_plan(pos->train_num, &authority_plan)) {
            node = traffic_find_plan_blocking_node(pos->train_num, &authority_plan,
                                                   &blocker_train);
            if (node) pos_wait_info_set(out, node, blocker_train, 0, -1);
        }
        return;

    case POS_WAIT_MIDREV_SECOND_LEG:
        if (!pos_query_build_midrev_second_leg_plan(pos, &plan)) return;

        if (!traffic_can_reserve_plan(pos->train_num, &plan)) {
            node = traffic_find_plan_blocking_node(pos->train_num, &plan,
                                                   &blocker_train);
            if (node) pos_wait_info_set(out, node, blocker_train, 0, -1);
            return;
        }

        switch_num = pos_query_first_blocking_switch(pos->midrev.sw_nums,
                                                     pos->midrev.sw_dirs,
                                                     pos->midrev.sw_count,
                                                     pos->train_num,
                                                     &blocker_train);
        if (switch_num > 0) {
            node = traffic_find_switch_blocking_node(switch_num, &blocker_train);
            pos_wait_info_set(out, node, blocker_train, 1, switch_num);
        }
        return;

    case POS_WAIT_NONE:
    default:
        pos_restore_pending_target(pos);
        target = pos->pending_target ? pos->pending_target
                                     : pos_route_current_goal(pos);
        if (!target) return;
        if (pos_evaluate_target_ready_now(pos, target, &eval) != POS_ROUTE_EVAL_BLOCKED) {
            return;
        }

        plan = eval.plan;
        if (plan.has_reversal) {
            plan.path_count2 = 0;
        }

        if (!traffic_can_reserve_plan(pos->train_num, &plan)) {
            node = traffic_find_plan_blocking_node(pos->train_num, &plan,
                                                   &blocker_train);
            if (node) pos_wait_info_set(out, node, blocker_train, 0, -1);
            return;
        }

        switch_num = pos_query_first_blocking_switch(eval.plan.sw_nums,
                                                     eval.plan.sw_dirs,
                                                     eval.plan.sw_count,
                                                     pos->train_num,
                                                     &blocker_train);
        if (switch_num > 0) {
            node = traffic_find_switch_blocking_node(switch_num, &blocker_train);
            pos_wait_info_set(out, node, blocker_train, 1, switch_num);
        }
        return;
    }
}
