#include "train_tracking/position_priv.h"
#include "train_tracking/planner_core.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include <stddef.h>
#include <stdint.h>

static planner_workspace_t g_pos_route_ws;
static planner_eval_t g_pos_eval_ready;
static int g_pos_planner_owners[TRACK_MAX];
static char g_pos_planner_switch_state[MAX_SWITCHES];

static void pos_planner_eval_to_pos(const planner_eval_t *src,
                                    pos_route_eval_t *out) {
    if (!out) return;
    *out = (pos_route_eval_t){0};
    if (!src) return;
    out->plan = src->plan;
    out->chosen_origin = src->chosen_origin;
    out->need_initial_reverse = src->need_initial_reverse;
    out->blocker_mask = src->blocker_mask;
}

track_node *pos_route_current_goal(train_pos_t *pos) {
    return pos_planner_current_goal(pos, NULL);
}

track_node *pos_planner_current_goal(const train_pos_t *pos,
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

static int pos_route_use_cur_reverse_start(const train_pos_t *pos) {
    return pos != NULL &&
           (pos->route_state == TRAIN_STATE_STOPPED ||
            pos->route_state == TRAIN_STATE_WAIT_RESOURCE) &&
           pos->stopped_on_target_hit;
}

void pos_route_fill_origins(const train_pos_t *pos, track_node *origins[2]) {
    track_node *cur_sensor_orig;
    track_node *plan_start;
    track_node *reverse_plan_start;
    uint64_t dt_ignored = 0;

    if (!origins) return;
    origins[0] = NULL;
    origins[1] = NULL;
    if (!pos || !pos->cur_sensor) return;

    cur_sensor_orig = pos->cur_sensor;
    plan_start = pos->pred.next_sensor;
    if (!plan_start) {
        plan_start = predict_next_sensor((train_pos_t *)pos, pos->cur_sensor,
                                         &dt_ignored);
    }
    reverse_plan_start = pos_route_use_cur_reverse_start(pos)
                         ? cur_sensor_orig->reverse
                         : (plan_start ? plan_start->reverse
                                       : cur_sensor_orig->reverse);
    origins[0] = plan_start;
    origins[1] = reverse_plan_start;
}

void pos_planner_build_env(planner_env_t *out, int owners[TRACK_MAX],
                           char switch_state[MAX_SWITCHES]) {
    const switch_entry_t *live_switches = track_get_switch_state();

    if (!out || !owners || !switch_state) return;

    traffic_snapshot_reservations(owners);
    for (int i = 0; i < MAX_SWITCHES; i++) {
        switch_state[i] = live_switches[i].state;
    }

    out->reservation_owner = owners;
    out->switch_state = switch_state;
    out->auto_dispatching_targets = 0;
}

void pos_planner_fill_view(const train_pos_t *pos, planner_train_view_t *out) {
    int32_t goal_offset = 0;
    track_node *goal;

    if (!out) return;
    *out = (planner_train_view_t){0};
    if (!pos) return;

    goal = pos_planner_current_goal(pos, &goal_offset);

    out->train_num = pos->train_num;
    out->train_ind = pos->train_ind;
    out->route_state = pos->route_state;
    out->goto_speed = pos->goto_speed;
    out->cur_sensor = pos->cur_sensor;
    pos_route_fill_origins(pos, out->origins);
    out->goal = goal;
    out->goal_offset_mm = goal_offset;
    out->blocker_mask = pos->replan.blocker_mask;
    out->resume_target = pos->deadlock_recover.resume_target;
    out->resume_offset_mm = pos->deadlock_recover.resume_offset_mm;
    out->yield_target = pos->deadlock_recover.yield_target;
    out->parked_at_yield = pos->deadlock_recover.parked_at_yield;
    out->wait_start_mask = pos->deadlock_recover.wait_start_mask;
    out->yield_history_count = pos->deadlock_recover.yield_history_count;
    for (int i = 0; i < pos->deadlock_recover.yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        out->yield_history[i] = pos->deadlock_recover.yield_history[i];
    }
}

uint8_t pos_route_blocker_mask_from_plan(int requester_train,
                                         const route_plan_t *plan) {
    planner_env_t env;

    pos_planner_build_env(&env, g_pos_planner_owners, g_pos_planner_switch_state);
    return planner_route_blocker_mask_from_plan(&env, requester_train, plan);
}

uint8_t pos_route_blocker_mask_from_switches(const int *sw_nums,
                                             const char *sw_dirs,
                                             int sw_count,
                                             int requester_train) {
    planner_env_t env;

    pos_planner_build_env(&env, g_pos_planner_owners, g_pos_planner_switch_state);
    return planner_route_blocker_mask_from_switches(&env, sw_nums, sw_dirs,
                                                    sw_count, requester_train);
}

int pos_route_blocker_mask_bit_count(uint8_t mask) {
    return planner_route_blocker_mask_bit_count(mask);
}

pos_route_eval_result_t pos_evaluate_target_plan(train_pos_t *pos,
                                                 track_node *user_target,
                                                 pos_route_eval_t *out) {
    planner_env_t env;
    planner_train_view_t view;
    planner_eval_t eval;
    planner_route_eval_result_t result;

    if (!pos || !pos->cur_sensor || !user_target) {
        if (out) *out = (pos_route_eval_t){0};
        return POS_ROUTE_EVAL_UNREACHABLE;
    }

    pos_planner_build_env(&env, g_pos_planner_owners, g_pos_planner_switch_state);
    pos_planner_fill_view(pos, &view);
    result = planner_evaluate_target_plan(&env, &view, user_target,
                                          &g_pos_route_ws, &eval);
    if (out) pos_planner_eval_to_pos(&eval, out);
    return (pos_route_eval_result_t)result;
}

pos_route_eval_result_t pos_evaluate_target_ready_now(train_pos_t *pos,
                                                      track_node *user_target,
                                                      pos_route_eval_t *out) {
    planner_env_t env;
    planner_train_view_t view;
    planner_eval_t eval;
    planner_route_eval_result_t result;

    if (!pos || !pos->cur_sensor || !user_target) {
        if (out) *out = (pos_route_eval_t){0};
        return POS_ROUTE_EVAL_UNREACHABLE;
    }

    pos_planner_build_env(&env, g_pos_planner_owners, g_pos_planner_switch_state);
    pos_planner_fill_view(pos, &view);
    result = planner_evaluate_target_ready_now(
        &env, &view, user_target, &g_pos_route_ws, out ? &eval : &g_pos_eval_ready);
    if (out) pos_planner_eval_to_pos(&eval, out);
    return (pos_route_eval_result_t)result;
}
