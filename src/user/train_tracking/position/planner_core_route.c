#include "train_tracking/planner_core.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "train_tracking/speed_table.h"
#include "../traffic/traffic_window_internal.h"
#include "track.h"
#include <stddef.h>
#include <stdint.h>

static int planner_train_to_index(int train_num) {
    switch (train_num) {
    case 13: return 0;
    case 14: return 1;
    case 15: return 2;
    case 17: return 3;
    case 18: return 4;
    case 55: return 5;
    default: return -1;
    }
}

static int planner_reverse_index(int idx) {
    track_node *rev;

    if (idx < 0 || idx >= TRACK_MAX) return -1;
    rev = g_track[idx].reverse;
    if (!rev) return -1;
    idx = (int)(rev - g_track);
    return (idx >= 0 && idx < TRACK_MAX) ? idx : -1;
}

static int32_t planner_early_stop_mm(const planner_train_view_t *view) {
    int32_t v;

    if (!view) return 0;
    v = speed_table_get_v(view->train_ind, view->goto_speed);
    if (v <= 0) return 0;
    return (int32_t)((int64_t)v *
                     (int64_t)speed_table_get_early_stop(view->train_ind,
                                                         view->goto_speed) /
                     1000000LL);
}

static int32_t planner_brake_dist_mm(const planner_train_view_t *view) {
    int32_t v;
    int32_t a;

    if (!view) return 0;
    v = speed_table_get_v(view->train_ind, view->goto_speed);
    a = speed_table_get_nominal_decel(view->train_ind, view->goto_speed);
    if (v <= 0 || a <= 0) return 0;
    return v * v / (2 * a);
}

int32_t planner_view_stop_dist_mm(const planner_train_view_t *view) {
    int32_t brake = planner_brake_dist_mm(view);
    int32_t early = planner_early_stop_mm(view);

    if (brake < 0) brake = 0;
    if (early < 0) early = 0;
    return brake + early;
}

int32_t planner_view_min_window_mm(const planner_train_view_t *view) {
    int32_t brake = planner_brake_dist_mm(view);
    int32_t early = planner_early_stop_mm(view);

    if (brake < 0) brake = 0;
    if (early < 0) early = 0;
    return GOTO_MIN_DIST_FACTOR * brake + early;
}

static int planner_route_switch_needs_change(const planner_env_t *env,
                                             int sw_num,
                                             char desired_dir) {
    int sw_idx;

    if (!env || !env->switch_state) return 1;
    sw_idx = track_switch_to_index(sw_num);
    if (sw_idx < 0) return 1;
    return env->switch_state[sw_idx] != desired_dir;
}

static int planner_switch_envelope_owner(const int owners[TRACK_MAX], int sw_num) {
    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        int checks[6];
        int c = 0;

        if (n->type != NODE_BRANCH || n->num != sw_num) continue;

        checks[c++] = i;
        checks[c++] = planner_reverse_index(i);
        if (n->edge[DIR_STRAIGHT].dest) {
            int idx = (int)(n->edge[DIR_STRAIGHT].dest - g_track);
            checks[c++] = idx;
            checks[c++] = planner_reverse_index(idx);
        }
        if (n->edge[DIR_CURVED].dest) {
            int idx = (int)(n->edge[DIR_CURVED].dest - g_track);
            checks[c++] = idx;
            checks[c++] = planner_reverse_index(idx);
        }

        for (int j = 0; j < c; j++) {
            int idx = checks[j];
            if (idx < 0 || idx >= TRACK_MAX) continue;
            if (owners[idx] >= 0) return owners[idx];
        }
        break;
    }
    return -1;
}

static void planner_build_constraints_for_train(const planner_env_t *env,
                                                int requester_train,
                                                uint8_t blocked[TRACK_MAX],
                                                char fixed_sw_dirs[TRACK_MAX]) {
    if (blocked) {
        for (int i = 0; i < TRACK_MAX; i++) {
            int owner = env->reservation_owner[i];
            blocked[i] = (owner >= 0 && owner != requester_train) ? 1 : 0;
        }
        traffic_expand_zone_marks(blocked);
    }

    if (fixed_sw_dirs) {
        for (int i = 0; i < TRACK_MAX; i++) fixed_sw_dirs[i] = '?';
        for (int i = 0; i < TRACK_MAX; i++) {
            track_node *n = &g_track[i];
            int sw_idx;
            char current_dir;

            if (n->type != NODE_BRANCH) continue;
            sw_idx = track_switch_to_index(n->num);
            if (sw_idx < 0) continue;
            current_dir = env->switch_state[sw_idx];
            if (current_dir != 'S' && current_dir != 'C') continue;
            if (planner_switch_envelope_owner(env->reservation_owner, n->num) ==
                requester_train) {
                fixed_sw_dirs[i] = current_dir;
            }
        }
    }
}

static int planner_plan_has_conflict(const int owners[TRACK_MAX],
                                     int train_num,
                                     const uint8_t want[TRACK_MAX]) {
    for (int i = 0; i < TRACK_MAX; i++) {
        if (!want[i]) continue;
        if (owners[i] >= 0 && owners[i] != train_num) return 1;
    }
    return 0;
}

static int planner_can_reserve_plan(const planner_env_t *env,
                                    int train_num,
                                    const route_plan_t *plan) {
    uint8_t want[TRACK_MAX];

    if (!env || !env->reservation_owner || !plan || train_num < 0) return 0;
    traffic_build_plan_marks_copy(plan, want);
    return !planner_plan_has_conflict(env->reservation_owner, train_num, want);
}

static uint8_t planner_blocker_mask_from_trains(int requester_train,
                                                const int *trains,
                                                int count) {
    uint8_t mask = 0;

    for (int i = 0; i < count; i++) {
        if (trains[i] == requester_train) continue;
        mask |= planner_train_bit(trains[i]);
    }
    return mask;
}

static int planner_collect_plan_blockers(const planner_env_t *env,
                                         int requester_train,
                                         const route_plan_t *plan,
                                         int *out_trains,
                                         int max_trains) {
    uint8_t want[TRACK_MAX];
    int unique[6];
    int total = 0;

    if (!env || !env->reservation_owner || !plan) return 0;

    traffic_build_plan_marks_copy(plan, want);
    for (int i = 0; i < TRACK_MAX; i++) {
        int owner;
        int seen = 0;

        if (!want[i]) continue;
        owner = env->reservation_owner[i];
        if (owner < 0 || owner == requester_train) continue;
        for (int j = 0; j < total; j++) {
            if (unique[j] == owner) {
                seen = 1;
                break;
            }
        }
        if (seen) continue;
        if (total < 6) unique[total] = owner;
        total++;
    }

    for (int i = 0; out_trains && i < total && i < max_trains; i++) {
        out_trains[i] = unique[i];
    }
    return total;
}

static int planner_collect_switch_blockers(const planner_env_t *env,
                                           int sw_num,
                                           int *out_trains,
                                           int max_trains) {
    int unique[6];
    int total = 0;

    if (!env || !env->reservation_owner) return 0;

    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        int checks[6];
        int c = 0;

        if (n->type != NODE_BRANCH || n->num != sw_num) continue;

        checks[c++] = i;
        checks[c++] = planner_reverse_index(i);
        if (n->edge[DIR_STRAIGHT].dest) {
            int idx = (int)(n->edge[DIR_STRAIGHT].dest - g_track);
            checks[c++] = idx;
            checks[c++] = planner_reverse_index(idx);
        }
        if (n->edge[DIR_CURVED].dest) {
            int idx = (int)(n->edge[DIR_CURVED].dest - g_track);
            checks[c++] = idx;
            checks[c++] = planner_reverse_index(idx);
        }

        for (int j = 0; j < c; j++) {
            int idx = checks[j];
            int owner;
            int seen = 0;

            if (idx < 0 || idx >= TRACK_MAX) continue;
            owner = env->reservation_owner[idx];
            if (owner < 0) continue;
            for (int k = 0; k < total; k++) {
                if (unique[k] == owner) {
                    seen = 1;
                    break;
                }
            }
            if (seen) continue;
            if (total < 6) unique[total] = owner;
            total++;
        }
        break;
    }

    for (int i = 0; out_trains && i < total && i < max_trains; i++) {
        out_trains[i] = unique[i];
    }
    return total;
}

static int planner_route_switch_blocker(const planner_env_t *env,
                                        const int *sw_nums,
                                        const char *sw_dirs,
                                        int sw_count,
                                        int requester_train) {
    (void)requester_train;
    for (int i = sw_count - 1; i >= 0; i--) {
        if (!planner_route_switch_needs_change(env, sw_nums[i], sw_dirs[i])) {
            continue;
        }
        if (planner_switch_envelope_owner(env->reservation_owner, sw_nums[i]) >= 0) {
            return planner_switch_envelope_owner(env->reservation_owner, sw_nums[i]);
        }
    }
    return -1;
}

static void planner_note_best_plan(const route_plan_t *cand,
                                   track_node *cand_origin,
                                   int cand_need_initial_reverse,
                                   route_plan_t *best_plan,
                                   track_node **best_origin,
                                   int *best_need_initial_reverse,
                                   int *have_best_plan) {
    if (!cand || !best_plan || !best_origin ||
        !best_need_initial_reverse || !have_best_plan) {
        return;
    }
    if (!*have_best_plan || cand->total_dist_mm < best_plan->total_dist_mm) {
        *best_plan = *cand;
        *best_origin = cand_origin;
        *best_need_initial_reverse = cand_need_initial_reverse;
        *have_best_plan = 1;
    }
}

static planner_route_eval_result_t planner_evaluate_target_plan_internal(
    const planner_env_t *env, const planner_train_view_t *view,
    track_node *user_target, int allow_blocked_fallback,
    planner_workspace_t *ws, planner_eval_t *out) {
    track_node *chosen_origin = NULL;
    track_node *blocked_origin = NULL;
    int need_initial_reverse = 0;
    int blocked_need_initial_reverse = 0;
    int32_t stop_dist_mm;
    int32_t min_window_mm;

    if (out) {
        *out = (planner_eval_t){0};
    }
    if (!env || !env->reservation_owner || !env->switch_state ||
        !view || !view->cur_sensor || !user_target || !ws) {
        return PLANNER_ROUTE_EVAL_UNREACHABLE;
    }

    stop_dist_mm = planner_view_stop_dist_mm(view);
    min_window_mm = planner_view_min_window_mm(view);
    if (stop_dist_mm <= 0 || min_window_mm <= 0) {
        return PLANNER_ROUTE_EVAL_UNREACHABLE;
    }

    planner_build_constraints_for_train(env, view->train_num, ws->blocked,
                                        ws->fixed_sw_dirs);

    if (planner_select_best_route_for_origins(view->origins, user_target,
                                              stop_dist_mm, min_window_mm,
                                              ws->blocked, ws->fixed_sw_dirs, ws,
                                              &ws->route_best_plan,
                                              &chosen_origin,
                                              &need_initial_reverse)) {
        if (out) {
            out->plan = ws->route_best_plan;
            out->chosen_origin = chosen_origin;
            out->need_initial_reverse = need_initial_reverse;
            out->blocker_mask = 0;
        }
        return PLANNER_ROUTE_EVAL_READY;
    }

    if (!planner_select_best_route_for_origins(view->origins, user_target,
                                               stop_dist_mm, min_window_mm,
                                               NULL, ws->fixed_sw_dirs, ws,
                                               &ws->route_blocked_plan,
                                               &blocked_origin,
                                               &blocked_need_initial_reverse)) {
        return PLANNER_ROUTE_EVAL_UNREACHABLE;
    }

    if (allow_blocked_fallback) {
        if (out) {
            out->plan = ws->route_blocked_plan;
            out->chosen_origin = blocked_origin;
            out->need_initial_reverse = blocked_need_initial_reverse;
            out->blocker_mask = 0;
        }
        return PLANNER_ROUTE_EVAL_READY;
    }

    if (out) {
        out->plan = ws->route_blocked_plan;
        out->chosen_origin = blocked_origin;
        out->need_initial_reverse = blocked_need_initial_reverse;
        out->blocker_mask =
            planner_route_blocker_mask_from_plan(env, view->train_num,
                                                 &ws->route_blocked_plan) |
            planner_route_blocker_mask_from_switches(
                env, ws->route_blocked_plan.sw_nums,
                ws->route_blocked_plan.sw_dirs,
                ws->route_blocked_plan.sw_count, view->train_num);
    }
    return PLANNER_ROUTE_EVAL_BLOCKED;
}

static int32_t planner_authority_path_dist(const uint16_t *path,
                                           int start_cursor,
                                           int end_cursor) {
    if (!path) return -1;
    if (end_cursor < start_cursor) return 0;
    return route_path_dist_from(path, start_cursor, end_cursor + 1);
}

uint8_t planner_train_bit(int train_num) {
    int idx = planner_train_to_index(train_num);
    return (idx >= 0) ? (uint8_t)(1u << idx) : 0;
}

int planner_node_index(const track_node *node) {
    int idx;

    if (!node) return -1;
    idx = (int)(node - g_track);
    return (idx >= 0 && idx < TRACK_MAX) ? idx : -1;
}

track_node *planner_node_from_index(int idx) {
    if (idx < 0 || idx >= TRACK_MAX) return NULL;
    return &g_track[idx];
}

int planner_route_plan_long_enough(const route_plan_t *plan,
                                   int32_t threshold) {
    int32_t effective_d;

    if (!plan) return 0;
    effective_d = plan->has_reversal
                  ? plan->dist_to_reversal_mm + plan->dist_after_reversal_mm
                  : plan->total_dist_mm;
    return effective_d > threshold;
}

int planner_route_blocker_mask_bit_count(uint8_t mask) {
    int count = 0;

    while (mask) {
        count += (mask & 1u);
        mask >>= 1;
    }
    return count;
}

int planner_same_physical_sensor(const track_node *a, const track_node *b) {
    if (!a || !b) return 0;
    return a == b || a->reverse == b || b->reverse == a;
}

int planner_candidate_in_yield_history(const planner_train_view_t *view,
                                       const track_node *cand) {
    if (!view || !cand) return 0;

    for (int i = 0; i < view->yield_history_count &&
                    i < DEADLOCK_YIELD_HISTORY_MAX; i++) {
        if (planner_same_physical_sensor(view->yield_history[i], cand)) {
            return 1;
        }
    }
    return 0;
}

int planner_select_best_route_for_origins(track_node *const origins[2],
                                          track_node *user_target,
                                          int32_t stop_dist_mm,
                                          int32_t min_window_mm,
                                          const uint8_t *blocked,
                                          const char *fixed_sw_dirs,
                                          planner_workspace_t *ws,
                                          route_plan_t *out_plan,
                                          track_node **out_origin,
                                          int *out_need_initial_reverse) {
    int have_best_plan = 0;
    track_node *best_origin = NULL;
    int best_need_initial_reverse = 0;

    if (out_plan) *out_plan = (route_plan_t){0};
    if (out_origin) *out_origin = NULL;
    if (out_need_initial_reverse) *out_need_initial_reverse = 0;
    if (!origins || !user_target || !ws) return 0;

    for (int o = 0; o < 2; o++) {
        if (!origins[o]) continue;

        if (bfs_find_route_optimal_constrained(origins[o], user_target,
                                               stop_dist_mm, blocked,
                                               fixed_sw_dirs,
                                               &ws->route_temp_plan) &&
            planner_route_plan_long_enough(&ws->route_temp_plan,
                                           min_window_mm)) {
            planner_note_best_plan(&ws->route_temp_plan, origins[o], o == 1,
                                   &ws->route_best_plan, &best_origin,
                                   &best_need_initial_reverse, &have_best_plan);
        }
    }

    if (!have_best_plan && origins[1] &&
        bfs_find_bootstrap_midrev(origins[1], user_target, stop_dist_mm,
                                  blocked, fixed_sw_dirs,
                                  &ws->route_temp_plan) &&
        planner_route_plan_long_enough(&ws->route_temp_plan, min_window_mm)) {
        planner_note_best_plan(&ws->route_temp_plan, origins[1], 1,
                               &ws->route_best_plan, &best_origin,
                               &best_need_initial_reverse, &have_best_plan);
    }

    if (!have_best_plan) return 0;

    if (out_plan) *out_plan = ws->route_best_plan;
    if (out_origin) *out_origin = best_origin;
    if (out_need_initial_reverse) {
        *out_need_initial_reverse = best_need_initial_reverse;
    }
    return 1;
}

uint8_t planner_route_blocker_mask_from_plan(const planner_env_t *env,
                                             int requester_train,
                                             const route_plan_t *plan) {
    int blockers[6];
    int count = planner_collect_plan_blockers(env, requester_train, plan,
                                              blockers, 6);
    return planner_blocker_mask_from_trains(requester_train, blockers, count);
}

uint8_t planner_route_blocker_mask_from_switches(const planner_env_t *env,
                                                 const int *sw_nums,
                                                 const char *sw_dirs,
                                                 int sw_count,
                                                 int requester_train) {
    uint8_t mask = 0;

    if (!env) return 0;
    for (int i = sw_count - 1; i >= 0; i--) {
        int blockers[6];
        int count;

        if (!planner_route_switch_needs_change(env, sw_nums[i], sw_dirs[i])) {
            continue;
        }
        count = planner_collect_switch_blockers(env, sw_nums[i], blockers, 6);
        mask |= planner_blocker_mask_from_trains(requester_train, blockers,
                                                 count);
    }
    return mask;
}

planner_route_eval_result_t planner_evaluate_target_plan(
    const planner_env_t *env, const planner_train_view_t *view,
    track_node *user_target, planner_workspace_t *ws, planner_eval_t *out) {
    return planner_evaluate_target_plan_internal(env, view, user_target, 1, ws,
                                                 out);
}

planner_route_eval_result_t planner_evaluate_target_ready_now(
    const planner_env_t *env, const planner_train_view_t *view,
    track_node *user_target, planner_workspace_t *ws, planner_eval_t *out) {
    planner_eval_t local_eval;
    planner_eval_t *eval = out ? out : &local_eval;
    route_plan_t reserve_plan;
    planner_route_eval_result_t result =
        planner_evaluate_target_plan_internal(env, view, user_target, 0, ws,
                                              eval);

    if (result != PLANNER_ROUTE_EVAL_READY) return result;

    reserve_plan = eval->plan;
    if (reserve_plan.has_reversal) reserve_plan.path_count2 = 0;

    if (!planner_can_reserve_plan(env, view->train_num, &reserve_plan)) {
        eval->blocker_mask = planner_route_blocker_mask_from_plan(
            env, view->train_num, &reserve_plan);
        return PLANNER_ROUTE_EVAL_BLOCKED;
    }

    if (planner_route_switch_blocker(env, eval->plan.sw_nums, eval->plan.sw_dirs,
                                     eval->plan.sw_count, view->train_num) >= 0) {
        eval->blocker_mask = planner_route_blocker_mask_from_switches(
            env, eval->plan.sw_nums, eval->plan.sw_dirs, eval->plan.sw_count,
            view->train_num);
        return PLANNER_ROUTE_EVAL_BLOCKED;
    }

    eval->blocker_mask = 0;
    return PLANNER_ROUTE_EVAL_READY;
}

int planner_prepare_launch_prefix(const planner_env_t *env,
                                  const planner_train_view_t *view,
                                  const route_plan_t *full_plan,
                                  int path_prefix_start_cursor,
                                  int path_dist_start_cursor,
                                  int allow_short_goal,
                                  int min_end_cursor,
                                  planner_workspace_t *ws,
                                  route_plan_t *out_prefix,
                                  int *out_reserved_end_cursor,
                                  int *out_switch_blocker_owner) {
    int have_short_goal = 0;
    int short_goal_end_cursor = -1;
    int32_t min_window_mm;
    int32_t stop_dist_mm;

    if (!env || !view || !full_plan || !ws || !out_prefix ||
        !out_reserved_end_cursor) {
        return 0;
    }
    if (path_prefix_start_cursor < 0 ||
        path_prefix_start_cursor >= full_plan->path_count) {
        return 0;
    }
    if (path_dist_start_cursor < 0 ||
        path_dist_start_cursor >= full_plan->path_count) {
        return 0;
    }
    if (path_dist_start_cursor < path_prefix_start_cursor) {
        path_dist_start_cursor = path_prefix_start_cursor;
    }

    min_window_mm = planner_view_min_window_mm(view);
    stop_dist_mm = planner_view_stop_dist_mm(view);
    if (min_window_mm <= 0 || stop_dist_mm <= 0) return 0;
    if (out_switch_blocker_owner) *out_switch_blocker_owner = -1;

    for (int end_cursor = path_prefix_start_cursor;
         end_cursor < full_plan->path_count; end_cursor++) {
        int32_t dist_mm;
        int switch_blocker;

        if (!traffic_window_build_prefix_plan(full_plan->path_nodes,
                                              full_plan->path_count,
                                              path_prefix_start_cursor,
                                              end_cursor,
                                              &ws->authority_candidate_prefix)) {
            break;
        }

        dist_mm = planner_authority_path_dist(full_plan->path_nodes,
                                              path_dist_start_cursor,
                                              end_cursor);
        if (dist_mm < 0) break;
        if (dist_mm <= 0) continue;

        if (end_cursor != full_plan->path_count - 1 &&
            g_track[full_plan->path_nodes[end_cursor]].type != NODE_SENSOR) {
            continue;
        }

        switch_blocker = planner_route_switch_blocker(
            env, ws->authority_candidate_prefix.sw_nums,
            ws->authority_candidate_prefix.sw_dirs,
            ws->authority_candidate_prefix.sw_count, view->train_num);
        if (switch_blocker >= 0) {
            if (out_switch_blocker_owner) {
                *out_switch_blocker_owner = switch_blocker;
            }
            break;
        }
        if (!planner_can_reserve_plan(env, view->train_num,
                                      &ws->authority_candidate_prefix)) {
            break;
        }

        if (allow_short_goal &&
            end_cursor == full_plan->path_count - 1 &&
            dist_mm >= stop_dist_mm &&
            end_cursor > min_end_cursor) {
            ws->authority_short_goal_prefix = ws->authority_candidate_prefix;
            short_goal_end_cursor = end_cursor;
            have_short_goal = 1;
        }

        if (dist_mm < min_window_mm) continue;
        if (end_cursor <= min_end_cursor) continue;

        *out_prefix = ws->authority_candidate_prefix;
        *out_reserved_end_cursor = end_cursor;
        return 1;
    }

    if (have_short_goal) {
        *out_prefix = ws->authority_short_goal_prefix;
        *out_reserved_end_cursor = short_goal_end_cursor;
        return 1;
    }

    return 0;
}

int planner_prepare_launch_strict(const planner_env_t *env,
                                  const planner_train_view_t *view,
                                  const route_plan_t *full_plan,
                                  int path_prefix_start_cursor,
                                  int path_dist_start_cursor,
                                  planner_workspace_t *ws,
                                  route_plan_t *out_prefix,
                                  int *out_reserved_end_cursor,
                                  int *out_switch_blocker_owner) {
    return planner_prepare_launch_prefix(
        env, view, full_plan, path_prefix_start_cursor, path_dist_start_cursor,
        0, -1, ws, out_prefix, out_reserved_end_cursor,
        out_switch_blocker_owner);
}
