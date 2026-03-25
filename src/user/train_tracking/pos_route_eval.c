#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include <stddef.h>
#include <stdint.h>

static uint8_t      g_pos_try_blocked[TRACK_MAX];
static char         g_pos_try_fixed_sw_dirs[TRACK_MAX];
static route_plan_t g_pos_try_rp;
static route_plan_t g_pos_try_rp_unconstrained;
static route_plan_t g_pos_try_rp_temp;
static route_plan_t g_pos_try_best_blocked_plan;
static pos_route_eval_t g_pos_try_eval_ready;

static int route_switch_needs_change(int sw_num, char desired_dir) {
    int sw_idx = track_switch_to_index(sw_num);
    if (sw_idx < 0) return 1;
    return track_get_switch_state()[sw_idx].state != desired_dir;
}

static uint8_t blocker_mask_from_trains(int requester_train, const int *trains, int count) {
    uint8_t mask = 0;
    for (int i = 0; i < count; i++) {
        if (trains[i] == requester_train) continue;
        mask |= pos_deadlock_train_bit(trains[i]);
    }
    return mask;
}

uint8_t pos_route_blocker_mask_from_plan(int requester_train,
                                         const route_plan_t *plan) {
    int blockers[6];
    int count = traffic_collect_plan_blockers(requester_train, plan, blockers, 6);
    return blocker_mask_from_trains(requester_train, blockers, count);
}

uint8_t pos_route_blocker_mask_from_switches(const int *sw_nums,
                                             const char *sw_dirs,
                                             int sw_count,
                                             int requester_train) {
    uint8_t mask = 0;

    for (int i = sw_count - 1; i >= 0; i--) {
        int blockers[6];
        int count;

        if (!route_switch_needs_change(sw_nums[i], sw_dirs[i])) continue;
        count = traffic_collect_switch_envelope_blockers(sw_nums[i], blockers, 6);
        mask |= blocker_mask_from_trains(requester_train, blockers, count);
    }
    return mask;
}

int pos_route_blocker_mask_bit_count(uint8_t mask) {
    int count = 0;
    while (mask) {
        count += (mask & 1u);
        mask >>= 1;
    }
    return count;
}

static int route_plan_long_enough(const route_plan_t *plan, int32_t threshold) {
    int32_t effective_d;

    if (!plan) return 0;
    effective_d = plan->has_reversal
                  ? plan->dist_to_reversal_mm + plan->dist_after_reversal_mm
                  : plan->total_dist_mm;
    return effective_d > threshold;
}

static void pos_note_blocked_plan(const route_plan_t *cand,
                                  route_plan_t *best_blocked_plan,
                                  int *have_blocked_plan) {
    if (!cand || !best_blocked_plan || !have_blocked_plan) return;
    if (!*have_blocked_plan || cand->total_dist_mm < best_blocked_plan->total_dist_mm) {
        *best_blocked_plan = *cand;
        *have_blocked_plan = 1;
    }
}

track_node *pos_route_current_goal(train_pos_t *pos) {
    if (!pos) return NULL;
    if (pos->orig_user_target) return pos->orig_user_target;
    if (pos->target_sensor) return pos->target_sensor;
    return pos->pending_target;
}

static int pos_route_use_cur_reverse_start(const train_pos_t *pos) {
    return pos != NULL &&
           (pos->route_state == TRAIN_STATE_STOPPED ||
            pos->route_state == TRAIN_STATE_WAIT_RESOURCE) &&
           pos->stopped_on_target_hit;
}

static void pos_build_fixed_switch_dirs(int requester_train,
                                        char fixed_sw_dirs[TRACK_MAX]) {
    for (int i = 0; i < TRACK_MAX; i++) fixed_sw_dirs[i] = '?';

    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        int sw_idx;
        char current_dir;

        if (n->type != NODE_BRANCH) continue;

        sw_idx = track_switch_to_index(n->num);
        if (sw_idx < 0) continue;

        current_dir = track_get_switch_state()[sw_idx].state;
        if (current_dir != 'S' && current_dir != 'C') continue;

        if (traffic_can_set_switch(n->num, requester_train) == requester_train) {
            fixed_sw_dirs[i] = current_dir;
        }
    }
}

static pos_route_eval_result_t pos_evaluate_target(train_pos_t *pos,
                                                   track_node *user_target,
                                                   pos_route_eval_t *out) {
    route_plan_t *rp = &g_pos_try_rp;
    route_plan_t *rp_unconstrained = &g_pos_try_rp_unconstrained;
    route_plan_t *rp_temp = &g_pos_try_rp_temp;
    route_plan_t *best_blocked_plan = &g_pos_try_best_blocked_plan;
    track_node *cur_sensor_orig;
    track_node *plan_start;
    track_node *reverse_plan_start;
    track_node *origins[2];
    uint8_t *blocked = g_pos_try_blocked;
    char *fixed_sw_dirs = g_pos_try_fixed_sw_dirs;
    track_node *chosen_origin = NULL;
    int have_blocked_plan = 0;
    int32_t best_total = 0;
    int need_initial_reverse = 0;
    int32_t tv;
    int32_t ta;
    int32_t d_brake;
    int32_t d_stop;
    int32_t threshold;

    if (out) {
        out->plan = (route_plan_t){0};
        out->chosen_origin = NULL;
        out->need_initial_reverse = 0;
        out->blocker_mask = 0;
    }
    if (!pos || !pos->cur_sensor || !user_target) return POS_ROUTE_EVAL_UNREACHABLE;

    cur_sensor_orig = pos->cur_sensor;
    plan_start = pos->pred.next_sensor;
    if (!plan_start) {
        uint64_t dt_ignored = 0;
        plan_start = predict_next_sensor(pos, pos->cur_sensor, &dt_ignored);
    }
    reverse_plan_start = pos_route_use_cur_reverse_start(pos)
                         ? cur_sensor_orig->reverse
                         : (plan_start ? plan_start->reverse : cur_sensor_orig->reverse);
    origins[0] = plan_start;
    origins[1] = reverse_plan_start;

    tv        = speed_table_get_v(pos->train_ind, GOTO_USER_SPEED);
    ta        = speed_table_get_nominal_decel(pos->train_ind, GOTO_USER_SPEED);
    d_brake   = tv * tv / (2 * ta);
    d_stop    = d_brake + (int32_t)((int64_t)tv * (int64_t)STOP_EARLY_US[pos->train_ind]
                                    / 1000000LL);
    threshold = GOTO_MIN_DIST_FACTOR * d_stop;

    traffic_build_constraints(pos->train_num, blocked);
    pos_build_fixed_switch_dirs(pos->train_num, fixed_sw_dirs);

    for (int o = 0; o < 2; o++) {
        if (!origins[o]) continue;
        if (!bfs_find_route_optimal_constrained(origins[o], user_target, d_stop,
                                                blocked, fixed_sw_dirs, rp_temp)) {
            if (bfs_find_route_optimal_constrained(origins[o], user_target, d_stop,
                                                   NULL, fixed_sw_dirs,
                                                   rp_unconstrained) &&
                route_plan_long_enough(rp_unconstrained, threshold)) {
                pos_note_blocked_plan(rp_unconstrained, best_blocked_plan,
                                      &have_blocked_plan);
            }
            continue;
        }

        if (!route_plan_long_enough(rp_temp, threshold)) continue;
        if (chosen_origin == NULL || rp_temp->total_dist_mm < best_total) {
            *rp                  = *rp_temp;
            chosen_origin        = origins[o];
            best_total           = rp_temp->total_dist_mm;
            need_initial_reverse = (o == 1);
        }
    }

    if (!chosen_origin) {
        track_node *boot_start = reverse_plan_start;
        int allow_bootstrap = !have_blocked_plan ||
                              pos->route_state == TRAIN_STATE_WAIT_RESOURCE;
        if (boot_start && allow_bootstrap &&
            bfs_find_bootstrap_midrev(boot_start, user_target, d_stop,
                                      blocked, fixed_sw_dirs, rp)) {
            chosen_origin = boot_start;
            need_initial_reverse = 1;
        } else if (boot_start && allow_bootstrap &&
                   bfs_find_bootstrap_midrev(boot_start, user_target, d_stop,
                                             NULL, fixed_sw_dirs,
                                             rp_unconstrained)) {
            pos_note_blocked_plan(rp_unconstrained, best_blocked_plan,
                                  &have_blocked_plan);
        }
    }

    if (!chosen_origin) {
        if (out && have_blocked_plan) {
            out->blocker_mask = pos_route_blocker_mask_from_plan(pos->train_num,
                                                                 best_blocked_plan);
        }
        return have_blocked_plan ? POS_ROUTE_EVAL_BLOCKED
                                 : POS_ROUTE_EVAL_UNREACHABLE;
    }

    if (out) {
        out->plan = *rp;
        out->chosen_origin = chosen_origin;
        out->need_initial_reverse = need_initial_reverse;
        out->blocker_mask = 0;
    }
    return POS_ROUTE_EVAL_READY;
}

pos_route_eval_result_t pos_evaluate_target_ready_now(train_pos_t *pos,
                                                      track_node *user_target,
                                                      pos_route_eval_t *out) {
    pos_route_eval_t *eval = out ? out : &g_pos_try_eval_ready;
    route_plan_t reserve_plan;
    pos_route_eval_result_t result = pos_evaluate_target(pos, user_target, eval);

    if (result != POS_ROUTE_EVAL_READY) return result;

    reserve_plan = eval->plan;
    if (reserve_plan.has_reversal) {
        reserve_plan.path_count2 = 0;
    }

    if (!traffic_can_reserve_plan(pos->train_num, &reserve_plan)) {
        eval->blocker_mask = pos_route_blocker_mask_from_plan(pos->train_num,
                                                              &reserve_plan);
        return POS_ROUTE_EVAL_BLOCKED;
    }

    if (pos_route_switch_blocker(eval->plan.sw_nums, eval->plan.sw_dirs,
                                 eval->plan.sw_count, pos->train_num) >= 0) {
        eval->blocker_mask = pos_route_blocker_mask_from_switches(eval->plan.sw_nums,
                                                                  eval->plan.sw_dirs,
                                                                  eval->plan.sw_count,
                                                                  pos->train_num);
        return POS_ROUTE_EVAL_BLOCKED;
    }

    eval->blocker_mask = 0;
    return POS_ROUTE_EVAL_READY;
}
