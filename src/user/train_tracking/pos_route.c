#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "train_tracking/speed_table.h"
#include "timer.h"
#include "ui.h"
#include "util.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX_SENSORS 80

static uint8_t      g_pos_try_blocked[TRACK_MAX];
static char         g_pos_try_fixed_sw_dirs[TRACK_MAX];
static route_plan_t g_pos_try_rp;
static route_plan_t g_pos_try_rp_unconstrained;
static route_plan_t g_pos_try_rp_temp;
static route_plan_t g_pos_try_best_blocked_plan;
static route_plan_t g_pos_try_candidate_plan;
static route_plan_t g_pos_try_reserve_plan;
static int          g_pos_try_snapshot[TRACK_MAX];

#define DEADLOCK_YIELD_MAX_CANDIDATES 8

typedef struct {
    track_node *target;
    int32_t     total_dist_mm;
} pos_deadlock_yield_candidate_t;

static pos_deadlock_yield_candidate_t
    g_pos_try_deadlock_candidates[DEADLOCK_YIELD_MAX_CANDIDATES];

typedef enum {
    POS_ROUTE_EVAL_UNREACHABLE = 0,
    POS_ROUTE_EVAL_BLOCKED     = 1,
    POS_ROUTE_EVAL_READY       = 2,
} pos_route_eval_result_t;

typedef struct {
    route_plan_t plan;
    track_node   *chosen_origin;
    int          need_initial_reverse;
    uint8_t      blocker_mask;
} pos_route_eval_t;

static pos_route_eval_t g_pos_try_eval_main;
static pos_route_eval_t g_pos_try_eval_candidate;
static pos_route_eval_t g_pos_try_eval_ready;

#ifdef TRACK_D
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {227, 232, 242, 229, 230};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {144, 144, 144, 144, 144};

    static const int32_t GOTO_DECEL_OVERRIDE[MAX_SENSORS] =
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#else
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {226, 224, 226, 222, 236};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {167, 167, 167, 167, 167};

    static const int32_t GOTO_DECEL_OVERRIDE[MAX_SENSORS] =
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#endif

int32_t speed_table_get_v(int32_t train_ind, int user_speed) {
    if (user_speed != GOTO_USER_SPEED) return 0;
    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;
    return GOTO_SPEED_MM_S[train_ind];
}

int32_t speed_table_get_decel(int32_t train_ind, int user_speed, track_node *target) {
    if (user_speed != GOTO_USER_SPEED) return 0;
    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;

    if (target->type == NODE_SENSOR) {
        int32_t override = GOTO_DECEL_OVERRIDE[target->num];
        return (override > -1) ? override : GOTO_DECEL_MM_S2[train_ind];
    }
    return GOTO_DECEL_MM_S2[train_ind];
}

static int route_switch_needs_change(int sw_num, char desired_dir) {
    int sw_idx = track_switch_to_index(sw_num);
    if (sw_idx < 0) return 1;
    char current_dir = track_get_switch_state()[sw_idx].state;
    return current_dir != desired_dir;
}

static uint8_t blocker_mask_from_trains(int requester_train, const int *trains, int count) {
    uint8_t mask = 0;
    for (int i = 0; i < count; i++) {
        if (trains[i] == requester_train) continue;
        mask |= pos_deadlock_train_bit(trains[i]);
    }
    return mask;
}

static uint8_t blocker_mask_from_plan(int requester_train, const route_plan_t *plan) {
    int blockers[6];
    int count = traffic_collect_plan_blockers(requester_train, plan, blockers, 6);
    return blocker_mask_from_trains(requester_train, blockers, count);
}

static uint8_t blocker_mask_from_switches(const int *sw_nums, const char *sw_dirs,
                                          int sw_count, int requester_train) {
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

static int blocker_mask_bit_count(uint8_t mask) {
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

static track_node *pos_route_current_goal(train_pos_t *pos) {
    if (!pos) return NULL;
    if (pos->orig_user_target) return pos->orig_user_target;
    if (pos->target_sensor) return pos->target_sensor;
    return pos->pending_target;
}

static void pos_build_fixed_switch_dirs(int requester_train, char fixed_sw_dirs[TRACK_MAX]) {
    for (int i = 0; i < TRACK_MAX; i++) fixed_sw_dirs[i] = '?';

    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        if (n->type != NODE_BRANCH) continue;

        int sw_idx = track_switch_to_index(n->num);
        if (sw_idx < 0) continue;

        char current_dir = track_get_switch_state()[sw_idx].state;
        if (current_dir != 'S' && current_dir != 'C') continue;

        if (traffic_can_set_switch(n->num, requester_train) == requester_train) {
            fixed_sw_dirs[i] = current_dir;
        }
    }
}

int pos_route_switch_blocker(const int *sw_nums, const char *sw_dirs,
                             int sw_count, int requester_train) {
    for (int i = sw_count - 1; i >= 0; i--) {
        if (!route_switch_needs_change(sw_nums[i], sw_dirs[i])) continue;
        int owner = traffic_can_set_switch(sw_nums[i], requester_train);
        if (owner >= 0) return owner;
    }
    return -1;
}

int pos_apply_route_switches_safe(const int *sw_nums, const char *sw_dirs,
                                  int sw_count, int requester_train) {
    if (pos_route_switch_blocker(sw_nums, sw_dirs, sw_count, requester_train) >= 0) {
        return 0;
    }
    for (int i = sw_count - 1; i >= 0; i--) {
        if (!route_switch_needs_change(sw_nums[i], sw_dirs[i])) continue;
        track_set_switch(sw_nums[i], sw_dirs[i]);
        track_update_switch(sw_nums[i], sw_dirs[i]);
    }
    return 1;
}

static bool pos_is_waiting_resource(train_pos_t *pos) {
    return !pos || pos->route_state == TRAIN_STATE_WAIT_RESOURCE;
}

void pos_enter_wait_resource(train_pos_t *pos, uint64_t now_us, uint8_t blocker_mask) {
    if (!pos) return;
    pos->replan.blocker_mask = blocker_mask;
    if (pos_is_waiting_resource(pos)) {
        ui_mark_position_dirty();
        return;
    }
    track_set_speed(pos->train_num, 0);
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM,
                                    pos_release_keep_end(pos->cur_sensor,
                                                         pos->pred.next_sensor));
    pos->route_state = TRAIN_STATE_WAIT_RESOURCE;
    pos->replan.retry_count = 0;
    pos->replan.next_us = now_us + REPLAN_INTERVAL_US;
    pos->replan.seen_generation = traffic_get_change_generation();
    pos->stopping_since_us = now_us;
    pos->effective_v = 0;
    pos_clear_prediction(pos);
    ui_mark_position_dirty();
}

static pos_route_eval_result_t pos_evaluate_target(train_pos_t *pos,
                                                   track_node *user_target,
                                                   pos_route_eval_t *out) {
    route_plan_t *rp = &g_pos_try_rp;
    route_plan_t *rp_unconstrained = &g_pos_try_rp_unconstrained;
    route_plan_t *rp_temp = &g_pos_try_rp_temp;
    track_node *cur_sensor_orig;
    track_node *plan_start;
    track_node *reverse_plan_start;
    track_node *origins[2];
    uint8_t *blocked = g_pos_try_blocked;
    char *fixed_sw_dirs = g_pos_try_fixed_sw_dirs;
    track_node *chosen_origin = NULL;
    route_plan_t *best_blocked_plan = &g_pos_try_best_blocked_plan;
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
    reverse_plan_start = plan_start ? plan_start->reverse : cur_sensor_orig->reverse;
    origins[0] = plan_start;
    origins[1] = reverse_plan_start;

    tv        = speed_table_get_v(pos->train_ind, GOTO_USER_SPEED);
    ta        = GOTO_DECEL_MM_S2[pos->train_ind];
    d_brake   = tv * tv / (2 * ta);
    d_stop    = d_brake + (int32_t)((int64_t)tv * (int64_t)STOP_EARLY_US[pos->train_ind] / 1000000LL);
    threshold = GOTO_MIN_DIST_FACTOR * d_stop;

    traffic_build_constraints(pos->train_num, blocked);
    pos_build_fixed_switch_dirs(pos->train_num, fixed_sw_dirs);

    for (int o = 0; o < 2; o++) {
        if (!origins[o]) continue;
        if (!bfs_find_route_optimal_constrained(origins[o], user_target, d_stop,
                                                blocked, fixed_sw_dirs, rp_temp)) {
            if (bfs_find_route_optimal_constrained(origins[o], user_target, d_stop,
                                                   NULL, fixed_sw_dirs, rp_unconstrained) &&
                route_plan_long_enough(rp_unconstrained, threshold)) {
                pos_note_blocked_plan(rp_unconstrained, best_blocked_plan, &have_blocked_plan);
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
                                             NULL, fixed_sw_dirs, rp_unconstrained)) {
            pos_note_blocked_plan(rp_unconstrained, best_blocked_plan, &have_blocked_plan);
        }
    }

    if (!chosen_origin) {
        if (out && have_blocked_plan) out->blocker_mask = blocker_mask_from_plan(pos->train_num,
                                                                                  best_blocked_plan);
        return have_blocked_plan ? POS_ROUTE_EVAL_BLOCKED : POS_ROUTE_EVAL_UNREACHABLE;
    }

    if (out) {
        out->plan = *rp;
        out->chosen_origin = chosen_origin;
        out->need_initial_reverse = need_initial_reverse;
        out->blocker_mask = 0;
    }
    return POS_ROUTE_EVAL_READY;
}

static pos_route_eval_result_t pos_evaluate_target_ready_now(train_pos_t *pos,
                                                             track_node *user_target,
                                                             pos_route_eval_t *out) {
    pos_route_eval_t *eval = out ? out : &g_pos_try_eval_ready;
    route_plan_t *reserve_plan = &g_pos_try_reserve_plan;

    pos_route_eval_result_t result = pos_evaluate_target(pos, user_target, eval);
    if (result != POS_ROUTE_EVAL_READY) return result;

    *reserve_plan = eval->plan;
    if (reserve_plan->has_reversal) {
        reserve_plan->path_count2 = 0;
    }

    if (!traffic_can_reserve_plan(pos->train_num, reserve_plan)) {
        eval->blocker_mask = blocker_mask_from_plan(pos->train_num, reserve_plan);
        return POS_ROUTE_EVAL_BLOCKED;
    }

    if (pos_route_switch_blocker(eval->plan.sw_nums, eval->plan.sw_dirs,
                                 eval->plan.sw_count, pos->train_num) >= 0) {
        eval->blocker_mask = blocker_mask_from_switches(eval->plan.sw_nums,
                                                        eval->plan.sw_dirs,
                                                        eval->plan.sw_count,
                                                        pos->train_num);
        return POS_ROUTE_EVAL_BLOCKED;
    }

    eval->blocker_mask = 0;
    return POS_ROUTE_EVAL_READY;
}

static uint8_t pos_simulate_deadlock_unblocked_mask(train_pos_t *victim,
                                                    uint8_t cycle_mask,
                                                    track_node *yield_target) {
    uint8_t ready_mask = 0;
    track_node *keep_end;

    if (!victim || !yield_target) return 0;

    keep_end = pos_release_keep_end(yield_target, NULL);
    traffic_snapshot_reservations(g_pos_try_snapshot);
    traffic_simulate_parked_train(victim->train_num, yield_target, keep_end);

    for (int i = 0; i < 6; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        train_pos_t *other;
        track_node *target;

        if (!(cycle_mask & bit)) continue;
        if (pos_deadlock_index_to_train(i) == victim->train_num) continue;

        other = pos_get(pos_deadlock_index_to_train(i));
        if (!other || other->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

        pos_restore_pending_target(other);
        target = pos_route_current_goal(other);
        if (!target) continue;

        if (pos_evaluate_target_ready_now(other, target, NULL) == POS_ROUTE_EVAL_READY) {
            ready_mask |= bit;
        }
    }

    traffic_restore_reservations(g_pos_try_snapshot);
    return ready_mask;
}

static int pos_deadlock_is_canonical_sensor(track_node *cand) {
    if (!cand || cand->type != NODE_SENSOR) return 0;
    if (!cand->reverse || cand->reverse->type != NODE_SENSOR) return 1;
    return cand < cand->reverse;
}

static int pos_deadlock_insert_yield_candidate(track_node *target,
                                               int32_t total_dist_mm) {
    pos_deadlock_yield_candidate_t entry;
    int count = 0;
    int insert_at = DEADLOCK_YIELD_MAX_CANDIDATES;

    if (!target) return 0;

    entry.target = target;
    entry.total_dist_mm = total_dist_mm;

    while (count < DEADLOCK_YIELD_MAX_CANDIDATES &&
           g_pos_try_deadlock_candidates[count].target != NULL) {
        if (insert_at == DEADLOCK_YIELD_MAX_CANDIDATES &&
            total_dist_mm < g_pos_try_deadlock_candidates[count].total_dist_mm) {
            insert_at = count;
        }
        count++;
    }

    if (insert_at == DEADLOCK_YIELD_MAX_CANDIDATES) {
        if (count >= DEADLOCK_YIELD_MAX_CANDIDATES) return count;
        g_pos_try_deadlock_candidates[count] = entry;
        return count + 1;
    }

    if (count < DEADLOCK_YIELD_MAX_CANDIDATES) count++;
    for (int i = count - 1; i > insert_at; i--) {
        g_pos_try_deadlock_candidates[i] = g_pos_try_deadlock_candidates[i - 1];
    }
    g_pos_try_deadlock_candidates[insert_at] = entry;
    return count;
}

int pos_pick_deadlock_yield_target(train_pos_t *pos, uint8_t cycle_mask,
                                   track_node **out_target, uint8_t *out_unblocked_mask) {
    track_node *current_target;
    track_node *best_target = NULL;
    track_node *fallback_target = NULL;
    uint8_t best_unblocked_mask = 0;
    uint8_t fallback_wait_mask;
    int32_t best_dist = 0;
    int32_t fallback_dist = 0;
    int candidate_count = 0;

    if (out_target) *out_target = NULL;
    if (out_unblocked_mask) *out_unblocked_mask = 0;
    if (!pos || !pos->cur_sensor) return 0;

    current_target = pos_route_current_goal(pos);
    fallback_wait_mask = cycle_mask & (uint8_t)~pos_deadlock_train_bit(pos->train_num);
    for (int i = 0; i < DEADLOCK_YIELD_MAX_CANDIDATES; i++) {
        g_pos_try_deadlock_candidates[i].target = NULL;
        g_pos_try_deadlock_candidates[i].total_dist_mm = 0;
    }

    /* Keep only the nearest ready-now park targets; simulating every sensor
     * against the whole wait cycle is too expensive on the hot path. */
    for (int i = 0; i < TRACK_MAX; i++) {
        int32_t cand_total_dist;
        track_node *cand = &g_track[i];
        if (!pos_deadlock_is_canonical_sensor(cand)) continue;
        if (cand == pos->cur_sensor) continue;
        if (cand == current_target) continue;
        if (pos_evaluate_target_ready_now(pos, cand, &g_pos_try_eval_candidate) != POS_ROUTE_EVAL_READY) continue;
        g_pos_try_candidate_plan = g_pos_try_eval_candidate.plan;
        cand_total_dist = g_pos_try_candidate_plan.total_dist_mm;
        if (fallback_target == NULL || cand_total_dist < fallback_dist) {
            fallback_target = cand;
            fallback_dist = cand_total_dist;
        }
        candidate_count = pos_deadlock_insert_yield_candidate(cand, cand_total_dist);
    }

    for (int i = 0; i < candidate_count; i++) {
        uint8_t unblocked_mask;
        track_node *cand = g_pos_try_deadlock_candidates[i].target;
        int32_t cand_total_dist = g_pos_try_deadlock_candidates[i].total_dist_mm;

        if (!cand) continue;
        unblocked_mask = pos_simulate_deadlock_unblocked_mask(pos, cycle_mask, cand);
        if (unblocked_mask == 0) continue;
        if (best_target == NULL ||
            blocker_mask_bit_count(unblocked_mask) > blocker_mask_bit_count(best_unblocked_mask) ||
            (blocker_mask_bit_count(unblocked_mask) == blocker_mask_bit_count(best_unblocked_mask) &&
             cand_total_dist < best_dist)) {
            best_target = cand;
            best_unblocked_mask = unblocked_mask;
            best_dist = cand_total_dist;
        }
    }

    if (best_target) {
        if (out_target) *out_target = best_target;
        if (out_unblocked_mask) *out_unblocked_mask = best_unblocked_mask;
        return 1;
    }

    /* No immediate unlock found: still move to the nearest viable park target
     * so the reservation graph changes and later retries can progress. */
    if (!fallback_target) return 0;
    if (out_target) *out_target = fallback_target;
    if (out_unblocked_mask) *out_unblocked_mask = fallback_wait_mask;
    return 1;
}

static int pos_try_direct_goto_impl(train_pos_t *pos,
                                    int wait_on_unreachable) {
    track_node *user_target = pos->pending_target;
    int32_t     offset_mm   = pos->pending_offset_mm;
    track_node *cur_sensor_orig;
    route_plan_t *rp;
    track_node *chosen_origin;
    int need_initial_reverse;

    if (!pos->cur_sensor || !user_target) return 0;

    switch (pos_evaluate_target_ready_now(pos, user_target, &g_pos_try_eval_main)) {
    case POS_ROUTE_EVAL_UNREACHABLE:
        if (!wait_on_unreachable) return 0;
        /* Treat planner misses like a transient wait so STOPPED callers and
         * replan loops keep retrying instead of tripping KASSERTs. */
        pos_enter_wait_resource(pos, read_timer(), 0);
        return 1;
    case POS_ROUTE_EVAL_BLOCKED:
        pos_enter_wait_resource(pos, read_timer(), g_pos_try_eval_main.blocker_mask);
        return 1;
    case POS_ROUTE_EVAL_READY:
        break;
    default:
        return 0;
    }

    cur_sensor_orig = pos->cur_sensor;
    rp = &g_pos_try_eval_main.plan;
    chosen_origin = g_pos_try_eval_main.chosen_origin;
    need_initial_reverse = g_pos_try_eval_main.need_initial_reverse;

    uint64_t now_us = read_timer();
    track_node *eff_start = chosen_origin;
    track_node *chosen_target = rp->has_reversal ? rp->reversal_sensor
                                                  : rp->chosen_target;
    route_plan_t *reserve_plan = &g_pos_try_reserve_plan;
    *reserve_plan = *rp;
    if (reserve_plan->has_reversal) {
        /* Reserve only the first leg up to the reversal point.
         * The second leg is reserved when the train actually reaches the midpoint. */
        reserve_plan->path_count2 = 0;
    }
    /* Keep the parked sensor window until the next real hit.
     * The new route is added on top of the existing stopped reservation. */
    if (!traffic_can_reserve_plan(pos->train_num, reserve_plan)) {
        uint8_t blocker_mask = blocker_mask_from_plan(pos->train_num, reserve_plan);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 1;
    }
    if (!pos_apply_route_switches_safe(rp->sw_nums, rp->sw_dirs, rp->sw_count,
                                       pos->train_num)) {
        uint8_t blocker_mask = blocker_mask_from_switches(rp->sw_nums, rp->sw_dirs,
                                                          rp->sw_count, pos->train_num);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 1;
    }
    if (!traffic_reserve_plan(pos->train_num, eff_start, reserve_plan)) {
        uint8_t blocker_mask = blocker_mask_from_plan(pos->train_num, reserve_plan);
        pos_enter_wait_resource(pos, now_us, blocker_mask);
        return 1;
    }
    if (rp->sw_count > 0) ui_mark_switches_dirty();

    if (need_initial_reverse) {
        track_reverse(pos->train_num);
        pos->cur_sensor    = cur_sensor_orig->reverse;
        pos->going_forward = !pos->going_forward;
        pos->pred.next_sensor  = cur_sensor_orig->reverse;
        pos->pred.alt_sensor   = NULL;
        pos->pred.branch_node  = NULL;
        pos->pred.trigger_time = 0;
        pos->pred.skipped_sensor_count = 0;
        pos->dead_track_deadline_us = 0;
    }
    pos->offroute_valid           = 0;
    pos->offroute_expected_sensor = NULL;

    /* Set up mid-route reversal state if needed. */
    if (rp->has_reversal) {
        pos->midrev.active       = 1;
        pos->midrev.sensor       = rp->reversal_sensor;
        pos->midrev.final_target = rp->chosen_target;
        pos->midrev.final_offset = offset_mm;
        pos->midrev.sw_count     = rp->sw_count2;
        for (int i = 0; i < rp->sw_count2; i++) {
            pos->midrev.sw_nums[i] = rp->sw_nums2[i];
            pos->midrev.sw_dirs[i] = rp->sw_dirs2[i];
        }
        pos->midrev.dist_after = rp->dist_after_reversal_mm;
        pos->target_sensor    = rp->reversal_sensor;
        pos->target_offset_mm = 0;
        pos->orig_user_target   = pos->midrev.final_target;
        pos->orig_target_offset = offset_mm;

        pos->route_path_count = rp->path_count;
        for (int i = 0; i < rp->path_count; i++) pos->route_path[i] = rp->path_nodes[i];
        pos->midrev.path2_count = rp->path_count2;
        for (int i = 0; i < rp->path_count2; i++) pos->midrev.path2[i] = rp->path_nodes2[i];
    } else {
        pos->midrev.active    = 0;
        pos->target_sensor    = chosen_target;
        pos->target_offset_mm = offset_mm;

        pos->route_path_count   = rp->path_count;
        pos->midrev.path2_count = 0;
        for (int i = 0; i < rp->path_count; i++) pos->route_path[i] = rp->path_nodes[i];
    }

    pos->route_path_cursor = 0;

    int32_t pd = route_path_dist_from(pos->route_path, 0, pos->route_path_count);
    pos->dist_to_target_mm = (pd >= 0) ? pd + pos->target_offset_mm : 0;
    if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;

    pos->pending_target    = NULL;
    pos->pending_offset_mm = 0;
    pos->replan.next_us = 0;
    pos->replan.seen_generation = traffic_get_change_generation();
    pos->replan.blocker_mask = 0;

    pos_arm_switch_settle(pos, rp->sw_count,
                          need_initial_reverse ? POS_SWITCH_SETTLE_REVERSED
                                               : POS_SWITCH_SETTLE_NORMAL,
                          now_us);

    ui_mark_position_dirty();
    return 1;
}

int pos_try_direct_goto(train_pos_t *pos) {
    return pos_try_direct_goto_impl(pos, 1);
}

int pos_try_direct_goto_strict(train_pos_t *pos) {
    return pos_try_direct_goto_impl(pos, 0);
}
