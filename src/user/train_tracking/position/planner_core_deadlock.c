#include "train_tracking/planner_core.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "train_tracking/speed_table.h"
#include "track.h"
#include <stddef.h>
#include <stdint.h>

static int planner_local_get_next_dir(const planner_env_t *env,
                                      track_node *node) {
    int sw_idx;
    char state;

    if (!env || !env->switch_state || !node) return -1;
    switch (node->type) {
    case NODE_SENSOR:
    case NODE_MERGE:
    case NODE_ENTER:
        return DIR_AHEAD;
    case NODE_BRANCH:
        sw_idx = track_switch_to_index(node->num);
        if (sw_idx < 0) return -1;
        state = env->switch_state[sw_idx];
        if (state == 'S') return DIR_STRAIGHT;
        if (state == 'C') return DIR_CURVED;
        return -1;
    default:
        return -1;
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

static track_edge *planner_local_get_next_edge(const planner_env_t *env,
                                               track_node *node) {
    int dir = planner_local_get_next_dir(env, node);
    if (!node || dir < 0) return NULL;
    return &node->edge[dir];
}

static int planner_local_follow_dist(const planner_env_t *env,
                                     track_node *cur,
                                     track_node *to,
                                     int max_hops) {
    int32_t dist = 0;

    if (!env || !cur || !to) return -1;
    if (cur == to) return 0;
    for (int h = 0; h < max_hops; h++) {
        track_edge *edge = planner_local_get_next_edge(env, cur);
        if (!edge || !edge->dest) return -1;
        dist += edge->dist;
        cur = edge->dest;
        if (cur == to) return dist;
        if (cur->type == NODE_EXIT) return -1;
    }
    return -1;
}

static track_node *planner_predict_next_sensor(const planner_env_t *env,
                                               track_node *cur) {
    if (!env || !cur) return NULL;

    for (int hops = 0; hops < 80; hops++) {
        track_edge *edge = planner_local_get_next_edge(env, cur);
        if (!edge || !edge->dest) return NULL;
        cur = edge->dest;
        if (cur->type == NODE_SENSOR) return cur;
        if (cur->type == NODE_EXIT) return NULL;
    }
    return NULL;
}

static void planner_keep_mark_node(uint8_t keep[TRACK_MAX], track_node *node) {
    int idx;
    int ridx;

    if (!keep || !node) return;
    idx = (int)(node - g_track);
    ridx = planner_reverse_index(idx);
    if (idx >= 0 && idx < TRACK_MAX) keep[idx] = 1;
    if (ridx >= 0 && ridx < TRACK_MAX) keep[ridx] = 1;
}

static void planner_keep_mark_walk_dist(const planner_env_t *env,
                                        uint8_t keep[TRACK_MAX],
                                        track_node *start,
                                        int32_t dist_mm) {
    track_node *cur;
    int32_t dist = 0;

    if (!env || !keep || !start) return;
    cur = start;
    for (int h = 0; h < 200; h++) {
        track_edge *edge;

        planner_keep_mark_node(keep, cur);
        if (dist >= dist_mm) break;
        edge = planner_local_get_next_edge(env, cur);
        if (!edge || !edge->dest) break;
        dist += edge->dist;
        cur = edge->dest;
    }
}

static void planner_keep_mark_walk_to(const planner_env_t *env,
                                      uint8_t keep[TRACK_MAX],
                                      track_node *start,
                                      track_node *end) {
    track_node *cur;

    if (!env || !keep || !start || !end) return;
    cur = start;
    for (int h = 0; h < 200; h++) {
        track_edge *edge;

        planner_keep_mark_node(keep, cur);
        if (cur == end) break;
        edge = planner_local_get_next_edge(env, cur);
        if (!edge || !edge->dest) break;
        cur = edge->dest;
    }
}

static void planner_build_keep_body_marks(const planner_env_t *env,
                                          track_node *last_hit,
                                          int32_t body_mm,
                                          track_node *next_hit,
                                          uint8_t keep[TRACK_MAX]) {
    int keep_to_next = 0;

    for (int i = 0; i < TRACK_MAX; i++) keep[i] = 0;
    if (last_hit && next_hit) {
        keep_to_next = (last_hit == next_hit) ||
                       (planner_local_follow_dist(env, last_hit, next_hit,
                                                  120) >= 0);
    }

    if (!last_hit) {
        traffic_expand_zone_marks(keep);
        return;
    }

    planner_keep_mark_node(keep, last_hit);
    if (last_hit->reverse) {
        planner_keep_mark_walk_dist(env, keep, last_hit->reverse, body_mm);
    }
    if (keep_to_next) {
        if (last_hit != next_hit) {
            planner_keep_mark_walk_to(env, keep, last_hit, next_hit);
        }
        planner_keep_mark_walk_dist(env, keep, next_hit, body_mm);
    }
    traffic_expand_zone_marks(keep);
}

static int planner_train_is_manual_stop_blocker(const planner_env_t *env,
                                                const planner_train_view_t *view) {
    if (!view || view->route_state != TRAIN_STATE_STOPPED) return 0;
    return !env || !env->auto_dispatching_targets;
}

static int planner_train_is_yield_stop_blocker(const planner_train_view_t *view) {
    if (!view || view->route_state != TRAIN_STATE_STOPPED) return 0;
    if (view->resume_target == NULL) return 0;
    if (view->yield_target == NULL) return 0;
    return view->parked_at_yield != 0;
}

static int planner_train_is_stopped_blocker(const planner_env_t *env,
                                            const planner_train_view_t *view) {
    return planner_train_is_manual_stop_blocker(env, view) ||
           planner_train_is_yield_stop_blocker(view);
}

static void planner_collect_participants(
    const planner_env_t *env, const planner_train_view_t *const *views,
    int view_count, planner_deadlock_participants_t *parts) {
    if (!parts) return;

    *parts = (planner_deadlock_participants_t){0};
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        parts->train_nums[i] = -1;
    }
    if (!views) return;

    for (int i = 0; i < view_count && parts->count < DEADLOCK_MAX_TRAINS; i++) {
        const planner_train_view_t *view = views[i];
        uint8_t local_bit;

        if (!view || view->train_num < 0) continue;
        if (view->route_state != TRAIN_STATE_WAIT_RESOURCE &&
            !planner_train_is_stopped_blocker(env, view)) {
            continue;
        }

        local_bit = (uint8_t)(1u << parts->count);
        parts->views[parts->count] = view;
        parts->train_nums[parts->count] = view->train_num;
        parts->global_bits[parts->count] = planner_train_bit(view->train_num);
        if (view->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            parts->wait_mask |= local_bit;
        } else {
            parts->stopped_mask |= local_bit;
        }
        parts->count++;
    }
}

static int planner_participant_index(const planner_deadlock_participants_t *parts,
                                     int train_num) {
    if (!parts) return -1;
    for (int i = 0; i < parts->count; i++) {
        if (parts->train_nums[i] == train_num) return i;
    }
    return -1;
}

static void planner_build_graph(const planner_deadlock_participants_t *parts,
                                uint8_t adj[DEADLOCK_MAX_TRAINS]) {
    if (!parts || !adj) return;

    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) adj[i] = 0;

    for (int i = 0; i < parts->count; i++) {
        const planner_train_view_t *view;

        if (!(parts->wait_mask & (uint8_t)(1u << i))) continue;
        view = parts->views[i];
        if (!view || view->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

        for (int j = 0; j < parts->count; j++) {
            if (i == j) continue;
            if (!(parts->wait_mask & (uint8_t)(1u << j))) continue;
            if (view->blocker_mask & parts->global_bits[j]) {
                adj[i] |= (uint8_t)(1u << j);
            }
        }
    }
}

static void planner_compute_reachability(const uint8_t adj[DEADLOCK_MAX_TRAINS],
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

static uint8_t planner_find_stopped_blocker_mask_for_train(
    int train_num, const planner_deadlock_participants_t *parts) {
    const planner_train_view_t *view;
    int start_idx = planner_participant_index(parts, train_num);
    uint8_t blockers = 0;

    if (!parts || start_idx < 0) return 0;
    view = parts->views[start_idx];
    if (!view || view->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    for (int i = 0; i < parts->count; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        if (!(parts->stopped_mask & bit)) continue;
        if (view->blocker_mask & parts->global_bits[i]) blockers |= bit;
    }

    if (blockers == 0) return 0;
    return (uint8_t)(blockers | (uint8_t)(1u << start_idx));
}

static int32_t planner_candidate_sort_dist(track_node *const origins[2],
                                           track_node *cand) {
    int32_t best = -1;

    if (!cand) return -1;
    for (int i = 0; i < 2; i++) {
        int32_t dist;
        if (!origins[i]) continue;
        dist = route_direct_sensor_dist(origins[i], cand);
        if (dist < 0) continue;
        if (best < 0 || dist < best) best = dist;
    }
    return best;
}

static int planner_merge_sorted_candidates(track_node *const origins[2],
                                           planner_workspace_t *ws) {
    int count0 = 0;
    int count1 = 0;
    int i0 = 0;
    int i1 = 0;
    int out_count = 0;

    if (!ws) return 0;

    __builtin_memset(ws->seen_candidate, 0, sizeof(ws->seen_candidate));

    if (origins[0]) {
        count0 = route_fill_sorted_direct_sensor_candidates(
            origins[0], ws->sorted_from_origin0, TRACK_MAX);
    }
    if (origins[1]) {
        count1 = route_fill_sorted_direct_sensor_candidates(
            origins[1], ws->sorted_from_origin1, TRACK_MAX);
    }

    while (i0 < count0 || i1 < count1) {
        track_node *cand0 = NULL;
        track_node *cand1 = NULL;
        int32_t dist0 = -1;
        int32_t dist1 = -1;
        track_node *chosen;
        int idx;

        if (i0 < count0) {
            cand0 = &g_track[ws->sorted_from_origin0[i0]];
            dist0 = planner_candidate_sort_dist(origins, cand0);
        }
        if (i1 < count1) {
            cand1 = &g_track[ws->sorted_from_origin1[i1]];
            dist1 = planner_candidate_sort_dist(origins, cand1);
        }

        if (!cand1 ||
            (cand0 != NULL &&
             (dist1 < 0 || dist0 < dist1 ||
              (dist0 == dist1 &&
               ws->sorted_from_origin0[i0] < ws->sorted_from_origin1[i1])))) {
            chosen = cand0;
            i0++;
        } else {
            chosen = cand1;
            i1++;
        }

        if (!chosen) continue;
        idx = (int)(chosen - g_track);
        if (idx < 0 || idx >= TRACK_MAX) continue;
        if (ws->seen_candidate[idx]) continue;

        ws->seen_candidate[idx] = 1;
        ws->merged_candidates[out_count++] = (uint16_t)idx;
    }

    return out_count;
}

uint8_t planner_deadlock_global_mask_from_local(
    const planner_deadlock_participants_t *parts, uint8_t local_mask) {
    uint8_t global_mask = 0;

    if (!parts) return 0;
    for (int i = 0; i < parts->count; i++) {
        if (local_mask & (uint8_t)(1u << i)) {
            global_mask |= parts->global_bits[i];
        }
    }
    return global_mask;
}

uint8_t planner_detect_deadlock(
    const planner_env_t *env, const planner_train_view_t *const *views,
    int view_count, int train_num, planner_deadlock_kind_t *out_kind,
    planner_deadlock_participants_t *out_parts) {
    planner_deadlock_participants_t parts;
    uint8_t adj[DEADLOCK_MAX_TRAINS];
    uint8_t reach[DEADLOCK_MAX_TRAINS];
    uint8_t cycle = 0;
    uint8_t stopped_blockers;
    int start_idx;

    if (out_kind) *out_kind = PLANNER_DEADLOCK_KIND_NONE;

    planner_collect_participants(env, views, view_count, &parts);
    if (out_parts) *out_parts = parts;

    start_idx = planner_participant_index(&parts, train_num);
    if (start_idx < 0) return 0;

    planner_build_graph(&parts, adj);
    if (parts.wait_mask & (uint8_t)(1u << start_idx)) {
        planner_compute_reachability(adj, parts.count, parts.wait_mask, reach);
        for (int i = 0; i < parts.count; i++) {
            uint8_t bit = (uint8_t)(1u << i);
            if (!(parts.wait_mask & bit)) continue;
            if ((reach[start_idx] & bit) &&
                (reach[i] & (uint8_t)(1u << start_idx))) {
                cycle |= bit;
            }
        }
        if (planner_route_blocker_mask_bit_count(cycle) >= 2) {
            if (out_kind) *out_kind = PLANNER_DEADLOCK_KIND_WAIT_CYCLE;
            return cycle;
        }
    }

    stopped_blockers = planner_find_stopped_blocker_mask_for_train(train_num,
                                                                   &parts);
    if (planner_route_blocker_mask_bit_count(stopped_blockers) >= 2) {
        if (out_kind) *out_kind = PLANNER_DEADLOCK_KIND_STOPPED_BLOCKER;
        return stopped_blockers;
    }
    return 0;
}

int planner_choose_victim(const planner_deadlock_participants_t *parts,
                          uint8_t cycle_mask,
                          planner_deadlock_kind_t kind) {
    if (!parts) return -1;

    if (kind == PLANNER_DEADLOCK_KIND_STOPPED_BLOCKER) {
        for (int i = 0; i < parts->count; i++) {
            const planner_train_view_t *view = parts->views[i];
            if (!(cycle_mask & (uint8_t)(1u << i))) continue;
            if (view && view->route_state == TRAIN_STATE_STOPPED) {
                return parts->train_nums[i];
            }
        }
    }

    for (int i = 0; i < parts->count; i++) {
        const planner_train_view_t *view = parts->views[i];
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        if (view && view->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            return parts->train_nums[i];
        }
    }

    for (int i = 0; i < parts->count; i++) {
        if (cycle_mask & (uint8_t)(1u << i)) return parts->train_nums[i];
    }
    return -1;
}

uint8_t planner_simulate_deadlock_unblocked_mask(
    const planner_env_t *env, const planner_train_view_t *victim,
    const planner_train_view_t *const *views, int view_count, uint8_t cycle_mask,
    track_node *yield_target, planner_workspace_t *ws) {
    planner_env_t sim_env;
    uint8_t ready_mask = 0;
    uint8_t victim_bit;
    track_node *keep_end;

    if (!env || !victim || !yield_target || !ws || !env->reservation_owner) {
        return 0;
    }

    victim_bit = planner_train_bit(victim->train_num);
    for (int i = 0; i < TRACK_MAX; i++) {
        ws->owners_copy[i] = env->reservation_owner[i];
    }

    keep_end = planner_predict_next_sensor(env, yield_target);
    planner_build_keep_body_marks(env, yield_target, TRAIN_BODY_MM, keep_end,
                                  ws->keep);

    for (int i = 0; i < TRACK_MAX; i++) {
        if (ws->owners_copy[i] == victim->train_num) ws->owners_copy[i] = -1;
    }
    for (int i = 0; i < TRACK_MAX; i++) {
        if (ws->keep[i]) ws->owners_copy[i] = victim->train_num;
    }

    sim_env = *env;
    sim_env.reservation_owner = ws->owners_copy;

    for (int i = 0; i < view_count; i++) {
        const planner_train_view_t *other = views[i];
        uint8_t bit;

        if (!other || other->train_num < 0) continue;
        bit = planner_train_bit(other->train_num);
        if (!(cycle_mask & bit)) continue;
        if (other->train_num == victim->train_num) continue;
        if (other->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        if (!(other->blocker_mask & victim_bit)) continue;
        if (!other->goal) continue;

        if (planner_evaluate_target_ready_now(&sim_env, other, other->goal, ws,
                                              NULL) ==
            PLANNER_ROUTE_EVAL_READY) {
            ready_mask |= bit;
        }
    }

    return ready_mask;
}

int planner_pick_yield_target(const planner_env_t *env,
                              const planner_train_view_t *victim,
                              const planner_train_view_t *const *views,
                              int view_count,
                              uint8_t cycle_mask,
                              planner_workspace_t *ws,
                              track_node **out_target,
                              uint8_t *out_unblocked_mask,
                              planner_eval_t *out_eval,
                              planner_deadlock_pick_kind_t *out_kind) {
    track_node *best_unlock_target = NULL;
    track_node *best_ready_reloc_target = NULL;
    planner_eval_t best_unlock_eval = {0};
    planner_eval_t best_ready_reloc_eval = {0};
    uint8_t best_unblocked_mask = 0;
    uint8_t fallback_wait_mask;
    int merged_count;
    int32_t min_dist_mm;

    if (out_target) *out_target = NULL;
    if (out_unblocked_mask) *out_unblocked_mask = 0;
    if (out_eval) *out_eval = (planner_eval_t){0};
    if (out_kind) *out_kind = PLANNER_DEADLOCK_PICK_NONE;
    if (!env || !victim || !victim->cur_sensor || !ws) return 0;

    if (!victim->origins[0] && !victim->origins[1]) return 0;

    fallback_wait_mask = cycle_mask & (uint8_t)~planner_train_bit(victim->train_num);
    min_dist_mm = 0;
    {
        int32_t v;
        int32_t a;
        int32_t brake;
        int32_t early;

        v = speed_table_get_v(victim->train_ind, victim->goto_speed);
        a = speed_table_get_nominal_decel(victim->train_ind, victim->goto_speed);
        if (v > 0 && a > 0) {
            brake = v * v / (2 * a);
            early = (int32_t)((int64_t)v *
                              (int64_t)speed_table_get_early_stop(
                                  victim->train_ind, victim->goto_speed) /
                              1000000LL);
            if (brake < 0) brake = 0;
            if (early < 0) early = 0;
            min_dist_mm = GOTO_MIN_DIST_FACTOR * brake + early;
        }
    }
    if (min_dist_mm <= 0) return 0;

    merged_count = planner_merge_sorted_candidates(victim->origins, ws);

    for (int i = 0; i < merged_count; i++) {
        track_node *cand = &g_track[ws->merged_candidates[i]];
        planner_eval_t eval;
        uint8_t unblocked_mask;
        int32_t sort_dist;

        sort_dist = planner_candidate_sort_dist(victim->origins, cand);
        if (sort_dist < 0 || sort_dist < min_dist_mm) continue;
        if (planner_same_physical_sensor(cand, victim->cur_sensor)) continue;
        if (planner_same_physical_sensor(cand, victim->goal)) continue;
        if (planner_candidate_in_yield_history(victim, cand)) continue;

        if (planner_evaluate_target_ready_now(env, victim, cand, ws, &eval) !=
            PLANNER_ROUTE_EVAL_READY) {
            continue;
        }

        best_ready_reloc_target = cand;
        best_ready_reloc_eval = eval;
        unblocked_mask = planner_simulate_deadlock_unblocked_mask(
            env, victim, views, view_count, cycle_mask, cand, ws);
        if (unblocked_mask == 0) continue;

        best_unlock_target = cand;
        best_unlock_eval = eval;
        best_unblocked_mask = unblocked_mask;
        break;
    }

    if (best_unlock_target) {
        if (out_target) *out_target = best_unlock_target;
        if (out_unblocked_mask) *out_unblocked_mask = best_unblocked_mask;
        if (out_eval) *out_eval = best_unlock_eval;
        if (out_kind) *out_kind = PLANNER_DEADLOCK_PICK_READY_UNLOCK;
        return 1;
    }

    if (best_ready_reloc_target) {
        if (out_target) *out_target = best_ready_reloc_target;
        if (out_unblocked_mask) *out_unblocked_mask = fallback_wait_mask;
        if (out_eval) *out_eval = best_ready_reloc_eval;
        if (out_kind) *out_kind = PLANNER_DEADLOCK_PICK_READY_RELOCATE;
        return 1;
    }

    for (int i = merged_count - 1; i >= 0; i--) {
        track_node *cand = &g_track[ws->merged_candidates[i]];
        planner_eval_t eval;
        int reserved_end_cursor = -1;
        int switch_blocker_owner = -1;
        int32_t sort_dist = planner_candidate_sort_dist(victim->origins, cand);

        if (sort_dist < 0 || sort_dist < min_dist_mm) continue;
        if (planner_same_physical_sensor(cand, victim->cur_sensor)) continue;
        if (planner_same_physical_sensor(cand, victim->goal)) continue;
        if (planner_candidate_in_yield_history(victim, cand)) continue;
        if (planner_evaluate_target_plan(env, victim, cand, ws, &eval) !=
            PLANNER_ROUTE_EVAL_READY) {
            continue;
        }
        if (!planner_prepare_launch_strict(env, victim, &eval.plan, 0, 0, ws,
                                           &ws->authority_candidate_prefix,
                                           &reserved_end_cursor,
                                           &switch_blocker_owner)) {
            continue;
        }

        if (out_target) *out_target = cand;
        if (out_unblocked_mask) *out_unblocked_mask = fallback_wait_mask;
        if (out_eval) *out_eval = eval;
        if (out_kind) *out_kind = PLANNER_DEADLOCK_PICK_FORCE_MOVE;
        return 1;
    }

    return 0;
}
