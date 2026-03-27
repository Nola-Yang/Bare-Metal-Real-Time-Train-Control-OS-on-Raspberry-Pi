#include "route_search_internal.h"

typedef struct {
    uint16_t sensor_idx;
    int32_t  lower_bound_mm;
} route_reversal_candidate_t;

static route_plan_t g_opt_best_plan;
static route_plan_t g_opt_cand_plan;
static route_reversal_candidate_t g_reversal_candidates[TRACK_MAX];
static uint32_t g_route_blocked_bits[ROUTE_BLOCKED_WORDS];

static int32_t fwd_dist[TRACK_MAX];
static int8_t  fwd_done[TRACK_MAX];
static int16_t fwd_prev[TRACK_MAX];
static int16_t fwd_sw_num[TRACK_MAX];
static char    fwd_sw_dir[TRACK_MAX];

static int32_t rev_dist[TRACK_MAX];
static int8_t  rev_done[TRACK_MAX];
static int16_t rev_prev[TRACK_MAX];
static int16_t rev_sw_num[TRACK_MAX];
static char    rev_sw_dir[TRACK_MAX];

static int dijk_reconstruct_nodes(int16_t *prev, track_node *target,
                                  uint16_t *out_nodes, int *out_count,
                                  int max_nodes) {
    int tgt_idx = (int)(target - g_track);
    uint16_t tmp[TRACK_MAX];
    int top = 0;
    int idx = tgt_idx;

    if (tgt_idx < 0 || tgt_idx >= TRACK_MAX ||
        !out_nodes || !out_count || max_nodes <= 0) {
        return 0;
    }

    while (idx >= 0 && idx < TRACK_MAX && top < TRACK_MAX) {
        tmp[top++] = (uint16_t)idx;
        idx = prev[idx];
    }
    if (top <= 0) return 0;

    *out_count = 0;
    for (int i = top - 1; i >= 0 && *out_count < max_nodes; i--) {
        out_nodes[*out_count] = tmp[i];
        (*out_count)++;
    }

    return (*out_count > 0);
}

static int32_t dijk_reconstruct(int32_t *dist, int16_t *prev,
                                int16_t *sw_num, char *sw_dir,
                                track_node *target,
                                int *out_sw_nums, char *out_sw_dirs,
                                int *out_sw_count, int max_sw) {
    int tgt_idx = (int)(target - g_track);
    int stk_num[20];
    char stk_dir[20];
    int stk_top = 0;
    int idx = tgt_idx;

    if (tgt_idx < 0 || tgt_idx >= TRACK_MAX) return -1;
    if (dist[tgt_idx] == DIJK_INF) return -1;

    while (prev[idx] >= 0 && stk_top < 20) {
        if (sw_num[idx] >= 0) {
            stk_num[stk_top] = sw_num[idx];
            stk_dir[stk_top] = sw_dir[idx];
            stk_top++;
        }
        idx = prev[idx];
    }

    *out_sw_count = 0;
    for (int i = stk_top - 1; i >= 0 && *out_sw_count < max_sw; i--) {
        out_sw_nums[*out_sw_count] = stk_num[i];
        out_sw_dirs[*out_sw_count] = stk_dir[i];
        (*out_sw_count)++;
    }

    return dist[tgt_idx];
}

static int32_t route_path_edge_dist(uint16_t from_idx, uint16_t to_idx) {
    track_node *from;
    track_node *to;

    if (from_idx >= TRACK_MAX || to_idx >= TRACK_MAX) return -1;

    from = &g_track[from_idx];
    to = &g_track[to_idx];

    if (from->type == NODE_BRANCH) {
        if (from->edge[DIR_STRAIGHT].dest == to) {
            return (int32_t)from->edge[DIR_STRAIGHT].dist;
        }
        if (from->edge[DIR_CURVED].dest == to) {
            return (int32_t)from->edge[DIR_CURVED].dist;
        }
        return -1;
    }

    if (from->edge[DIR_AHEAD].dest != to) return -1;
    return (int32_t)from->edge[DIR_AHEAD].dist;
}

static int32_t route_path_first_branch_dist(const uint16_t *path, int path_count) {
    int32_t total = 0;

    if (!path || path_count <= 1) return DIJK_INF;

    for (int i = 0; i < path_count - 1; i++) {
        int32_t edge_dist = route_path_edge_dist(path[i], path[i + 1]);
        if (edge_dist < 0) return -1;
        total += edge_dist;
        if (g_track[path[i + 1]].type == NODE_BRANCH) return total;
    }

    return DIJK_INF;
}

static int route_midrev_has_branch_clearance(const uint16_t *path, int path_count) {
    int32_t first_branch_dist = route_path_first_branch_dist(path, path_count);
    int32_t required_clearance = TRAIN_BODY_MM + MIDREV_STOP_TOLERANCE_MM;

    if (first_branch_dist < 0) return 0;
    if (first_branch_dist == DIJK_INF) return 1;
    return first_branch_dist >= required_clearance;
}

static int32_t route_sub_bound(int32_t upper_bound_mm, int32_t used_mm) {
    if (upper_bound_mm == DIJK_INF) return DIJK_INF;
    if (used_mm > upper_bound_mm) return -1;
    return upper_bound_mm - used_mm;
}

static int route_plan_is_better(const route_plan_t *cand, int cand_target_order,
                                const route_plan_t *best, int best_target_order,
                                int have_best) {
    int cand_rev_idx;
    int best_rev_idx;

    if (!cand) return 0;
    if (!have_best || !best) return 1;

    if (cand->total_dist_mm != best->total_dist_mm) {
        return cand->total_dist_mm < best->total_dist_mm;
    }
    if (cand_target_order != best_target_order) {
        return cand_target_order < best_target_order;
    }
    if (cand->has_reversal != best->has_reversal) {
        return cand->has_reversal < best->has_reversal;
    }

    cand_rev_idx = (cand->has_reversal && cand->reversal_sensor)
                       ? (int)(cand->reversal_sensor - g_track)
                       : -1;
    best_rev_idx = (best->has_reversal && best->reversal_sensor)
                       ? (int)(best->reversal_sensor - g_track)
                       : -1;
    return cand_rev_idx < best_rev_idx;
}

static int route_build_direct_plan(route_plan_t *cand, track_node *target,
                                   int32_t *dist, int16_t *prev,
                                   int16_t *sw_num, char *sw_dir) {
    if (!cand || !target) return 0;

    *cand = (route_plan_t){0};
    cand->has_reversal = 0;
    cand->chosen_target = target;
    cand->total_dist_mm = dijk_reconstruct(dist, prev, sw_num, sw_dir,
                                           target, cand->sw_nums, cand->sw_dirs,
                                           &cand->sw_count, 20);
    if (cand->total_dist_mm < 0) return 0;
    if (!dijk_reconstruct_nodes(prev, target, cand->path_nodes,
                                &cand->path_count, TRACK_MAX)) {
        return 0;
    }

    cand->path_count2 = 0;
    cand->sw_count2 = 0;
    return 1;
}

static int route_build_midrev_plan(route_plan_t *cand, track_node *target,
                                   track_node *reversal_sensor,
                                   int32_t penalty_mm,
                                   int32_t *dist1, int16_t *prev1,
                                   int16_t *sw_num1, char *sw_dir1,
                                   int32_t *dist2, int16_t *prev2,
                                   int16_t *sw_num2, char *sw_dir2) {
    int reversal_idx;
    int target_idx;

    if (!cand || !target || !reversal_sensor) return 0;

    reversal_idx = (int)(reversal_sensor - g_track);
    target_idx = (int)(target - g_track);

    *cand = (route_plan_t){0};
    cand->has_reversal = 1;
    cand->chosen_target = target;
    cand->reversal_sensor = reversal_sensor;
    cand->dist_to_reversal_mm = dist1[reversal_idx];
    cand->dist_after_reversal_mm = dist2[target_idx];
    if (cand->dist_to_reversal_mm == DIJK_INF ||
        cand->dist_after_reversal_mm == DIJK_INF) {
        return 0;
    }

    cand->total_dist_mm = cand->dist_to_reversal_mm + penalty_mm +
                          cand->dist_after_reversal_mm;

    dijk_reconstruct(dist1, prev1, sw_num1, sw_dir1,
                     reversal_sensor, cand->sw_nums, cand->sw_dirs,
                     &cand->sw_count, 20);
    if (!dijk_reconstruct_nodes(prev1, reversal_sensor, cand->path_nodes,
                                &cand->path_count, TRACK_MAX)) {
        return 0;
    }

    dijk_reconstruct(dist2, prev2, sw_num2, sw_dir2,
                     target, cand->sw_nums2, cand->sw_dirs2,
                     &cand->sw_count2, 20);
    if (!dijk_reconstruct_nodes(prev2, target, cand->path_nodes2,
                                &cand->path_count2, TRACK_MAX)) {
        return 0;
    }

    return route_midrev_has_branch_clearance(cand->path_nodes2,
                                             cand->path_count2);
}

static int route_build_reversal_candidates(track_node *start, track_node *target,
                                           int32_t penalty_mm,
                                           route_reversal_candidate_t *out) {
    int start_idx;
    int target_idx;
    int count = 0;

    if (!start || !target || !out) return 0;

    start_idx = (int)(start - g_track);
    target_idx = (int)(target - g_track);
    if (start_idx < 0 || start_idx >= TRACK_MAX ||
        target_idx < 0 || target_idx >= TRACK_MAX) {
        return 0;
    }

    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *sensor = &g_track[i];
        int rev_idx;
        int32_t dist_to_sensor;
        int32_t dist_after_rev;

        if (sensor->type != NODE_SENSOR || !sensor->reverse) continue;

        rev_idx = (int)(sensor->reverse - g_track);
        dist_to_sensor = route_free_track_dist_idx(start_idx, i);
        dist_after_rev = route_free_track_dist_idx(rev_idx, target_idx);
        if (dist_to_sensor == DIJK_INF || dist_after_rev == DIJK_INF) continue;

        out[count].sensor_idx = (uint16_t)i;
        out[count].lower_bound_mm = dist_to_sensor + penalty_mm + dist_after_rev;
        count++;
    }

    for (int i = 1; i < count; i++) {
        route_reversal_candidate_t cur = out[i];
        int j = i;

        while (j > 0) {
            route_reversal_candidate_t prev = out[j - 1];
            if (prev.lower_bound_mm < cur.lower_bound_mm ||
                (prev.lower_bound_mm == cur.lower_bound_mm &&
                 prev.sensor_idx < cur.sensor_idx)) {
                break;
            }
            out[j] = prev;
            j--;
        }
        out[j] = cur;
    }

    return count;
}

int bfs_find_route_optimal(track_node *start, track_node *target,
                           int32_t d_brake, route_plan_t *plan) {
    return bfs_find_route_optimal_constrained(start, target, d_brake,
                                              NULL, NULL, plan);
}

int bfs_find_route_optimal_constrained(track_node *start, track_node *target,
                                       int32_t d_brake, const uint8_t *blocked,
                                       const char *fixed_sw_dirs,
                                       route_plan_t *plan) {
    track_node *tgts[2];
    route_plan_t *best = &g_opt_best_plan;
    route_plan_t *cand = &g_opt_cand_plan;
    int best_target_order = 2;
    int have_best = 0;
    int32_t best_total = DIJK_INF;
    int32_t threshold;

    if (!start || !target || !plan) return 0;

    threshold = (int32_t)GOTO_MIN_DIST_FACTOR * d_brake;
    tgts[0] = target;
    tgts[1] = target->reverse;
    route_pack_blocked_bits(blocked, g_route_blocked_bits);

    for (int ti = 0; ti < 2; ti++) {
        track_node *tgt = tgts[ti];

        if (!tgt) continue;
        if (route_search_leg_astar(fwd_dist, fwd_done, fwd_prev, fwd_sw_num, fwd_sw_dir,
                                   start, tgt, blocked, g_route_blocked_bits,
                                   fixed_sw_dirs, best_total) &&
            route_build_direct_plan(cand, tgt, fwd_dist, fwd_prev,
                                    fwd_sw_num, fwd_sw_dir) &&
            route_plan_is_better(cand, ti, best, best_target_order, have_best)) {
            *best = *cand;
            best_total = best->total_dist_mm;
            best_target_order = ti;
            have_best = 1;
        }
    }

    for (int ti = 0; ti < 2; ti++) {
        track_node *tgt = tgts[ti];
        int cand_count;

        if (!tgt) continue;

        cand_count = route_build_reversal_candidates(start, tgt, 2 * d_brake,
                                                     g_reversal_candidates);
        for (int ci = 0; ci < cand_count; ci++) {
            track_node *sensor;
            int sensor_idx;
            int rev_idx;
            int32_t free_after_rev;
            int32_t first_leg_bound;
            int32_t second_leg_bound;
            int32_t first_leg_dist;
            int32_t second_leg_dist;

            if (have_best &&
                g_reversal_candidates[ci].lower_bound_mm > best_total) {
                break;
            }

            sensor_idx = (int)g_reversal_candidates[ci].sensor_idx;
            sensor = &g_track[sensor_idx];
            rev_idx = (int)(sensor->reverse - g_track);
            free_after_rev =
                route_free_track_dist_idx(rev_idx, (int)(tgt - g_track));

            first_leg_bound = route_sub_bound(best_total,
                                              2 * d_brake + free_after_rev);
            if (have_best && first_leg_bound < 0) continue;

            if (!route_search_leg_astar(fwd_dist, fwd_done, fwd_prev,
                                        fwd_sw_num, fwd_sw_dir,
                                        start, sensor, blocked, g_route_blocked_bits,
                                        fixed_sw_dirs, first_leg_bound)) {
                continue;
            }

            first_leg_dist = fwd_dist[sensor_idx];
            if (first_leg_dist == DIJK_INF || first_leg_dist < threshold) continue;
            if (have_best &&
                first_leg_dist + 2 * d_brake + free_after_rev > best_total) {
                continue;
            }

            second_leg_bound = route_sub_bound(best_total,
                                               first_leg_dist + 2 * d_brake);
            if (have_best && second_leg_bound < 0) continue;

            if (!route_search_leg_astar(rev_dist, rev_done, rev_prev,
                                        rev_sw_num, rev_sw_dir,
                                        sensor->reverse, tgt, blocked,
                                        g_route_blocked_bits, NULL,
                                        second_leg_bound)) {
                continue;
            }

            second_leg_dist = rev_dist[(int)(tgt - g_track)];
            if (second_leg_dist == DIJK_INF || second_leg_dist < threshold) continue;

            if (!route_build_midrev_plan(cand, tgt, sensor, 2 * d_brake,
                                         fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                                         rev_dist, rev_prev, rev_sw_num, rev_sw_dir)) {
                continue;
            }

            if (route_plan_is_better(cand, ti, best, best_target_order, have_best)) {
                *best = *cand;
                best_total = best->total_dist_mm;
                best_target_order = ti;
                have_best = 1;
            }
        }
    }

    if (!have_best) return 0;
    *plan = *best;
    return 1;
}

int bfs_find_bootstrap_midrev(track_node *start_rev, track_node *target,
                              int32_t d_brake, const uint8_t *blocked,
                              const char *fixed_sw_dirs,
                              route_plan_t *plan) {
    track_node *tgts[2];
    route_plan_t *best = &g_opt_best_plan;
    route_plan_t *cand = &g_opt_cand_plan;
    int best_target_order = 2;
    int have_best = 0;
    int32_t best_total = DIJK_INF;
    int32_t threshold;

    if (!start_rev || !target || !plan) return 0;

    threshold = (int32_t)GOTO_MIN_DIST_FACTOR * d_brake;
    tgts[0] = target;
    tgts[1] = target->reverse;
    route_pack_blocked_bits(blocked, g_route_blocked_bits);

    for (int ti = 0; ti < 2; ti++) {
        track_node *tgt = tgts[ti];
        int cand_count;

        if (!tgt) continue;

        cand_count = route_build_reversal_candidates(start_rev, tgt, 0,
                                                     g_reversal_candidates);
        for (int ci = 0; ci < cand_count; ci++) {
            track_node *sensor;
            int sensor_idx;
            int rev_idx;
            int32_t free_after_rev;
            int32_t first_leg_bound;
            int32_t second_leg_bound;
            int32_t first_leg_dist;
            int32_t second_leg_dist;

            if (have_best &&
                g_reversal_candidates[ci].lower_bound_mm > best_total) {
                break;
            }

            sensor_idx = (int)g_reversal_candidates[ci].sensor_idx;
            sensor = &g_track[sensor_idx];
            rev_idx = (int)(sensor->reverse - g_track);
            free_after_rev =
                route_free_track_dist_idx(rev_idx, (int)(tgt - g_track));

            first_leg_bound = route_sub_bound(best_total, free_after_rev);
            if (have_best && first_leg_bound < 0) continue;

            if (!route_search_leg_astar(fwd_dist, fwd_done, fwd_prev,
                                        fwd_sw_num, fwd_sw_dir,
                                        start_rev, sensor, blocked, g_route_blocked_bits,
                                        fixed_sw_dirs, first_leg_bound)) {
                continue;
            }

            first_leg_dist = fwd_dist[sensor_idx];
            if (first_leg_dist == DIJK_INF || first_leg_dist < threshold) continue;
            if (have_best && first_leg_dist + free_after_rev > best_total) continue;

            second_leg_bound = route_sub_bound(best_total, first_leg_dist);
            if (have_best && second_leg_bound < 0) continue;

            if (!route_search_leg_astar(rev_dist, rev_done, rev_prev,
                                        rev_sw_num, rev_sw_dir,
                                        sensor->reverse, tgt, blocked,
                                        g_route_blocked_bits, NULL,
                                        second_leg_bound)) {
                continue;
            }

            second_leg_dist = rev_dist[(int)(tgt - g_track)];
            if (second_leg_dist == DIJK_INF || second_leg_dist < threshold) continue;

            if (!route_build_midrev_plan(cand, tgt, sensor, 0,
                                         fwd_dist, fwd_prev, fwd_sw_num, fwd_sw_dir,
                                         rev_dist, rev_prev, rev_sw_num, rev_sw_dir)) {
                continue;
            }

            if (route_plan_is_better(cand, ti, best, best_target_order, have_best)) {
                *best = *cand;
                best_total = best->total_dist_mm;
                best_target_order = ti;
                have_best = 1;
            }
        }
    }

    if (!have_best) return 0;
    *plan = *best;
    return 1;
}
