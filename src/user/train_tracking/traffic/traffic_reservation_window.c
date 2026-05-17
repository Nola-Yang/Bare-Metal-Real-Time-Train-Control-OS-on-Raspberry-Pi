#include "traffic_window_internal.h"
#include "train_tracking/route_priv.h"
#include "track.h"
#include <stddef.h>
#include <stdint.h>

static int append_prefix_switch(route_plan_t *plan, int sw_num, char sw_dir) {
    if (!plan) return 0;
    if (plan->sw_count >= 20) return 0;
    plan->sw_nums[plan->sw_count] = sw_num;
    plan->sw_dirs[plan->sw_count] = sw_dir;
    plan->sw_count++;
    return 1;
}

static int append_extra_reserve_node(route_plan_t *plan, uint16_t node_idx) {
    if (!plan) return 0;
    for (int i = 0; i < plan->extra_reserve_count; i++) {
        if (plan->extra_reserve_nodes[i] == node_idx) return 1;
    }
    if (plan->extra_reserve_count >= 4) return 0;
    plan->extra_reserve_nodes[plan->extra_reserve_count++] = node_idx;
    return 1;
}

int traffic_window_get_trailing_branch(const uint16_t *path, int path_count,
                                       int end_cursor,
                                       int *out_branch_idx,
                                       int *out_sw_num,
                                       char *out_sw_dir) {
    int branch_idx;
    int next_idx;
    track_node *branch;
    track_node *next;
    char sw_dir;

    if (out_branch_idx) *out_branch_idx = -1;
    if (out_sw_num) *out_sw_num = -1;
    if (out_sw_dir) *out_sw_dir = '?';

    if (!path || path_count <= 0) return 0;
    if (end_cursor < 0 || end_cursor + 2 >= path_count) return 0;

    branch_idx = (int)path[end_cursor + 1];
    next_idx = (int)path[end_cursor + 2];
    if (branch_idx < 0 || branch_idx >= TRACK_MAX) return 0;
    if (next_idx < 0 || next_idx >= TRACK_MAX) return 0;

    branch = &g_track[branch_idx];
    next = &g_track[next_idx];
    if (branch->type != NODE_BRANCH) return 0;

    if (branch->edge[DIR_STRAIGHT].dest == next) {
        sw_dir = 'S';
    } else if (branch->edge[DIR_CURVED].dest == next) {
        sw_dir = 'C';
    } else {
        return 0;
    }

    if (out_branch_idx) *out_branch_idx = branch_idx;
    if (out_sw_num) *out_sw_num = branch->num;
    if (out_sw_dir) *out_sw_dir = sw_dir;
    return 1;
}

int traffic_window_build_prefix_plan(const uint16_t *path, int path_count,
                                     int start_cursor, int end_cursor,
                                     route_plan_t *out_plan) {
    int trailing_branch_idx;
    int trailing_sw_num;
    char trailing_sw_dir;
    track_node *trailing_branch;

    if (!path || !out_plan) return 0;
    if (path_count <= 0) return 0;
    if (start_cursor < 0) start_cursor = 0;
    if (end_cursor >= path_count) end_cursor = path_count - 1;
    if (start_cursor > end_cursor) return 0;
    *out_plan = (route_plan_t){0};

    for (int src = start_cursor; src <= end_cursor; src++) {
        int dst = src - start_cursor;
        if (dst >= TRACK_MAX) return 0;
        out_plan->path_nodes[dst] = path[src];
        out_plan->path_count++;
    }

    if (out_plan->path_count <= 0) return 0;

    for (int i = 0; i < out_plan->path_count - 1; i++) {
        track_node *a = &g_track[out_plan->path_nodes[i]];
        track_node *b = &g_track[out_plan->path_nodes[i + 1]];

        if (a->type != NODE_BRANCH) continue;

        if (a->edge[DIR_STRAIGHT].dest == b) {
            if (!append_prefix_switch(out_plan, a->num, 'S')) return 0;
        } else if (a->edge[DIR_CURVED].dest == b) {
            if (!append_prefix_switch(out_plan, a->num, 'C')) return 0;
        } else {
            return 0;
        }
    }

    /* Keep the authority stop at end_cursor, but if the next planned node is
     * a branch, also reserve and pre-stage that switch. */
    if (traffic_window_get_trailing_branch(path, path_count, end_cursor,
                                           &trailing_branch_idx,
                                           &trailing_sw_num,
                                           &trailing_sw_dir)) {
        trailing_branch = &g_track[trailing_branch_idx];
        if (!append_prefix_switch(out_plan, trailing_sw_num, trailing_sw_dir)) {
            return 0;
        }
        if (!append_extra_reserve_node(out_plan, (uint16_t)trailing_branch_idx)) {
            return 0;
        }
        if (trailing_branch->edge[DIR_STRAIGHT].dest != NULL &&
            !append_extra_reserve_node(out_plan,
                                       (uint16_t)(trailing_branch->edge[DIR_STRAIGHT].dest - g_track))) {
            return 0;
        }
        if (trailing_branch->edge[DIR_CURVED].dest != NULL &&
            !append_extra_reserve_node(out_plan,
                                       (uint16_t)(trailing_branch->edge[DIR_CURVED].dest - g_track))) {
            return 0;
        }
    }

    out_plan->chosen_target = &g_track[path[end_cursor]];
    out_plan->total_dist_mm = route_path_dist_from(path, start_cursor,
                                                   end_cursor + 1);
    if (out_plan->total_dist_mm < 0) return 0;
    return 1;
}
