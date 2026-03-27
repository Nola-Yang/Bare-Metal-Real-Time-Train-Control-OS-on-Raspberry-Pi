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

int traffic_window_build_prefix_plan(const uint16_t *path, int path_count,
                                     int start_cursor, int end_cursor,
                                     route_plan_t *out_plan) {
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

    out_plan->chosen_target = &g_track[out_plan->path_nodes[out_plan->path_count - 1]];
    out_plan->total_dist_mm = route_path_dist_from(out_plan->path_nodes, 0,
                                                   out_plan->path_count);
    if (out_plan->total_dist_mm < 0) return 0;
    return 1;
}
