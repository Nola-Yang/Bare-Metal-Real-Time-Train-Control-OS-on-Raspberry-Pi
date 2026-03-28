#include "train_tracking/position_priv.h"
#include "train_tracking/planner_core.h"
#include "demo_manager.h"
#include <stddef.h>
#include <stdint.h>

static planner_workspace_t g_pos_yield_ws;
static int g_pos_yield_owners[TRACK_MAX];
static char g_pos_yield_switch_state[MAX_SWITCHES];

int pos_pick_deadlock_yield_target(train_pos_t *pos, uint8_t cycle_mask,
                                   track_node **out_target,
                                   uint8_t *out_unblocked_mask,
                                   pos_deadlock_pick_kind_t *out_kind) {
    static const int train_order[6] = {13, 14, 15, 17, 18, 55};
    planner_env_t env;
    planner_train_view_t victim_view;
    planner_train_view_t live_views[6];
    const planner_train_view_t *view_ptrs[6];
    planner_deadlock_pick_kind_t pick_kind = PLANNER_DEADLOCK_PICK_NONE;
    int view_count = 0;

    if (out_target) *out_target = NULL;
    if (out_unblocked_mask) *out_unblocked_mask = 0;
    if (out_kind) *out_kind = POS_DEADLOCK_PICK_NONE;
    if (!pos || !pos->cur_sensor) return 0;

    pos_planner_build_env(&env, g_pos_yield_owners, g_pos_yield_switch_state);
    env.auto_dispatching_targets = demo_is_auto_dispatching_targets();
    pos_planner_fill_view(pos, &victim_view);

    for (int i = 0; i < 6; i++) {
        train_pos_t *other = pos_get(train_order[i]);

        if (!other) continue;
        pos_planner_fill_view(other, &live_views[view_count]);
        view_ptrs[view_count] = &live_views[view_count];
        view_count++;
    }

    if (!planner_pick_yield_target(&env, &victim_view, view_ptrs, view_count,
                                   cycle_mask, &g_pos_yield_ws, out_target,
                                   out_unblocked_mask, NULL, &pick_kind)) {
        return 0;
    }

    if (out_kind) *out_kind = (pos_deadlock_pick_kind_t)pick_kind;
    return 1;
}
