#include "train_tracking/position.h"
#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"

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
