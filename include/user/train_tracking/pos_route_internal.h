#ifndef _pos_route_internal_h_
#define _pos_route_internal_h_

#include <stdint.h>
#include "position.h"

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

track_node *pos_route_current_goal(train_pos_t *pos);

uint8_t pos_route_blocker_mask_from_plan(int requester_train,
                                         const route_plan_t *plan);

uint8_t pos_route_blocker_mask_from_switches(const int *sw_nums,
                                             const char *sw_dirs,
                                             int sw_count,
                                             int requester_train);

int pos_route_blocker_mask_bit_count(uint8_t mask);

pos_route_eval_result_t pos_evaluate_target_ready_now(train_pos_t *pos,
                                                      track_node *user_target,
                                                      pos_route_eval_t *out);

#endif /* _pos_route_internal_h_ */
