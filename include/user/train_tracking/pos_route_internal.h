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

typedef enum {
    POS_DEADLOCK_PICK_NONE = 0,
    POS_DEADLOCK_PICK_READY_UNLOCK = 1,
    POS_DEADLOCK_PICK_READY_RELOCATE = 2,
    POS_DEADLOCK_PICK_FORCE_MOVE = 3,
} pos_deadlock_pick_kind_t;

track_node *pos_route_current_goal(train_pos_t *pos);

void pos_route_fill_origins(const train_pos_t *pos, track_node *origins[2]);
void pos_route_fill_deadlock_origins(const train_pos_t *pos,
                                     track_node *origins[2]);

void pos_route_build_constraints_for_train(int requester_train,
                                           uint8_t blocked[TRACK_MAX],
                                           char fixed_sw_dirs[TRACK_MAX]);

uint8_t pos_route_blocker_mask_from_plan(int requester_train,
                                         const route_plan_t *plan);

uint8_t pos_route_blocker_mask_from_switches(const int *sw_nums,
                                             const char *sw_dirs,
                                             int sw_count,
                                             int requester_train);

int pos_route_blocker_mask_bit_count(uint8_t mask);

pos_route_eval_result_t pos_evaluate_target_plan(train_pos_t *pos,
                                                 track_node *user_target,
                                                 pos_route_eval_t *out);
pos_route_eval_result_t pos_evaluate_target_plan_deadlock(train_pos_t *pos,
                                                          track_node *user_target,
                                                          pos_route_eval_t *out);

pos_route_eval_result_t pos_evaluate_target_ready_now(train_pos_t *pos,
                                                      track_node *user_target,
                                                      pos_route_eval_t *out);
pos_route_eval_result_t pos_evaluate_target_ready_now_deadlock(train_pos_t *pos,
                                                               track_node *user_target,
                                                               pos_route_eval_t *out);

/* Recompute the trains currently blocking a WAIT_RESOURCE train.
 * Returns 0 when the train is not blocked right now. */
uint8_t pos_wait_resource_current_blocker_mask(train_pos_t *pos);

#endif /* _pos_route_internal_h_ */
