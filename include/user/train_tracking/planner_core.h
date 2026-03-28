#ifndef _planner_core_h_
#define _planner_core_h_ 1

#include <stdint.h>
#include "position.h"

typedef struct {
    const int  *reservation_owner;
    const char *switch_state;
    uint8_t     auto_dispatching_targets;
} planner_env_t;

typedef struct {
    int                 train_num;
    int                 train_ind;
    train_route_state_t route_state;
    int                 goto_speed;
    track_node         *cur_sensor;
    track_node         *origins[2];
    track_node         *goal;
    int32_t             goal_offset_mm;
    uint8_t             blocker_mask;
    track_node         *resume_target;
    int32_t             resume_offset_mm;
    track_node         *yield_target;
    uint8_t             parked_at_yield;
    uint8_t             wait_start_mask;
    uint8_t             yield_history_count;
    track_node         *yield_history[DEADLOCK_YIELD_HISTORY_MAX];
} planner_train_view_t;

typedef struct {
    route_plan_t plan;
    track_node  *chosen_origin;
    int          need_initial_reverse;
    uint8_t      blocker_mask;
} planner_eval_t;

typedef enum {
    PLANNER_ROUTE_EVAL_UNREACHABLE = 0,
    PLANNER_ROUTE_EVAL_BLOCKED = 1,
    PLANNER_ROUTE_EVAL_READY = 2,
} planner_route_eval_result_t;

typedef enum {
    PLANNER_DEADLOCK_PICK_NONE = 0,
    PLANNER_DEADLOCK_PICK_READY_UNLOCK = 1,
    PLANNER_DEADLOCK_PICK_READY_RELOCATE = 2,
    PLANNER_DEADLOCK_PICK_FORCE_MOVE = 3,
} planner_deadlock_pick_kind_t;

typedef enum {
    PLANNER_DEADLOCK_KIND_NONE = 0,
    PLANNER_DEADLOCK_KIND_WAIT_CYCLE = 1,
    PLANNER_DEADLOCK_KIND_STOPPED_BLOCKER = 2,
} planner_deadlock_kind_t;

typedef struct {
    int                       count;
    const planner_train_view_t *views[DEADLOCK_MAX_TRAINS];
    int                       train_nums[DEADLOCK_MAX_TRAINS];
    uint8_t                   global_bits[DEADLOCK_MAX_TRAINS];
    uint8_t                   wait_mask;
    uint8_t                   stopped_mask;
} planner_deadlock_participants_t;

typedef struct {
    route_plan_t route_best_plan;
    route_plan_t route_temp_plan;
    route_plan_t route_blocked_plan;
    route_plan_t authority_candidate_prefix;
    route_plan_t authority_short_goal_prefix;
    uint8_t      blocked[TRACK_MAX];
    char         fixed_sw_dirs[TRACK_MAX];
    int          owners_copy[TRACK_MAX];
    uint8_t      keep[TRACK_MAX];
    uint16_t     sorted_from_origin0[TRACK_MAX];
    uint16_t     sorted_from_origin1[TRACK_MAX];
    uint16_t     merged_candidates[TRACK_MAX];
    uint8_t      seen_candidate[TRACK_MAX];
} planner_workspace_t;

uint8_t planner_train_bit(int train_num);
int planner_node_index(const track_node *node);
track_node *planner_node_from_index(int idx);
int32_t planner_view_stop_dist_mm(const planner_train_view_t *view);
int32_t planner_view_min_window_mm(const planner_train_view_t *view);
int planner_route_plan_long_enough(const route_plan_t *plan,
                                   int32_t threshold);
int planner_route_blocker_mask_bit_count(uint8_t mask);
int planner_same_physical_sensor(const track_node *a, const track_node *b);
int planner_candidate_in_yield_history(const planner_train_view_t *view,
                                       const track_node *cand);

int planner_select_best_route_for_origins(track_node *const origins[2],
                                          track_node *user_target,
                                          int32_t stop_dist_mm,
                                          int32_t min_window_mm,
                                          const uint8_t *blocked,
                                          const char *fixed_sw_dirs,
                                          planner_workspace_t *ws,
                                          route_plan_t *out_plan,
                                          track_node **out_origin,
                                          int *out_need_initial_reverse);

uint8_t planner_route_blocker_mask_from_plan(const planner_env_t *env,
                                             int requester_train,
                                             const route_plan_t *plan);
uint8_t planner_route_blocker_mask_from_switches(const planner_env_t *env,
                                                 const int *sw_nums,
                                                 const char *sw_dirs,
                                                 int sw_count,
                                                 int requester_train);

planner_route_eval_result_t planner_evaluate_target_plan(
    const planner_env_t *env, const planner_train_view_t *view,
    track_node *user_target, planner_workspace_t *ws, planner_eval_t *out);

planner_route_eval_result_t planner_evaluate_target_ready_now(
    const planner_env_t *env, const planner_train_view_t *view,
    track_node *user_target, planner_workspace_t *ws, planner_eval_t *out);

int planner_prepare_launch_prefix(const planner_env_t *env,
                                  const planner_train_view_t *view,
                                  const route_plan_t *full_plan,
                                  int path_prefix_start_cursor,
                                  int path_dist_start_cursor,
                                  int allow_short_goal,
                                  int min_end_cursor,
                                  planner_workspace_t *ws,
                                  route_plan_t *out_prefix,
                                  int *out_reserved_end_cursor,
                                  int *out_switch_blocker_owner);

int planner_prepare_launch_strict(const planner_env_t *env,
                                  const planner_train_view_t *view,
                                  const route_plan_t *full_plan,
                                  int path_prefix_start_cursor,
                                  int path_dist_start_cursor,
                                  planner_workspace_t *ws,
                                  route_plan_t *out_prefix,
                                  int *out_reserved_end_cursor,
                                  int *out_switch_blocker_owner);

uint8_t planner_detect_deadlock(
    const planner_env_t *env, const planner_train_view_t *const *views,
    int view_count, int train_num, planner_deadlock_kind_t *out_kind,
    planner_deadlock_participants_t *out_parts);

uint8_t planner_deadlock_global_mask_from_local(
    const planner_deadlock_participants_t *parts, uint8_t local_mask);

int planner_choose_victim(const planner_deadlock_participants_t *parts,
                          uint8_t cycle_mask,
                          planner_deadlock_kind_t kind);

uint8_t planner_simulate_deadlock_unblocked_mask(
    const planner_env_t *env, const planner_train_view_t *victim,
    const planner_train_view_t *const *views, int view_count, uint8_t cycle_mask,
    track_node *yield_target, planner_workspace_t *ws);

int planner_pick_yield_target(const planner_env_t *env,
                              const planner_train_view_t *victim,
                              const planner_train_view_t *const *views,
                              int view_count,
                              uint8_t cycle_mask,
                              planner_workspace_t *ws,
                              track_node **out_target,
                              uint8_t *out_unblocked_mask,
                              planner_eval_t *out_eval,
                              planner_deadlock_pick_kind_t *out_kind);

#endif /* _planner_core_h_ */
