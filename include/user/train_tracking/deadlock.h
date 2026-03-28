#ifndef _deadlock_h_
#define _deadlock_h_ 1

#include <stdint.h>
#include "position.h"
#include "track.h"

typedef struct {
    int        sw_nums[20];
    char       sw_dirs[20];
    int        sw_count;
    int32_t    total_dist_mm;
    int        chosen_target_idx;
    int        has_reversal;
    int        reversal_sensor_idx;
    int32_t    dist_to_reversal_mm;
    int        sw_count2;
    int        sw_nums2[20];
    char       sw_dirs2[20];
    int32_t    dist_after_reversal_mm;
    int        path_count;
    uint16_t   path_nodes[TRACK_MAX];
    int        path_count2;
    uint16_t   path_nodes2[TRACK_MAX];
} deadlock_route_plan_t;

typedef struct {
    int                 train_num;
    int                 train_ind;
    train_route_state_t route_state;
    int                 goto_speed;
    int                 cur_sensor_idx;
    int                 origin0_idx;
    int                 origin1_idx;
    int                 goal_idx;
    int32_t             goal_offset_mm;
    uint8_t             blocker_mask;
    int                 resume_target_idx;
    int32_t             resume_offset_mm;
    int                 yield_target_idx;
    uint8_t             parked_at_yield;
    uint8_t             wait_start_mask;
    uint8_t             yield_history_count;
    int                 yield_history_idx[DEADLOCK_YIELD_HISTORY_MAX];
} deadlock_participant_snapshot_t;

typedef struct {
    uint64_t                        now_us;
    uint32_t                        traffic_generation;
    uint32_t                        switch_generation;
    uint8_t                         auto_dispatching_targets;
    int                             reservation_owner[TRACK_MAX];
    char                            switch_state[MAX_SWITCHES];
    int                             participant_count;
    deadlock_participant_snapshot_t participants[DEADLOCK_MAX_TRAINS];
} deadlock_snapshot_t;

typedef enum {
    DEADLOCK_RESULT_NONE = 0,
    DEADLOCK_RESULT_NOTICE_ONLY = 1,
    DEADLOCK_RESULT_REROUTE = 2,
} deadlock_result_action_t;

typedef struct {
    deadlock_result_action_t        action;
    uint64_t                        snapshot_now_us;
    uint32_t                        traffic_generation;
    uint32_t                        switch_generation;
    uint8_t                         auto_dispatching_targets;
    int                             participant_count;
    deadlock_participant_snapshot_t participants[DEADLOCK_MAX_TRAINS];
    int                             victim_train;
    int                             cycle_count;
    int                             cycle_trains[DEADLOCK_MAX_TRAINS];
    int                             blocked_target_idx;
    int                             yield_target_idx;
    int                             resume_target_idx;
    int32_t                         resume_offset_mm;
    uint8_t                         wait_start_mask;
    int                             chosen_origin_idx;
    int                             need_initial_reverse;
    deadlock_route_plan_t           route_plan;
} deadlock_result_t;

int deadlock_plan_from_snapshot(const deadlock_snapshot_t *snapshot,
                                deadlock_result_t *out);

void pos_deadlock_build_snapshot(deadlock_snapshot_t *out, uint64_t now_us);
int pos_deadlock_apply_result(const deadlock_result_t *result);

#endif /* _deadlock_h_ */
