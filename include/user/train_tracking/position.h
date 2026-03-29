#ifndef _position_h_
#define _position_h_ 1

#include <stdint.h>
#include "track_node.h"
#include "track.h"

/* Maximum number of trains tracked simultaneously */
#define MAX_POS_TRAINS 5
#define DEADLOCK_MAX_TRAINS 3

/* Physical train body length (mm). */
#define TRAIN_BODY_MM 200

/* ---------- Train route state ---------- */

typedef enum {
    TRAIN_STATE_UNKNOWN           = 0,  /* pos/dir/speed unknown; no operation on this train */
    TRAIN_STATE_KNOWN             = 1,  /* running via tr; pos/dir/speed known */
    TRAIN_STATE_STOPPING_TR       = 2,  /* tr 0 sent; physically decelerating */
    TRAIN_STATE_ON_ROUTE          = 3,  /* following planned route to target */
    TRAIN_STATE_STOPPING          = 4,  /* goto braking command issued */
    TRAIN_STATE_STOPPED           = 5,  /* fully stationary; pos known */
    TRAIN_STATE_FIND_POS     = 6,  /* position unknown; running until first sensor hit */
    TRAIN_STATE_RECOVERY_STOPPING = 7,  /* off-route deviation; stopping -> replan */
    TRAIN_STATE_STOPPING_GOTO     = 8, /* goto while running; stop sent -> replan */
    TRAIN_STATE_DEAD_TRACK        = 9, /* dead track detected; stop is sent and bootstrap may re-arm immediately */
    TRAIN_STATE_WAIT_RESOURCE     = 10, /* route blocked by reservation; stopped and waiting */
    TRAIN_STATE_WAIT_SWITCH_SETTLE = 11, /* switches set; waiting for turnout settle before launch */
} train_route_state_t;

typedef enum {
    POS_SWITCH_SETTLE_NONE = 0,
    POS_SWITCH_SETTLE_NORMAL = 1,
    POS_SWITCH_SETTLE_REVERSED = 2,
} pos_switch_settle_mode_t;

typedef enum {
    POS_WAIT_NONE = 0,
    POS_WAIT_PRELAUNCH_ROUTE = 1,
    POS_WAIT_RESUME_ROUTE = 2,
    POS_WAIT_MIDREV_SECOND_LEG = 3,
} pos_wait_mode_t;

typedef enum {
    POS_TARGET_COL_NONE  = 0,
    POS_TARGET_COL_FINAL = 1,
    POS_TARGET_COL_STAGE = 2,
} pos_target_col_t;

/* ---------- Prediction sub-state ---------- */

typedef struct {
    track_node *next_sensor;      
    track_node *alt_sensor;        /* first sensor in OTHER branch direction at next switch */
    track_node *branch_node;       /* first BRANCH on path from cur_sensor; paired with alt_sensor */
    uint64_t    trigger_time;      
    uint8_t     skipped_sensor_count; /* consecutive predicted sensors skipped since last real hit */
    int64_t     last_time_err_us;  /* t_actual - t_predicted (us) */
    int32_t     last_dist_err_mm;  /* effective_v * last_time_err_us / 1e6 */
} pos_pred_t;

/* ---------- Mid-route reversal sub-state ---------- */

typedef struct {
    int        active;             /* 1 = reversal is pending; 0 = no reversal */
    track_node *sensor;            /* sensor to stop at before reversing */
    track_node *final_target;      /* ultimate destination after reversal */
    int32_t    final_offset;       /* offset past final target (mm) */
    int        sw_count;
    int        sw_nums[20];        
    char       sw_dirs[20];       
    int32_t    dist_after;         /* dist from reversal->reverse to final target */
    uint16_t   path2[TRACK_MAX];   /* second-leg node path */
    int        path2_count;
} pos_midrev_t;

/* ---------- WAIT_RESOURCE backoff sub-state ---------- */

typedef struct {
    uint64_t next_us;       
    int      retry_count;    /* exponential backoff retry counter */
    uint32_t rand_state;     /* LCG state for jitter randomization */
    uint32_t seen_generation; /* last reservation-change generation observed */
    uint8_t  blocker_mask;   /* bitmask of trains currently blocking this replan */
    uint8_t  wait_mode;      /* committed-route wait strategy */
    uint8_t  need_initial_reverse; /* saved launch metadata for prelaunch waits */
    track_node *launch_origin; /* chosen origin for the committed launch route */
} pos_replan_t;

typedef struct {
    uint8_t    valid;
    track_node *orig_target;
    int32_t    orig_offset_mm;
} pos_dead_track_recover_t;

typedef struct {
    uint8_t    valid;
    track_node *resume_target;
    int32_t    resume_offset_mm;
    track_node *yield_target;
    uint8_t    wait_start_mask;
    uint8_t    parked_at_yield;
    uint64_t   parked_since_us;
} pos_deadlock_recover_t;

typedef struct {
    uint8_t    active;
    uint8_t    unresolved;
    int        victim_train;
    int        cycle_trains[DEADLOCK_MAX_TRAINS];
    int        cycle_count;
    track_node *blocked_target;
    track_node *yield_target;
    track_node *resume_target;
    uint64_t   expire_us;
} pos_deadlock_notice_t;

/* ---------- Per-train position state ---------- */

typedef struct {
    int train_num;              /* -1 = empty slot */
    int train_ind;

    /* Current position */
    track_node *cur_sensor;     /* most-recently triggered sensor node */
    uint64_t    cur_sensor_time;   // us 
    int32_t     effective_v;    /* EMA-corrected speed estimate (mm/s) */
    int         user_speed;     /* 0-14 */
    int         goto_speed;

    /* Prediction */
    pos_pred_t  pred;

    /* Route / target */
    train_route_state_t route_state;
    track_node *target_sensor;
    int32_t     target_offset_mm;  /* additional mm past target_sensor */
    int32_t     dist_to_target_mm;

    /* Deferred goto: target stored here until speed is stable on the loop.
     * Cleared when the route is actually executed. */
    track_node *pending_target;
    int32_t     pending_offset_mm;

    /* One-slot queued goto request while a goto is already active.
     * last-write-wins when multiple queued requests arrive. */
    track_node *queued_target;
    int32_t     queued_offset_mm;
    int         queued_valid;

    /* 1 = forward direction; 0 = reverse direction. */
    int going_forward;
    int prev_going_forward;
    uint8_t position_known;

    /* Original user-specified goto target, preserved across route execution
     * for off-route recovery. */
    track_node *orig_user_target;
    int32_t     orig_target_offset;

    /* Last off-route mismatch snapshot for UI:
     * expected sensor vs actual hit sensor. */
    int         offroute_valid;
    track_node *offroute_expected_sensor;

    /* Timestamp (us) when route_state entered TRAIN_STATE_STOPPING.
     * Used by pos_on_tick() to fire the STOPPING → STOPPED transition. */
    uint64_t    stopping_since_us;

    /* 1 when the last STOPPED state was reached after physically hitting the
     * route target sensor; reverse replans should anchor at cur_sensor->reverse. */
    uint8_t     stopped_on_target_hit;

    /* UI latch for the last logical stop target: keep the reached target
     * column highlighted until a new destination overrides it. */
    uint8_t     parked_target_col;

    /* Deferred launch after issuing switch commands. */
    uint64_t    switch_settle_due_us;
    uint8_t     switch_settle_mode;

    /* WAIT_RESOURCE backoff */
    pos_replan_t replan;

    /* Deadline for observing the next predicted progress sensor.
     * Typically now + DEAD_TRACK_DEADLINE_MULTIPLIER * T1. */
    uint64_t    dead_track_deadline_us;
    uint64_t    dead_track_bootstrap_due_us;

    /* Per-speed EMA cache.
     * cached_v[s] holds the last calibrated effective_v for user speed s.
     * 0 = unset; fall back to speed_table_get_v(train, s).
     * Saved on each stop; restored on restart at the same speed. */
    int32_t     cached_v[15];

    /* Distance remaining (mm) in the post-speed-change warm-up window.
     * EMA calibration is suppressed while this is > 0.
     * Decremented by the measured edge distance on each sensor trigger. */
    int32_t     speed_warmup_mm;

    /* Acceleration model (0 → constant speed).
     * accel_a_eff : fixed mm/s² constant seeded from GOTO_ACCEL_MM_S2.
     * is_accelerating : 1 while the train is in the ramp-up phase;
     *                   cleared automatically when full speed is reached.
     * accel_start_us  : wall-clock time when the train actually begins
     *                   moving (= CAN command time + GO_LATENCY_US). */
    int32_t     accel_a_eff;
    uint8_t     is_accelerating;
    uint64_t    accel_start_us;
    uint8_t     awaiting_post_launch_sensor; /* 1 until the first hit after a goto launch */
    uint8_t     force_offroute_on_next_sensor;
    uint8_t     dead_track_rescue_pending;
    uint8_t     dead_track_warn_active; /* UI latch: dead-track rebootstrap is in progress. */
    pos_dead_track_recover_t dead_track_recover;
    pos_deadlock_recover_t deadlock_recover;

    /* Mid-route reversal */
    pos_midrev_t midrev;

    /* Path-based distance tracking.
     * route_path[0..count-1]: node indices of the current leg in forward order.
     * route_path_cursor: index of the last confirmed sensor (cur_sensor) in route_path.
     * route_rem_tick_us: timestamp (us) of the last dist_to_target_mm decrement.
    */
    uint16_t route_path[TRACK_MAX];
    int      route_path_count;
    int      route_path_cursor;
    int      route_reserved_end_cursor;
    uint64_t route_rem_tick_us;
    uint32_t authority_seen_generation;
    uint64_t authority_next_us;

    /* If 1: started via pos_start_find_pos; stop after the first sensor hit
     * instead of planning a route to a target. */
    uint8_t  stop_after_find_pos;

} train_pos_t;

/* ---------- Route plan (from Dijkstra route planning) ---------- */

typedef struct {
    int   sw_nums[20];
    char  sw_dirs[20];
    int   sw_count;
    int32_t total_dist_mm;        /* accumulated path distance from start to target */

    track_node *chosen_target;

    /* Optional mid-route reversal */
    int        has_reversal;           /* 1 = route has one mid-point reversal */
    track_node *reversal_sensor;       /* sensor to stop at before reversing */
    int32_t    dist_to_reversal_mm;    /* first-leg distance (start → reversal) */
    int        sw_count2;              /* switch count for second leg */
    int        sw_nums2[20];           /* switch numbers after reversal */
    char       sw_dirs2[20];           /* switch directions after reversal */
    int32_t    dist_after_reversal_mm; /* second-leg distance (reversal→reverse → target) */

    /* Reconstructed node indices of planned legs for reservation. */
    int        path_count;
    uint16_t   path_nodes[TRACK_MAX];
    int        path_count2;
    uint16_t   path_nodes2[TRACK_MAX];
} route_plan_t;

typedef enum {
    POS_TARGET_UNREACHABLE = 0,
    POS_TARGET_BLOCKED     = 1,
    POS_TARGET_READY       = 2,
} pos_target_query_status_t;

typedef struct {
    pos_target_query_status_t status;
    route_plan_t             plan;
    uint8_t                  blocker_mask;
} pos_target_query_t;

typedef enum {
    POS_GAME_EVENT_NONE       = 0,
    POS_GAME_EVENT_SENSOR_HIT = 1,
    POS_GAME_EVENT_GOAL_STOP  = 2,
} pos_game_event_type_t;

typedef struct {
    uint32_t              seq;
    pos_game_event_type_t type;
    int                   train_num;
    uint16_t              sensor_num;
    uint64_t              time_us;
} pos_game_event_t;

/* ---------- Public API ---------- */

void pos_init(void);

/* Called every time a sensor fires.
 * Tries to attribute the event to the correct tracked train. */
void pos_on_sensor_trigger(uint16_t sensor_id, uint64_t time_us);

/* Called on every 10 ms tick to handle predicted-sensor timeouts. */
void pos_on_tick(uint64_t now_us);

void pos_replan_on_tick(uint64_t now_us);

void pos_on_switch_settle_tick(uint64_t now_us);

/* Register or update the speed of a tracked train (call after tr command). */
void pos_on_speed_change(int train_num, int user_speed);

/* Called when a physical reverse completes (rv command).
 * Flips going_forward and cur_sensor to keep position tracking consistent. */
void pos_on_reverse(int train_num);




/* Execute a goto */
int pos_goto(int train_num, track_node *target, int speed_level, int32_t offset_mm);

/* Start an UNKNOWN train moving to find its current sensor anchor.
 * No destination is planned — train transitions to STOPPED after the first hit.
 * Returns 1 on success, 0 if train is not UNKNOWN or slot unavailable. */
int pos_start_find_pos(int train_num, int speed_level);


/* Returns 1 if the given train has an active goto in progress; 0 otherwise. */
int pos_is_train_goto_active(int train_num);

/* Returns 1 if the train's current sensor anchor and travel direction are known. */
int pos_is_train_position_known(int train_num);

/* Return pointer to the position state for a given train (-1 if none). */
train_pos_t *pos_get(int train_num);

/* Return pointer to the i-th slot in the position table (0 .. MAX_POS_TRAINS-1).
 * Returns NULL if i is out of range. */
train_pos_t *pos_get_by_index(int i);

/* Queue a goto for an already-active train. last-write-wins. */
int pos_queue_goto(int train_num, track_node *target, int speed_level, int32_t offset_mm);

int pos_read_game_events(uint32_t *io_seq, pos_game_event_t *out, int max_events);

pos_target_query_status_t pos_query_target(int train_num, track_node *target,
                                           pos_target_query_t *out);

void pos_mark_routes_dirty(void);

void pos_reset_dead_train(int train_num);

void pos_get_deadlock_notice(pos_deadlock_notice_t *out);

#endif /* _position_h_ */
