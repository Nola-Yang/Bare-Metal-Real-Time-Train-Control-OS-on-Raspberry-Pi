#ifndef _position_h_
#define _position_h_ 1

#include <stdint.h>
#include "track_node.h"
#include "track.h"

/* Maximum number of trains tracked simultaneously */
#define MAX_POS_TRAINS 5

/* Number of sensors in the fixed loop (same for Track A and Track B) */
#define LOOP_SENSOR_COUNT 10

/* Speed-stabilisation parameters for the goto loop phase */
#define STABLE_TIME_ERR_US  600000LL  
#define STABLE_SENSOR_MIN   3         

/* ---------- Train route state ---------- */

typedef enum {
    TRAIN_STATE_UNKNOWN           = 0,  /* pos/dir/speed unknown; no operation on this train */
    TRAIN_STATE_KNOWN             = 1,  /* running via tr; pos/dir/speed known */
    TRAIN_STATE_STOPPING_TR       = 2,  /* tr 0 sent; physically decelerating */
    TRAIN_STATE_ON_ROUTE          = 3,  /* following planned route to target */
    TRAIN_STATE_STOPPING          = 4,  /* goto braking command issued */
    TRAIN_STATE_STOPPED           = 5,  /* fully stationary; pos known */
    TRAIN_STATE_LOOP_FIND_DIR     = 6,  /* position unknown; running until first sensor hit */
    TRAIN_STATE_RECOVERY_STOPPING = 7,  /* off-route deviation; stopping -> replan */
    TRAIN_STATE_STOPPING_GOTO     = 8, /* goto while running; stop sent -> replan */
    TRAIN_STATE_DEAD_TRACK        = 9, /* stopped on powerless track; waiting for manual push */
    TRAIN_STATE_WAIT_RESOURCE     = 10, /* route blocked by reservation; stopped and waiting */
} train_route_state_t;

/* ---------- Per-train position state ---------- */

typedef struct {
    int train_num;              /* -1 = empty slot */
    int train_ind;

    /* Current position */
    track_node *cur_sensor;     // most-recently triggered sensor node 
    uint64_t    cur_sensor_time;   //us
    int32_t     effective_v;    // EMA-corrected speed estimate (mm/s) 
    int         user_speed;     //0-14

    /* Prediction */
    track_node *pred_next_sensor;
    track_node *pred_alt_sensor;   /* first sensor in OTHER branch direction at next switch */
    track_node *pred_branch_node;  /* first BRANCH on path from cur_sensor; paired with pred_alt_sensor */
    uint64_t    pred_trigger_time;
    int64_t     last_time_err_us;  /* t_actual - t_predicted (us) */
    int32_t     last_dist_err_mm;  /* effective_v * last_time_err_us / 1e6 */

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

    /* 1 = forward loop direction.
     * 0 = reverse direction.
     * affects which target vs target->reverse. */
    int going_forward;
    uint8_t position_known;

    /* Original user-specified goto target, preserved across route execution
     * for off-route recovery.*/
    track_node *orig_user_target;
    int32_t     orig_target_offset;

    /* Last off-route mismatch snapshot for UI:
     * expected sensor vs actual hit sensor. */
    int         offroute_valid;
    track_node *offroute_expected_sensor;

    /* Timestamp (us) when route_state entered TRAIN_STATE_STOPPING.
     * Used by pos_on_tick() to fire the STOPPING → STOPPED transition. */
    uint64_t    stopping_since_us;

    /* WAIT_RESOURCE bookkeeping */
    uint64_t    next_replan_us;
    int         replan_retry_count;  /* exponential backoff retry counter */
    uint32_t    replan_rand_state;   /* LCG state for jitter randomization */

    /* cur_sensor_time + 2*(T1+T2), where T1/T2 are the
     * expected travel times to the next two sensors.  
     */
    uint64_t    dead_track_deadline_us;

    /* Per-speed EMA cache.
     * cached_v[s] holds the last calibrated effective_v for user speed s.
     * 0 = unset; fall back to speed_table_get_v(train, s).
     * Saved on each stop; restored on restart at the same speed
    */
    int32_t     cached_v[15];

    /* Distance remaining (mm) in the post-speed-change warm-up window.
     * EMA calibration is suppressed while this is > 0.
     * Decremented by the measured edge distance on each sensor trigger. */
    int32_t     speed_warmup_mm;

    /* Mid-route reversal state.
     * When midrev_active=1, the current target_sensor is the reversal stop
     * point.  After the train stops there, it reverses and continues to
     * midrev_final_target via the second-leg switches stored here. */
    int         midrev_active;
    track_node *midrev_sensor;        /* sensor that triggered the reversal stop */
    track_node *midrev_final_target;  /* ultimate destination after reversal */
    int32_t     midrev_final_offset;  /* offset past final target (mm) */
    int         midrev_sw_count;
    int         midrev_sw_nums[20];
    char        midrev_sw_dirs[20];
    int32_t     midrev_dist_after;    /* dist from reversal->reverse to final target */

    /* If 1: started via pos_start_direction_find; stop after direction confirmed
     * instead of planning a route to a target. */
    uint8_t     find_dir_only;

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

/* ---------- Public API ---------- */

void pos_init(void);

/* Called every time a sensor fires.
 * Tries to attribute the event to the correct tracked train. */
void pos_on_sensor_trigger(uint16_t sensor_id, uint64_t time_us);

/* Called on every 10 ms tick to handle predicted-sensor timeouts. */
void pos_on_tick(uint64_t now_us);

/* Register or update the speed of a tracked train (call after tr command). */
void pos_on_speed_change(int train_num, int user_speed);

/* Called when a physical reverse completes (rv command).
 * Flips going_forward and cur_sensor to keep position tracking consistent. */
void pos_on_reverse(int train_num);


/* Apply loop switch settings (SW7/8/14/9/6/15=S, SW11=C). */
void pos_apply_loop_switches(void);


/* Execute a goto */
int pos_goto(int train_num, track_node *target, int32_t offset_mm);

/* Start an UNKNOWN train moving to find direction; stop once direction is confirmed.
 * No destination is planned — train transitions to STOPPED at the second sensor.
 * Returns 1 on success, 0 if train is not UNKNOWN or slot unavailable. */
int pos_start_direction_find(int train_num);


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
int pos_queue_goto(int train_num, track_node *target, int32_t offset_mm);

void pos_mark_routes_dirty(void);

/* Find a track node by name.
 * Returns NULL if not found. */
track_node *pos_find_node(const char *name);

#endif /* _position_h_ */
