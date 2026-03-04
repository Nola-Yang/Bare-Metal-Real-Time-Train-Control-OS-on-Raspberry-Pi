#ifndef _position_h_
#define _position_h_ 1

#include <stdint.h>
#include "track_node.h"
#include "track.h"

/* Maximum number of trains tracked simultaneously */
#define MAX_POS_TRAINS 4

/* Number of sensors in the fixed loop (same for Track A and Track B) */
#define LOOP_SENSOR_COUNT 10

/* Speed-stabilisation parameters for the goto loop phase */
#define STABLE_TIME_ERR_US  10000000LL  
#define STABLE_SENSOR_MIN   3         

/* ---------- Train route state ---------- */

typedef enum {
    TRAIN_STATE_UNKNOWN           = 0,  /* pos/dir/speed unknown; no operation on this train */
    TRAIN_STATE_KNOWN             = 1,  /* running via tr; pos/dir/speed known */
    TRAIN_STATE_STOPPING_TR       = 2,  /* tr 0 sent; physically decelerating */
    TRAIN_STATE_ON_ROUTE          = 3,  /* following planned route to target */
    TRAIN_STATE_STOPPING          = 4,  /* goto braking command issued */
    TRAIN_STATE_STOPPED           = 5,  /* fully stationary; pos known */
    TRAIN_STATE_LOOP_STABILIZE    = 6,  /* on loop, running; waiting for speed to stabilise */
    TRAIN_STATE_LOOP_FIND_DIR     = 7,  /* on loop, running; need two sensors to confirm dir */
    TRAIN_STATE_RECOVERY_STOPPING = 8,  /* off-route deviation; stopping -> ENTER_LOOP */
    TRAIN_STATE_ENTER_LOOP        = 9,  /* pos/dir known, stationary; driving back to loop */
    TRAIN_STATE_STOPPING_GOTO     = 10, /* goto while running; stop sent -> ENTER_LOOP */
    TRAIN_STATE_DEAD_TRACK        = 11, /* stopped on powerless track; waiting for manual push */
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

    /* Consecutive sensors with acceptable prediction error (LOOP_STABILIZE).
     * Reset to 0 on any unstable reading. */
    int stable_sensor_count;

    /* 1 = forward loop direction.
     * 0 = reverse direction.
     * affects which target vs target->reverse. */
    int going_forward;

    /* Original user-specified goto target, preserved across route execution
     * for off-route recovery.*/
    track_node *orig_user_target;
    int32_t     orig_target_offset;

    /* Last planned route from loop to destination (for UI display). */
    int         last_plan_valid;
    track_node *last_plan_loop_start;
    track_node *last_plan_target;
    int         last_plan_sw_count;
    int         last_plan_sw_nums[20];
    char        last_plan_sw_dirs[20];

    /* Last off-route mismatch snapshot for UI:
     * expected sensor vs actual hit sensor. */
    int         offroute_valid;
    track_node *offroute_expected_sensor;
    track_node *offroute_actual_sensor;

    /* Timestamp (us) when route_state entered TRAIN_STATE_STOPPING.
     * Used by pos_on_tick() to fire the STOPPING → STOPPED transition. */
    uint64_t    stopping_since_us;

    /* cur_sensor_time + 2*(T1+T2), where T1/T2 are the
     * expected travel times to the next two sensors.  
     */
    uint64_t    dead_track_deadline_us;

    /* Speed used for the loop phase */
    int         goto_speed;

    /* Per-speed EMA cache.
     * cached_v[s] holds the last calibrated effective_v for user speed s.
     * 0 = unset; fall back to SPEED_V_MM_S[s] from the polynomial table.
     * Saved on each stop; restored on restart at the same speed
    */
    int32_t     cached_v[15];

    /* Distance remaining (mm) in the post-speed-change warm-up window.
     * EMA and edge-factor calibration are suppressed while this is > 0.
     * Decremented by the measured edge distance on each sensor trigger. */
    int32_t     speed_warmup_mm;

} train_pos_t;

/* ---------- Route plan (from plan_route BFS) ---------- */

typedef struct {
    track_node *loop_exit_branch; /* first branch off the loop */
    int   sw_nums[20];
    char  sw_dirs[20];
    int   sw_count;
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
int pos_goto(int train_num, track_node *target, int32_t offset_mm, int goto_speed);


/* Returns 1 if the given train has an active goto in progress; 0 otherwise. */
int pos_is_train_goto_active(int train_num);

/* Return pointer to the position state for a given train (-1 if none). */
train_pos_t *pos_get(int train_num);

/* Return pointer to the i-th slot in the position table (0 .. MAX_POS_TRAINS-1).
 * Returns NULL if i is out of range. */
train_pos_t *pos_get_by_index(int i);

/* Find a sensor node by name.
 * Returns NULL if not found. */
track_node *pos_find_sensor(const char *name);

#endif /* _position_h_ */
