#ifndef _track_h_
#define _track_h_ 1

#include <stdint.h>

#define MAX_SWITCHES 22
#define SENSOR_LOG_SIZE 16
#define MAX_ACTIVE_TRAINS 8

typedef struct {
    char state;           // 'S', 'C', or '?'
    uint64_t last_update_us;
} switch_entry_t;

typedef struct {
    uint16_t sensor_id;
    uint64_t time_us;
    uint8_t state;  // 0=leaving, 1=entering
} sensor_entry_t;

typedef struct {
    int train_num;           // -1 = empty slot
    int speed;
    int direction;           // 0=forward, 1=reverse
    int rv_state;            // 0=idle, 1=wait_stop
    int rv_prev_speed;
} train_state_t;

// Initialize track module with server TIDs
void track_init(int can_server_tid, int term_server_tid);

// State management functions
void track_log_sensor(uint16_t sensor_id, uint64_t time_us, uint8_t state);
void track_update_switch(int sw_id, char state);
void track_update_speed(int train, int speed);
void track_update_direction(int train, int direction);
const sensor_entry_t* track_get_sensor_log(int *head);
const switch_entry_t* track_get_switch_state(void);
const train_state_t* track_get_trains(void);

// Switch number mapping (user number 1-18, 153-156 to array index 0-21)
int track_switch_to_index(int sw_num);
int track_index_to_switch(int index);
int track_is_valid_switch(int sw_num);

// Control functions (send commands via CAN server)
void track_set_speed(int train, int speed);
void track_reverse(int train);
void track_set_switch(int sw, char dir);
void track_set_light(int train, int on);

// Reverse state machine
int track_start_reverse(int train);
void track_complete_reverse(int train_num);

#endif /* _track_h_ */
