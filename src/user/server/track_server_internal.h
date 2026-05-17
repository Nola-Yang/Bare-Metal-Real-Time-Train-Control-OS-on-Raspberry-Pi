#ifndef _track_server_internal_h_
#define _track_server_internal_h_ 1

#include "track.h"
#include "mcp2515.h"

typedef enum {
    TRACK_REQ_INIT = 0,
    TRACK_REQ_SET_SPEED = 1,
    TRACK_REQ_SEND_DIRECTION = 2,
    TRACK_REQ_REVERSE = 3,
    TRACK_REQ_SET_SWITCH = 4,
    TRACK_REQ_SET_LIGHT = 5,
    TRACK_REQ_START_REVERSE = 6,
    TRACK_REQ_COMPLETE_REVERSE = 7,
    TRACK_REQ_LOG_SENSOR = 8,
    TRACK_REQ_UPDATE_SWITCH = 9,
    TRACK_REQ_CAN_FRAME = 10,
    TRACK_REQ_RESET_STARTUP = 11,
} track_request_type_t;

typedef struct {
    int type;
    int can_server_tid;
    int train;
    int value;
    uint16_t sensor_id;
    uint8_t sensor_state;
    uint64_t now_us;
    char dir;
    can_frame_t frame;
} TrackRequest_t;

typedef struct {
    int status;
} TrackReply_t;

void track_bind_can_server(int can_server_tid);
void track_init_graph(void);
void track_local_log_sensor(uint16_t sensor_id, uint64_t time_us, uint8_t state);
void track_local_update_switch(int sw_num, char state);
int track_local_send_direction(int train_num, uint8_t dir);
int track_local_set_speed(int train, int speed);
int track_local_reverse(int train_num);
int track_local_set_switch(int sw_num, char dir);
int track_local_set_light(int train, int on);
int track_local_complete_reverse(int train_num);
int track_local_start_reverse(int train_num);
void track_local_reset_state(void);
void track_local_bootstrap_defaults(void);

#endif /* _track_server_internal_h_ */
