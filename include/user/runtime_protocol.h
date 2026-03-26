#ifndef _runtime_protocol_h_
#define _runtime_protocol_h_ 1

#include <stdint.h>
#include "mcp2515.h"

#define TRAIN_CMD_MAX_LEN 80
#define TRAIN_CMD_MAX_ARGS 8
#define TRAIN_CMD_TOKEN_MAX 16

typedef enum {
    TRAIN_CMD_NONE = 0,
    TRAIN_CMD_PARSE_ERROR = 1,
    TRAIN_CMD_QUIT = 2,
    TRAIN_CMD_TR = 3,
    TRAIN_CMD_SW = 4,
    TRAIN_CMD_RV = 5,
    TRAIN_CMD_LIGHT = 6,
    TRAIN_CMD_GOTO = 7,
    TRAIN_CMD_DEMO = 8,
    TRAIN_CMD_FINDPOS = 9,
    TRAIN_CMD_UNKNOWN = 10,
} train_command_type_t;

typedef enum {
    TRAIN_CMD_ERR_NONE = 0,
    TRAIN_CMD_ERR_USAGE_TR,
    TRAIN_CMD_ERR_USAGE_SW,
    TRAIN_CMD_ERR_USAGE_RV,
    TRAIN_CMD_ERR_USAGE_LIGHT,
    TRAIN_CMD_ERR_USAGE_GOTO,
    TRAIN_CMD_ERR_USAGE_DEMO,
    TRAIN_CMD_ERR_USAGE_FINDPOS,
    TRAIN_CMD_ERR_TRAIN_NOT_NUMBER,
    TRAIN_CMD_ERR_TRAIN_INVALID,
    TRAIN_CMD_ERR_SPEED_NOT_NUMBER,
    TRAIN_CMD_ERR_SPEED_RANGE,
    TRAIN_CMD_ERR_SWITCH_NOT_NUMBER,
    TRAIN_CMD_ERR_SWITCH_INVALID,
    TRAIN_CMD_ERR_SWITCH_DIR,
    TRAIN_CMD_ERR_LIGHT_NOT_NUMBER,
    TRAIN_CMD_ERR_LIGHT_RANGE,
    TRAIN_CMD_ERR_OFFSET_NOT_NUMBER,
    TRAIN_CMD_ERR_NODE_UNKNOWN,
    TRAIN_CMD_ERR_DEMO_ARG_NOT_NUMERIC,
    TRAIN_CMD_ERR_INVALID_GOTO_SPEED,
    TRAIN_CMD_ERR_UNKNOWN,
} train_command_error_t;

typedef struct {
    train_command_type_t type;
    train_command_error_t error;
    int argc;
    char argv[TRAIN_CMD_MAX_ARGS][TRAIN_CMD_TOKEN_MAX];
    char raw_cmdline[TRAIN_CMD_MAX_LEN];
    int train;
    int value;
    int aux;
    int target_idx;
    int32_t offset_mm;
    char dir;
} train_command_t;

typedef enum {
    RUNTIME_EVENT_COMMAND = 0,
    RUNTIME_EVENT_RUNTIME_READY = 1,
    RUNTIME_EVENT_SENSOR_HIT = 2,
    RUNTIME_EVENT_SWITCH_ACK = 3,
    RUNTIME_EVENT_FAST_TICK = 4,
    RUNTIME_EVENT_REPLAN_TICK = 5,
    RUNTIME_EVENT_DEMO_TICK = 6,
    RUNTIME_EVENT_SWITCH_SETTLE_TICK = 7,
    RUNTIME_EVENT_RV_REQUEST = 8,
    RUNTIME_EVENT_RV_COMPLETE = 9,
    RUNTIME_EVENT_CAN_FRAME = 10,
} runtime_event_type_t;

typedef struct {
    runtime_event_type_t type;
    train_command_t command;
    can_frame_t frame;
    uint64_t now_us;
    uint64_t arrival_us;
    int train;
    int delay_ticks;
    int sw_num;
    uint16_t sensor_id;
    uint8_t sensor_state;
    char sw_dir;
} runtime_event_t;

typedef struct {
    int status;
    int train;
    int delay_ticks;
} runtime_reply_t;

#endif /* _runtime_protocol_h_ */
