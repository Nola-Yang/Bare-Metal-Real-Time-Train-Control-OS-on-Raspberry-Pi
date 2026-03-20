#ifndef _train_control_h_
#define _train_control_h_ 1

#include "mcp2515.h"
#include "task_scheduler.h"

// Message types for train control task
#define TRAIN_MSG_COMMAND   0   // Complete command line input
#define TRAIN_MSG_CAN_FRAME 1   // CAN frame received
#define TRAIN_MSG_RV_REQUEST  3   // Reverse delay task requests parameters
#define TRAIN_MSG_RV_COMPLETE 4
#define TRAIN_MSG_DEMO_TICK   5   // 1s periodic demo tick
#define TRAIN_POS_TICK        6   // Update the position of the train
#define TRAIN_POS_REPLAN_TICK 7   // Replan the route for the trains
#define TRAIN_POS_SWITCH_SETTLE_TICK 8   // Complete deferred post-switch launches
#define TRAIN_MSG_RUNTIME_READY 9   // Runtime worker finished initialization
#define TRAIN_MSG_RETRY_DEAD_TRAIN 10   // Retry a queued dead-train recovery

#define TRAIN_CMD_MAX_LEN 80

typedef struct {
    int type;
    int train;
    char cmdline[TRAIN_CMD_MAX_LEN];
    can_frame_t frame;
    uint64_t arrival_us;   // frame arrival time stamped by courier
} TrainControlMsg_t;

typedef struct {
    int status;
    int train;
    int delay_ticks;
} TrainControlReply_t;

// Main train control task
void train_control_task(void);

// Command input task
void command_input_task(void);

// Periodic UI refresh task
void ui_tick_task(void);

#endif /* _train_control_h_ */
