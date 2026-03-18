#ifndef _train_control_h_
#define _train_control_h_ 1

#include "mcp2515.h"
#include "task_scheduler.h"

// Message types for train control task
#define TRAIN_MSG_CHAR      0   // Keyboard character input
#define TRAIN_MSG_CAN_FRAME 1   // CAN frame received
#define TRAIN_MSG_TICK      2   // Periodic UI tick
#define TRAIN_MSG_RV_REQUEST  3   // Reverse delay task requests parameters
#define TRAIN_MSG_RV_COMPLETE 4
#define TRAIN_MSG_DEMO_TICK   5   // 1s periodic demo tick
#define TRAIN_POS_TICK        6   // Update the position of the train


typedef struct {
    int type;
    char ch;
    int train;
    can_frame_t frame;
} TrainControlMsg_t;

typedef struct {
    int status;
    int train;
    int delay_ticks;
} TrainControlReply_t;

// Main train control task
void train_control_task(void);

// Keyboard courier task
void keyboard_courier_task(void);

// UI tick task
void ui_tick_task(void);

// Demo tick task (1s interval)
void demo_tick_task(void);

// Reverse delay courier task
void rv_delay_task(void);

#endif /* _train_control_h_ */
