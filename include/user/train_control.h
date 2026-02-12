#ifndef _train_control_h_
#define _train_control_h_ 1

#include "mcp2515.h"
#include "task_scheduler.h"

// Message types for train control task
#define TRAIN_MSG_CHAR      0   // Keyboard character input
#define TRAIN_MSG_CAN_FRAME 1   // CAN frame received
#define TRAIN_MSG_TICK      2   // Periodic UI tick


typedef struct {
    int type;
    char ch;
    can_frame_t frame;
} TrainControlMsg_t;

typedef struct {
    int status;
} TrainControlReply_t;

// Main train control task
void train_control_task(void);

// Keyboard courier task
void keyboard_courier_task(void);

// UI tick task
void ui_tick_task(void);

#endif /* _train_control_h_ */
