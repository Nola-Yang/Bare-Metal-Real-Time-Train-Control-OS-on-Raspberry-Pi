#ifndef _task_scheduler_h_
#define _task_scheduler_h_ 1

#include "task_list.h"


#define PRIORITY_LEVELS 32

// Priority levels 
#define NAMESERVER_PRIORITY           0  // Highest priority - responds to all queries
#define CLOCK_NOTIFIER_PRIORITY       1  
#define CLOCK_SERVER_PRIORITY         2
#define FIRST_USER_TASK_PRIORITY     30
#define IDLE_TASK_PRIORITY           31

// Sever priority

#define IO_NOTIFIER_PRIORITY          1  // Notifiers: highest after nameserver
#define TERMINAL_SERVER_PRIORITY      2  // Same as clock server
#define CAN_SERVER_PRIORITY           2  // Same as clock server
#define UI_SERVER_PRIORITY            2  // Serialized terminal UI output
#define RUNTIME_CAN_INGRESS_PRIORITY  3
#define TRACK_SERVER_PRIORITY         3
#define POSITION_SERVER_PRIORITY      4
#define RUNTIME_FAST_TICK_PRIORITY    4
#define TRAIN_CONTROL_PRIORITY        5
#define RUNTIME_CORE_PRIORITY         5
#define KEYBOARD_BUFFER_PRIORITY      5
#define RUNTIME_SLOW_TICK_PRIORITY    6
#define DEMO_SERVER_PRIORITY          6
#define GAME_SERVER_PRIORITY          6
#define UI_TICK_PRIORITY              6
#define TRAIN_COURIER_PRIORITY        6
#define DEADLOCK_SERVER_PRIORITY      7
#define DEADLOCK_TICK_PRIORITY        7

#define CAN_NOTIFIER_PRIORITY  1

#define TERM_NOTIFIER_PRIORITY  1
#define TERM_SERVER_PRIORITY    2

typedef struct {
    TaskList_t *task_lists;
    uint32_t size;
} TaskScheduler_t;


extern TaskScheduler_t GlobalTaskScheduler;


void init_global_task_scheduler(TaskList_t *task_lists, uint32_t size);

void global_task_scheduler_add_task(TaskDescriptor_t *task);

void global_task_scheduler_remove_task(TaskDescriptor_t *task);

// Pop the next runnable task from the highest-priority non-empty queue
TaskDescriptor_t *schedule();

#endif
