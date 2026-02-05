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

// K3 client task priorities
// k3 requiremnts specifies priorities 3,4,5,6 with "smaller is higher"
// Mapped to: 28,27,26,25 to follow existing convention
#define K3_CLIENT_PRIORITY_3     3
#define K3_CLIENT_PRIORITY_4     4
#define K3_CLIENT_PRIORITY_5     5
#define K3_CLIENT_PRIORITY_6     6  


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
