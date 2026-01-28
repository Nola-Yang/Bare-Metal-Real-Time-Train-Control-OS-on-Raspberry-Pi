#ifndef _task_scheduler_h_
#define _task_scheduler_h_ 1

#include "ring_buffer.h"
#include "task.h"


#define PRIORITY_LEVELS 32

// Priority levels for k2 tasks 
#define NAMESERVER_PRIORITY     31  // Highest priority - responds to all queries
#define RPS_SERVER_PRIORITY     30  
#define RPS_CLIENT_PRIORITY     29  


typedef struct {
    RingBuffer_t *queues;
    uint32_t size;
} TaskScheduler_t;


extern TaskScheduler_t GlobalTaskScheduler;


void init_global_task_scheduler(RingBuffer_t *queues, uint32_t size);

void global_task_scheduler_add_task(TaskDescriptor_t *task);

void global_task_scheduler_remove_task(TaskDescriptor_t *task);

// Get highest priority ready task
bool get_next_task(TaskScheduler_t *task_scheduler, TaskDescriptor_t **result);

// Pop the next runnable task from the highest-priority non-empty queue
TaskDescriptor_t *schedule();

#endif
