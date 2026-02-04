#ifndef _task_scheduler_h_
#define _task_scheduler_h_ 1

#include "ring_buffer.h"
#include "task.h"


#define PRIORITY_LEVELS 32

// Priority levels 
#define NAMESERVER_PRIORITY     31  // Highest priority - responds to all queries

// K3 client task priorities
// k3 requiremnts specifies priorities 3,4,5,6 with "smaller is higher"
// Mapped to: 28,27,26,25 to follow existing convention
#define K3_CLIENT_PRIORITY_3    28
#define K3_CLIENT_PRIORITY_4    27
#define K3_CLIENT_PRIORITY_5    26
#define K3_CLIENT_PRIORITY_6    25  


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
