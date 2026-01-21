#include "task_scheduler.h"
#include "task.h"
#include "ring_buffer.h"


TaskScheduler_t GlobalTaskScheduler;


void init_global_task_scheduler(RingBuffer_t *queues, uint32_t size) {
    GlobalTaskScheduler.queues = queues;
    GlobalTaskScheduler.size = size;
}

void global_task_scheduler_add_task(TaskDescriptor_t *task) {
    ring_buf_append(&(GlobalTaskScheduler.queues[task->priority]), &task);
}

// Remove task from front of its priority queue (current running task)
void global_task_scheduler_remove_task(TaskDescriptor_t *task) {
    RingBuffer_t *queue = &(GlobalTaskScheduler.queues[task->priority]);
    
    TaskDescriptor_t *removed_task;
    ring_buf_pop_left(queue, &removed_task);
}

bool get_next_task(TaskScheduler_t *task_scheduler, TaskDescriptor_t **result) {
    int32_t queue_count = task_scheduler->size;
    RingBuffer_t *queues = task_scheduler->queues;

    for (int32_t i = queue_count - 1; i >= 0; --i) {
        if (is_ring_buf_empty(&queues[i])) continue;

        RingBuffer_t *queue = &(queues[i]);
        ring_buf_from_tail(queue, result, 0, true);

        return true;
    }

    return false;
}

TaskDescriptor_t * schedule() {
    TaskDescriptor_t *current_task = get_current_task();
    TaskDescriptor_t *temp_task;

    RingBuffer_t *queue = &(GlobalTaskScheduler.queues[current_task->priority]);
    ring_buf_pop_left(queue, &temp_task);
    ring_buf_append(queue, &temp_task);

    TaskDescriptor_t *next_task;

    bool success = get_next_task(&GlobalTaskScheduler, &next_task);
    if (!success || current_task == next_task) return current_task;

    //TODO: context switch

    return next_task;
}

