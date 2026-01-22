#include "task_scheduler.h"
#include "task.h"
#include "ring_buffer.h"
#include "util.h"


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
    if (is_ring_buf_empty(queue)) return;

    uint32_t remaining = queue->count;
    bool removed = false;

    while (remaining-- > 0) {
        TaskDescriptor_t *candidate;
        if (!ring_buf_pop_left(queue, &candidate)) break;

        if (!removed && candidate == task) {
            removed = true;
            continue;
        }

        ring_buf_append(queue, &candidate);
    }
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
    int32_t queue_count = GlobalTaskScheduler.size;
    RingBuffer_t *queues = GlobalTaskScheduler.queues;
    TaskDescriptor_t *next_task;

    for (int32_t i = queue_count - 1; i >= 0; --i) {
        RingBuffer_t *queue = &(queues[i]);
        if (is_ring_buf_empty(queue)) continue;

        if (!ring_buf_pop_left(queue, &next_task)) continue;


        return next_task;
    }

    return NULL;
}
