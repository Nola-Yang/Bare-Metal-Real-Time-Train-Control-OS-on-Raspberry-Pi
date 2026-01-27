#include "task.h"


void init_task_descriptor(TaskDescriptor_t *task_descriptor, int tid, int parent_tid, int priority, int state) {
    task_descriptor->tid = tid;
    task_descriptor->parent_tid = parent_tid;
    task_descriptor->priority = priority;
    task_descriptor->state = state;

    // Initialize message passing fields
    task_descriptor->send_queue_head = 0;
    task_descriptor->send_queue_tail = 0;
    task_descriptor->msg_buf = 0;
    task_descriptor->msg_len = 0;
    task_descriptor->reply_buf = 0;
    task_descriptor->reply_len = 0;
    task_descriptor->reply_wait_tid = -1;
}