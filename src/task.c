#include "task.h"


void init_task_descriptor(TaskDescriptor_t *task_descriptor, int tid, int parent_tid, int priority, int state, void (*function)()) {
    task_descriptor->tid = tid;
    task_descriptor->parent_tid = parent_tid;
    task_descriptor->priority = priority;
    task_descriptor->state = state;
    task_descriptor->function = function;
}