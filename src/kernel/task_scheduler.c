#include "task_scheduler.h"
#include "task.h"
#include "util.h"
#include <stddef.h>


TaskScheduler_t GlobalTaskScheduler;


void init_global_task_scheduler(TaskList_t *task_lists, uint32_t size) {
    GlobalTaskScheduler.task_lists = task_lists;
    GlobalTaskScheduler.size = size;
}

void global_task_scheduler_add_task(TaskDescriptor_t *task) {
    task_list_append(&(GlobalTaskScheduler.task_lists[task->priority]), task);
}

// Remove task from front of its priority queue (current running task)
void global_task_scheduler_remove_task(TaskDescriptor_t *task) {
    TaskList_t *task_list = &(GlobalTaskScheduler.task_lists[task->priority]);
    if (task_list_is_empty(task_list)) return;

    uint32_t remaining = task_list->count;
    bool removed = false;

    while (remaining-- > 0) {
        TaskDescriptor_t *candidate;
        if (!task_list_pop_left(task_list, &candidate)) break;

        if (!removed && candidate == task) {
            removed = true;
            continue;
        }

        task_list_append(task_list, candidate);
    }
}

TaskDescriptor_t * schedule() {
    int32_t task_list_count = GlobalTaskScheduler.size;
    TaskList_t *task_lists = GlobalTaskScheduler.task_lists;
    TaskDescriptor_t *next_task;
    TaskList_t *task_list;

    for (int32_t i = 0; i < task_list_count; ++i) {
        task_list = &(task_lists[i]);

        if (task_list_is_empty(task_list)) continue;
        if (!task_list_pop_left(task_list, &next_task)) continue;

        return next_task;
    }

    return NULL;
}
