#include "task_list.h"
#include <stddef.h>


void init_task_list(TaskList_t *task_list) {
    task_list->head = NULL;
    task_list->tail = NULL;
    task_list->count = 0;
}

bool task_list_is_empty(TaskList_t *task_list) {
    return (task_list->count <= 0 || task_list->head == NULL);
}

void task_list_append(TaskList_t *task_list, TaskDescriptor_t *task) {
    task->next = NULL;
    if (task_list->tail) {
        task_list->tail->next = task;
        task_list->tail = task;
    } else {
        task_list->head = task;
        task_list->tail = task;
    }

    task_list->count++;
}

bool task_list_pop_left(TaskList_t *task_list, TaskDescriptor_t **result) {
    if (task_list_is_empty(task_list)) return false;

    *result = task_list->head;
    task_list->head = (*result)->next;
    if (task_list->head == NULL) {
        task_list->tail = NULL;
    }
    (*result)->next = NULL;
    task_list->count--;
    return true;
}
