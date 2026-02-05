#ifndef TASK_LIST_H
#define TASK_LIST_H

#include <stdint.h>
#include <stdbool.h>
#include "task.h"

// TaskList: A linked list for tasks
typedef struct {
    uint32_t count;
    TaskDescriptor_t *head;
} TaskList_t;


// init_task_list: Creates a new task linked list
void init_task_list(TaskList_t *task_list);

// task_list_is_empty: Checks if the task linked list is empty
bool task_list_is_empty(TaskList_t *task_list);

// task_list_append: Adds a new task to the end of the linked list
void task_list_append(TaskList_t *task_list, TaskDescriptor_t *task);

// task_list_pop_left: Removes a task from the head of the linked list
bool task_list_pop_left(TaskList_t *task_list, TaskDescriptor_t **result);

#endif