#ifndef _task_manager_h_
#define _task_manager_h_ 1

#include "task.h"
#include "task_scheduler.h"


#define MAX_TASKS_COUNT 4096

typedef struct {
    int32_t current_task_id;
    TaskDescriptor_t *tasks;
    TaskScheduler_t *task_scheduler;
} TaskManager_t;


extern TaskManager_t GlobalTaskManager;

void init_global_task_manager(TaskDescriptor_t *tasks);

void activate(TaskDescriptor_t *current_task);

// =========================

int Create(int priority, void (*function)());

int MyTid();

int MyParentTid();

void Yield();

void Exit();

#endif