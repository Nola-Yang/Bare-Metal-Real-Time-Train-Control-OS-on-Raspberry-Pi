#ifndef _task_manager_h_
#define _task_manager_h_ 1

#include "task.h"
#include "task_scheduler.h"
#include "gic.h"


#define MAX_TASKS_COUNT 4096

typedef struct {
    int32_t current_task_id;
    TaskDescriptor_t *tasks;
    TaskDescriptor_t *free_list;
    TaskScheduler_t *task_scheduler;
    TaskDescriptor_t *event_wait_queue[EVENT_COUNT];  // Tasks waiting for each event
    uint32_t event_pending[EVENT_COUNT];              // Pending events when no waiter
} TaskManager_t;


extern TaskManager_t GlobalTaskManager;

void init_global_task_manager(TaskDescriptor_t *tasks);

void activate(TaskDescriptor_t *current_task);

// Kernel-side Create for bootstrap 
int kern_Create(int priority, void (*function)());

// Initialize interrupt handling
void init_interrupts(void);

#endif
