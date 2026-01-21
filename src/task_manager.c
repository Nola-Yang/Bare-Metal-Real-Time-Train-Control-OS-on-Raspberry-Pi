#include "task_manager.h"
#include "trapframe.h"
#include "util.h"


// Syscall numbers
#define SYSCALL_CREATE       0
#define SYSCALL_MYTID        1
#define SYSCALL_MYPARENTTID  2
#define SYSCALL_YIELD        3
#define SYSCALL_EXIT         4


TaskManager_t GlobalTaskManager; 


void init_global_task_manager(TaskDescriptor_t *tasks) {
    GlobalTaskManager.task_scheduler = &GlobalTaskScheduler;
    GlobalTaskManager.tasks = tasks;
    GlobalTaskManager.current_task_id = 0;
}

extern void return_to_task(void);

void activate(TaskDescriptor_t *task) {
    set_current_task(task);
    task->state = TASK_STATE_RUNNING;
    return_to_task();
}

static bool is_valid_priority(int priority) {
    return (0 <= priority) && (priority < PRIORITY_LEVELS);
}


static bool no_more_task_descriptors() {
    return GlobalTaskManager.current_task_id >= MAX_TASKS_COUNT;
}


// =============================


int Create(int priority, void (*function)()) {
    if (!is_valid_priority(priority)) return -1;
    if (no_more_task_descriptors()) return -2;

    int32_t task_id = GlobalTaskManager.current_task_id;
    TaskDescriptor_t *new_task = &(GlobalTaskManager.tasks[task_id]);
    
    int32_t parent_id = -1;
    if (task_id != 0) {
        TaskDescriptor_t *current_task = get_current_task();
        parent_id = current_task->tid;
    }

    init_task_descriptor(new_task, task_id, parent_id, priority, TASK_STATE_FREE, function);
    memcpy(&(GlobalTaskManager.tasks[task_id]), new_task, sizeof(TaskDescriptor_t));

    GlobalTaskManager.current_task_id++;

    // TODO: There probably is more stuff to add into the trap frame

    return 0;
}

int MyTid() {
    TaskDescriptor_t *current_task = get_current_task();
    return current_task->tid;
}

int MyParentTid() {
    TaskDescriptor_t *current_task = get_current_task();

    int parent_id = current_task->parent_tid;
    if (parent_id < 0) return parent_id;

    TaskDescriptor_t *parent_task = &(GlobalTaskManager.tasks[parent_id]);

    if (parent_task->state == TASK_STATE_EXITED) return -2;
    return parent_id;
}

void Yield() {
    // TODO: Implement context switch for yield
}

void Exit() {
    TaskDescriptor_t *current_task = get_current_task();

    // TODO: Any cleanup for trapframe needed?

    current_task->state = TASK_STATE_EXITED;
    global_task_scheduler_remove_task(current_task);
}


// =============================


void syscall_dispatch() {
    TaskDescriptor_t *current_task = get_current_task();
    trapframe_t *tf = &current_task->tf;

    uint64_t syscall_num = tf->x[8];
    int64_t ret = 0;

    switch (syscall_num) {
        case SYSCALL_CREATE:
            ret = Create((int)tf->x[0], (void (*)())tf->x[1]);
            break;
        case SYSCALL_MYTID:
            ret = MyTid();
            break;
        case SYSCALL_MYPARENTTID:
            ret = MyParentTid();
            break;
        case SYSCALL_YIELD:
            Yield();
            break;
        case SYSCALL_EXIT:
            Exit();
            break;
        default:
            ret = -1;
            break;
    }

    tf->x[0] = (uint64_t)ret;
}