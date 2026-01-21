#include "task_manager.h"
#include "trapframe.h"
#include "util.h"
#include "uart.h"
#include <string.h>


// Syscall numbers
#define SYSCALL_CREATE       0
#define SYSCALL_MYTID        1
#define SYSCALL_MYPARENTTID  2
#define SYSCALL_YIELD        3
#define SYSCALL_EXIT         4

#define SPSR_FOR_EL0t 0x0


TaskManager_t GlobalTaskManager; 


void init_global_task_manager(TaskDescriptor_t *tasks) {
    GlobalTaskManager.task_scheduler = &GlobalTaskScheduler;
    GlobalTaskManager.tasks = tasks;
    GlobalTaskManager.current_task_id = 0;
    GlobalTaskManager.free_list = NULL;
}

extern void return_to_task(void);

void activate(TaskDescriptor_t *task) {
    global_task_scheduler_remove_task(task);
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

static TaskDescriptor_t *alloc_task_descriptor(int32_t *task_id) {
    if (!no_more_task_descriptors()) {
        *task_id = GlobalTaskManager.current_task_id;
        TaskDescriptor_t *td = &(GlobalTaskManager.tasks[GlobalTaskManager.current_task_id]);
        GlobalTaskManager.current_task_id++;
        td->next = NULL;
        return td;
    }

    if (GlobalTaskManager.free_list == NULL) return NULL;

    TaskDescriptor_t *td = GlobalTaskManager.free_list;
    GlobalTaskManager.free_list = td->next;
    td->next = NULL;
    *task_id = td->tid;
    return td;
}

static void free_task_descriptor(TaskDescriptor_t *task) {
    task->state = TASK_STATE_FREE;
    task->next = GlobalTaskManager.free_list;
    GlobalTaskManager.free_list = task;
}


// =============================

static void check_stack_canary(TaskDescriptor_t *task) {
    if (task->stack_canary != STACK_CANARY_VALUE) {
        uart_printf(CONSOLE, "PANIC: User stack overflow detected! tid=%d canary=0x%lx\r\n",
                    task->tid, task->stack_canary);
        for (;;) __asm__ volatile("wfi");
    }
}

static void check_user_stack_bounds(TaskDescriptor_t *task) {
    uint64_t sp = task->tf.sp_el0;
    uint64_t stack_bottom = (uint64_t)task->stack;
    uint64_t stack_top = stack_bottom + TASK_STACK_SIZE;

    if (sp < stack_bottom || sp > stack_top) {
        uart_printf(CONSOLE, "PANIC: User stack out of bounds! tid=%d sp=0x%lx bounds=[0x%lx, 0x%lx]\r\n",
                    task->tid, sp, stack_bottom, stack_top);
        for (;;) __asm__ volatile("wfi");
    }
}

int kern_Create(int priority, void (*function)()) {
    if (!is_valid_priority(priority)) return -1;

    int32_t task_id = -1;
    TaskDescriptor_t *new_task = alloc_task_descriptor(&task_id);
    if (new_task == NULL) return -2;
    
    int32_t parent_id = -1;
    TaskDescriptor_t *current_task = get_current_task();
    if (current_task != NULL) {
        parent_id = current_task->tid;
    }

    init_task_descriptor(new_task, task_id, parent_id, priority, TASK_STATE_READY, function);

    memset(&new_task->tf, 0, sizeof(trapframe_t));

    /* Initialize stack canary for overflow detection */
    new_task->stack_canary = STACK_CANARY_VALUE;

    new_task->tf.elr_el1 = (uint64_t)function;
    new_task->tf.sp_el0 = (uint64_t)(new_task->stack + TASK_STACK_SIZE);
    new_task->tf.spsr_el1 = SPSR_FOR_EL0t;

    global_task_scheduler_add_task(new_task);

    return task_id;
}

static int kern_MyTid(void) {
    TaskDescriptor_t *current_task = get_current_task();
    return current_task->tid;
}

static int kern_MyParentTid(void) {
    TaskDescriptor_t *current_task = get_current_task();

    int parent_id = current_task->parent_tid;
    if (parent_id < 0) return parent_id;

    TaskDescriptor_t *parent_task = &(GlobalTaskManager.tasks[parent_id]);

    if (parent_task->state == TASK_STATE_EXITED || parent_task->state == TASK_STATE_FREE) return -2;
    return parent_id;
}

static void kern_Yield(void) {
    // Put current running task back into ready queue, then pick next to run.
    TaskDescriptor_t *current_task = get_current_task();
    current_task->state = TASK_STATE_READY;
    global_task_scheduler_add_task(current_task);
    TaskDescriptor_t *next_task = schedule();
    set_current_task(next_task);
    next_task->state = TASK_STATE_RUNNING;
}

static void kern_Exit(void) {
    TaskDescriptor_t *current_task = get_current_task();

    current_task->state = TASK_STATE_EXITED;
    global_task_scheduler_remove_task(current_task);
    free_task_descriptor(current_task);

    TaskDescriptor_t *next_task = schedule();
    if (next_task == current_task) {
        // No runnable tasks left; stop here.
        for (;;) {
            __asm__ volatile("wfi");
        }
    }
    set_current_task(next_task);
    next_task->state = TASK_STATE_RUNNING;
}


// =============================


void syscall_dispatch() {
    TaskDescriptor_t *current_task = get_current_task();
    trapframe_t *tf = &current_task->tf;

    /* Check for user stack overflow on every syscall entry */
    check_stack_canary(current_task);
    check_user_stack_bounds(current_task);

    uint64_t syscall_num = tf->x[8];
    int64_t ret = 0;

    switch (syscall_num) {
        case SYSCALL_CREATE:
            ret = kern_Create((int)tf->x[0], (void (*)())tf->x[1]);
            break;
        case SYSCALL_MYTID:
            ret = kern_MyTid();
            break;
        case SYSCALL_MYPARENTTID:
            ret = kern_MyParentTid();
            break;
        case SYSCALL_YIELD:
            kern_Yield();
            break;
        case SYSCALL_EXIT:
            kern_Exit();
            break;
        default:
            ret = -1;
            break;
    }

    tf->x[0] = (uint64_t)ret;

    if (syscall_num == SYSCALL_YIELD || syscall_num == SYSCALL_EXIT) {
        return;
    }

    current_task->state = TASK_STATE_READY;
    global_task_scheduler_add_task(current_task);
    TaskDescriptor_t *next_task = schedule();
    set_current_task(next_task);
    next_task->state = TASK_STATE_RUNNING;
}
