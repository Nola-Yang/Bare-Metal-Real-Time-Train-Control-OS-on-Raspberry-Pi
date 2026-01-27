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
#define SYSCALL_SEND         5
#define SYSCALL_RECEIVE      6
#define SYSCALL_REPLY        7

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

// Remove sender from receiver's send queue
static TaskDescriptor_t *dequeue_sender(TaskDescriptor_t *receiver) {
    TaskDescriptor_t *sender = receiver->send_queue_head;
    if (sender == NULL) return NULL;

    receiver->send_queue_head = sender->next;
    if (receiver->send_queue_head == NULL) {
        receiver->send_queue_tail = NULL;
    }
    sender->next = NULL;
    return sender;
}


static void unblock_sender_with_error(TaskDescriptor_t *sender, int err) {
    sender->tf.x[0] = (uint64_t)err;
    sender->reply_wait_tid = -1;
    sender->state = TASK_STATE_READY;
    global_task_scheduler_add_task(sender);
}

static void fail_all_senders(TaskDescriptor_t *receiver, int err) {
    TaskDescriptor_t *sender = dequeue_sender(receiver);
    while (sender != NULL) {
        unblock_sender_with_error(sender, err);
        sender = dequeue_sender(receiver);
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

    init_task_descriptor(new_task, task_id, parent_id, priority, TASK_STATE_READY);

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

    fail_all_senders(current_task, -2);
    for (int i = 0; i < GlobalTaskManager.current_task_id; i++) {
        TaskDescriptor_t *task = &GlobalTaskManager.tasks[i];
        if (task->state == TASK_STATE_REPLY_BLOCKED &&
            task->reply_wait_tid == current_task->tid) {
            unblock_sender_with_error(task, -2);
        }
    }

    current_task->state = TASK_STATE_EXITED;
    global_task_scheduler_remove_task(current_task);
    free_task_descriptor(current_task);

    TaskDescriptor_t *next_task = schedule();
    if (next_task == NULL) {
        // No runnable tasks left; stop here.
        for (;;) {
            __asm__ volatile("wfi");
            // for interrupts
            next_task = schedule();                                                                                                               
            if (next_task) break;   
        }
    }
    set_current_task(next_task);
    next_task->state = TASK_STATE_RUNNING;
}

// Message Passing 

static inline int min_int(int a, int b) {
    return (a < b) ? a : b;
}

static TaskDescriptor_t *get_task_by_tid(int tid) {
    if ((tid < 0) || (tid >= GlobalTaskManager.current_task_id)) return NULL;

    TaskDescriptor_t *task = &GlobalTaskManager.tasks[tid];
    if (task->state == TASK_STATE_FREE || task->state == TASK_STATE_EXITED) {
        return NULL;
    }
    return task;
}

// Add sender to receiver's send queue
static void enqueue_sender(TaskDescriptor_t *receiver, TaskDescriptor_t *sender) {
    sender->next = NULL;
    if (receiver->send_queue_tail == NULL) {
        receiver->send_queue_head = sender;
        receiver->send_queue_tail = sender;
    } else {
        receiver->send_queue_tail->next = sender;
        receiver->send_queue_tail = sender;
    }
}


// kern_Send: Send message to tid, block until reply
// Returns: size of reply on success, -1 if tid invalid, -2 on failure
static int kern_Send(int tid, const char *msg, int msglen, char *reply, int rplen) {
    TaskDescriptor_t *sender = get_current_task();
    TaskDescriptor_t *receiver = get_task_by_tid(tid);

    if (receiver == NULL) {
        uart_printf(CONSOLE, "kern_Send: Invalid tid %d\r\n", tid);
        return -1;  
    }

    sender->msg_buf = msg;
    sender->msg_len = msglen;
    sender->reply_buf = reply;
    sender->reply_len = rplen;

    if (receiver->state == TASK_STATE_RECEIVE_BLOCKED) {
        int copy_len = min_int(msglen, (int)receiver->tf.x[2]);
        memcpy((char *)receiver->tf.x[1], msg, copy_len);

        *((int *)receiver->tf.x[0]) = sender->tid;
        sender->state = TASK_STATE_REPLY_BLOCKED;
        sender->reply_wait_tid = receiver->tid;

        receiver->state = TASK_STATE_READY;
        receiver->tf.x[0] = (uint64_t)msglen;
        global_task_scheduler_add_task(receiver);
    } else {
        sender->state = TASK_STATE_SEND_BLOCKED;
        enqueue_sender(receiver, sender);
    }

    // Schedule next task (sender is blocked)
    TaskDescriptor_t *next_task = schedule();
    if (next_task == NULL) {
        for (;;) {
            __asm__ volatile("wfi");
            next_task = schedule();
            if (next_task) break;
        }
    }
    set_current_task(next_task);
    next_task->state = TASK_STATE_RUNNING;

    return 0;
}

// kern_Receive: Block until a message arrives
// Returns: size of message on success
static int kern_Receive(int *tid, char *msg, int msglen) {
    TaskDescriptor_t *receiver = get_current_task();

    TaskDescriptor_t *sender = dequeue_sender(receiver);

    if (sender != NULL) {
        int copy_len = min_int(sender->msg_len, msglen);
        memcpy(msg, sender->msg_buf, copy_len);
        *tid = sender->tid;

        sender->state = TASK_STATE_REPLY_BLOCKED;
        sender->reply_wait_tid = receiver->tid;
        return sender->msg_len;
    }

    receiver->state = TASK_STATE_RECEIVE_BLOCKED;
    
    TaskDescriptor_t *next_task = schedule();
    if (next_task == NULL) {
        for (;;) {
            __asm__ volatile("wfi");
            next_task = schedule();
            if (next_task) break;
        }
    }
    set_current_task(next_task);
    next_task->state = TASK_STATE_RUNNING;

    return 0;
}

// kern_Reply: Send reply to a task that sent a message
// Returns: size of reply on success, -1 if tid invalid, -2 if not reply-blocked
static int kern_Reply(int tid, const char *reply, int rplen) {
    TaskDescriptor_t *sender = get_task_by_tid(tid);

    if (sender == NULL) {
        uart_printf(CONSOLE, "kern_Reply: Invalid tid %d\r\n", tid);
        return -1;  
    }

    if (sender->state != TASK_STATE_REPLY_BLOCKED) {
        return -2;  
    }

    int copy_len = min_int(rplen, sender->reply_len);
    memcpy(sender->reply_buf, reply, copy_len);

    sender->tf.x[0] = (uint64_t)copy_len;
    sender->reply_wait_tid = -1;

    sender->state = TASK_STATE_READY;
    global_task_scheduler_add_task(sender);

    return copy_len;
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
            return;  // kern_Yield handles scheduling
        case SYSCALL_EXIT:
            kern_Exit();
            return;  // kern_Exit handles scheduling
        case SYSCALL_SEND:
            ret = kern_Send((int)tf->x[0], (const char *)tf->x[1], (int)tf->x[2],
                            (char *)tf->x[3], (int)tf->x[4]);
            if (ret < 0) {
                break;  
            }
            return;  
        case SYSCALL_RECEIVE:
            ret = kern_Receive((int *)tf->x[0], (char *)tf->x[1], (int)tf->x[2]);
            if (get_current_task()->state == TASK_STATE_RECEIVE_BLOCKED) {
                return;  // Blocked, scheduling done by kern_Receive
            }
            break;  // Not blocked, continue to reschedule
        case SYSCALL_REPLY:
            ret = kern_Reply((int)tf->x[0], (const char *)tf->x[1], (int)tf->x[2]);
            break;
        default:
            ret = -1;
            break;
    }

    tf->x[0] = (uint64_t)ret;

    current_task->state = TASK_STATE_READY;
    global_task_scheduler_add_task(current_task);
    TaskDescriptor_t *next_task = schedule();
    set_current_task(next_task);
    next_task->state = TASK_STATE_RUNNING;
}
