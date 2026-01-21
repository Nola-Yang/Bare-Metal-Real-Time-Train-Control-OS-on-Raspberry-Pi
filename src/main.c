#include "rpi.h"
#include "trapframe.h"
#include "task.h"
#include "task_manager.h"
#include "uart.h"
#include "util.h"
#include <stdint.h>


#define SPSR_MASK_ALL 0x3C0
#define SPSR_EL0t     0x0
#define SPSR_FOR_EL0t (SPSR_MASK_ALL | SPSR_EL0t)

extern void activate_first_task(void);

static TaskDescriptor_t first_task;
static uint8_t user_stack[4096] __attribute__((aligned(16)));
static const uint32_t FIRST_USER_TASK_PRIORITY = 1;


extern void syscall_dispatch(void);
extern void setup_mmu(); // in mmu.S


//Todo
static void user_entry(void) {
    for (;;) {
        __asm__ volatile("svc #0");
    }
}

static TaskDescriptor_t *create_first_task(void) {
    TaskDescriptor_t *t = &first_task;

    memset(t, 0, sizeof(*t));
    t->tid = 1;
    t->parent_tid = 0;
    t->priority = 0;
    t->state = TASK_STATE_READY;

    t->tf.elr_el1 = (uint64_t)user_entry;
    t->tf.sp_el0 = (uint64_t)(user_stack + sizeof(user_stack));
    t->tf.spsr_el1 = SPSR_FOR_EL0t;

    return t;
}


void child_task() {
	uint32_t child_id = MyTid();
	uint32_t parent_id = MyParentTid();
	uart_printf(CONSOLE, "Child tid: %u, Parent tid: %u\r\n", child_id, parent_id);

	Yield();

	child_id = MyTid();
	parent_id = MyParentTid();
	uart_printf(CONSOLE, "Child tid: %u, Parent tid: %u\r\n", child_id, parent_id);

	Exit();
}

void first_user_task() {
	uint32_t priority;
	int32_t child_id;

	for (uint32_t i = 0; i < 4; ++i) {
		priority = (i < 2) ? FIRST_USER_TASK_PRIORITY - 1 : FIRST_USER_TASK_PRIORITY + 1;
		child_id = Create(priority, child_task);

		if (child_id >= 0) {
			uart_printf(CONSOLE, "Created: %u\r\n", child_id);
		} else {
			uart_printf(CONSOLE, "Error creating child task: %u\r\n", child_id);
		}
	}

	uart_printf(CONSOLE, "FirstUserTask: exiting\r\n");
	Exit();
}


int kmain() {
#if defined(MMU)
	setup_mmu();
#endif

	// set up GPIO pins for both console uart and canbus
	gpio_init();

	// not strictly necessary, since console is configured during boot
	uart_config_and_enable(CONSOLE);

	// setup multilevel queues
	TaskDescriptor_t *raw_buffers [PRIORITY_LEVELS][RING_BUFFER_SIZE];
	RingBuffer_t queues[PRIORITY_LEVELS];
	
	for (uint32_t i = 0; i < PRIORITY_LEVELS; ++i) {
		init_custom_ring_buffer(&queues[i], raw_buffers[i], RING_BUFFER_SIZE, sizeof(TaskDescriptor_t *));
	}

	init_global_task_scheduler(queues, PRIORITY_LEVELS);

	TaskDescriptor_t tasks[MAX_TASKS_COUNT];
	init_global_task_manager(tasks);

	// main kernel
	Create(FIRST_USER_TASK_PRIORITY, first_user_task);
	TaskDescriptor_t *current_task;

    for (;;) {
		current_task = schedule();
		activate(current_task);
	}

    return 0;
}