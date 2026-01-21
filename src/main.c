#include "rpi.h"
#include "task.h"
#include "task_manager.h"
#include "task_scheduler.h"
#include "syscall.h"
#include "uart.h"
#include "util.h"
#include <stdint.h>


static const uint32_t FIRST_USER_TASK_PRIORITY = 1;

extern void setup_mmu(); // in mmu.S

// Static allocation to avoid stack overflow (these are too large for 4KB kernel stack)
static TaskDescriptor_t tasks[MAX_TASKS_COUNT];
static TaskDescriptor_t *raw_buffers[PRIORITY_LEVELS][RING_BUFFER_SIZE];
static RingBuffer_t queues[PRIORITY_LEVELS];


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


	for (uint32_t i = 0; i < PRIORITY_LEVELS; ++i) {
		init_custom_ring_buffer(&queues[i], raw_buffers[i], RING_BUFFER_SIZE, sizeof(TaskDescriptor_t *));
	}

	init_global_task_scheduler(queues, PRIORITY_LEVELS);
	init_global_task_manager(tasks);

	kern_Create(FIRST_USER_TASK_PRIORITY, first_user_task);

	// After this, control flow is driven by syscalls, not this loop. not sure for now
	activate(&tasks[0]);

    return 0;
}