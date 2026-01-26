#include "rpi.h"
#include "task.h"
#include "task_manager.h"
#include "task_scheduler.h"
#include "syscall.h"
#include "uart.h"
#include "util.h"
#include "nameserver.h"
#include "rps.h"
#include <stdint.h>


static const uint32_t FIRST_USER_TASK_PRIORITY = 1;

extern void setup_mmu(); // in mmu.S

static TaskDescriptor_t tasks[MAX_TASKS_COUNT];
static TaskDescriptor_t *raw_buffers[PRIORITY_LEVELS][RING_BUFFER_SIZE];
static RingBuffer_t queues[PRIORITY_LEVELS];


// Priority levels for k2 tasks
#define NAMESERVER_PRIORITY     0  // Highest priority
#define RPS_SERVER_PRIORITY     2
#define RPS_CLIENT_PRIORITY     3
#define NUM_RPS_CLIENTS         4

void first_user_task() {
	int32_t tid;

	uart_printf(CONSOLE, "========================================\r\n");
	uart_printf(CONSOLE, "FirstUserTask: Starting K2 Test\r\n");
	uart_printf(CONSOLE, "========================================\r\n");

	tid = Create(NAMESERVER_PRIORITY, nameserver_task); //tid == 1
	
	if (tid != NAMESERVER_TID) {
		uart_printf(CONSOLE, "FirstUserTask: NameServer created with wrong tid %d (expected %d)\r\n",
		            tid, NAMESERVER_TID);
		Exit();
	}


	if (tid >= 0) {
		uart_printf(CONSOLE, "FirstUserTask: Created NameServer, tid=%d\r\n", tid);
	} else {
		uart_printf(CONSOLE, "FirstUserTask: Failed to create NameServer: %d\r\n", tid);
		Exit();
	}

	tid = Create(RPS_SERVER_PRIORITY, rps_server_task);
	if (tid >= 0) {
		uart_printf(CONSOLE, "FirstUserTask: Created RPS Server, tid=%d\r\n", tid);
	} else {
		uart_printf(CONSOLE, "FirstUserTask: Failed to create RPS Server: %d\r\n", tid);
		Exit();
	}

	// 3. Create RPS clients (must be even number for pairing)
	for (int i = 0; i < NUM_RPS_CLIENTS; i++) {
		tid = Create(RPS_CLIENT_PRIORITY, rps_client_task);
		if (tid >= 0) {
			uart_printf(CONSOLE, "FirstUserTask: Created RPS Client %d, tid=%d\r\n", i, tid);
		} else {
			uart_printf(CONSOLE, "FirstUserTask: Failed to create RPS Client %d: %d\r\n", i, tid);
		}
	}

	uart_printf(CONSOLE, "FirstUserTask: All tasks created, exiting\r\n");
	uart_printf(CONSOLE, "========================================\r\n");
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

	// transfer control flow to the syscalls
	activate(&tasks[0]);

    return 0;
}
