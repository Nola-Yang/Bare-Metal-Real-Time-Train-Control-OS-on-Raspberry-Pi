#include "rpi.h"
#include "task.h"
#include "task_manager.h"
#include "task_scheduler.h"
#include "syscall.h"
#include "uart.h"
#include "util.h"
#include "nameserver.h"
#include "rps_server.h"
#include "rps_client.h"
#include "rps_test.h"
#include "performance_test.h"
#include <stdint.h>


static const uint32_t FIRST_USER_TASK_PRIORITY = 1;

extern void setup_mmu(); // in mmu.S

static TaskDescriptor_t tasks[MAX_TASKS_COUNT];
static TaskDescriptor_t *raw_buffers[PRIORITY_LEVELS][RING_BUFFER_SIZE];
static RingBuffer_t queues[PRIORITY_LEVELS];


void first_user_task() {
    int32_t tid;

    uart_printf(CONSOLE, "========================================\r\n");

    #ifndef MEASURE
    uart_printf(CONSOLE, "K2 Test Suite\r\n");
    #else
    uart_printf(CONSOLE, "K2 Performance Test\r\n");
    #endif

    uart_printf(CONSOLE, "========================================\r\n");

    tid = Create(NAMESERVER_PRIORITY, nameserver_task);

    if (tid != NAMESERVER_TID) {
        uart_printf(CONSOLE, "ERROR: NameServer tid=%d (expected %d)\r\n", tid, NAMESERVER_TID);
        Exit();
    }

    #ifndef MEASURE
    uart_printf(CONSOLE, "Created NameServer, tid=%d\r\n", tid);
    rps_test_run();
    #else
    perform_test_run();
    #endif

    Exit();
}


int kmain() {
#if defined(MMU)
	setup_mmu();
#endif

    #ifdef MEASURE
	bool dcache_on = false;
	bool icache_on = false;

	#ifdef DCACHE
	dcache_on = true;
	#endif

	#ifdef ICACHE
	icache_on = true;
	#endif

    toggle_caches(dcache_on, icache_on);
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
