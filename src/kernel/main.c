#include "rpi.h"
#include "task.h"
#include "task_manager.h"
#include "task_scheduler.h"
#include "syscall.h"
#include "uart.h"
#include "nameserver.h"
#include "timer.h"
#include "gic.h"
#include "clock_server.h"
#include "idle_task.h"
#include <stdint.h>
#include <stddef.h>
#include "util.h"
#include "client_tasks.h"


extern void setup_mmu(); // in mmu.S

static TaskDescriptor_t tasks[MAX_TASKS_COUNT];
static TaskList_t task_lists[PRIORITY_LEVELS];


int kmain() {
	/* Init kernel stack canary */
	extern uint64_t __kernel_stack_canary;
	*((volatile uint64_t *)&__kernel_stack_canary) = STACK_CANARY_VALUE;

#if defined(MMU)
	setup_mmu();
#endif

	bool dcache_on = false;
	bool icache_on = false;

	#ifdef DCACHE
	dcache_on = true;
	#endif

	#ifdef ICACHE
	icache_on = true;
	#endif

    toggle_caches(dcache_on, icache_on);

	// set up GPIO pins for both console uart and canbus
	gpio_init();
	gpio_init_interrupt();

	// not strictly necessary, since console is configured during boot
	uart_config_and_enable(CONSOLE);


	for (uint32_t i = 0; i < PRIORITY_LEVELS; ++i) {
		init_task_list(&task_lists[i]);
	}

	init_global_task_scheduler(task_lists, PRIORITY_LEVELS);
	init_global_task_manager(tasks);

	init_interrupts();

	kern_Create(FIRST_USER_TASK_PRIORITY, first_user_task);

	// transfer control flow to the syscalls
	activate(&tasks[0]);

    return 0;
}
