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


static const uint32_t FIRST_USER_TASK_PRIORITY = 1;

extern void setup_mmu(); // in mmu.S

static TaskDescriptor_t tasks[MAX_TASKS_COUNT];
static TaskDescriptor_t *raw_buffers[PRIORITY_LEVELS][RING_BUFFER_SIZE];
static RingBuffer_t queues[PRIORITY_LEVELS];


typedef struct {
    int delay_interval;  // Ticks to delay each time
    int num_delays;      // Number of delays to perform
} ClientParams_t;

// Client task - delays n times for t ticks each, printing after each delay
static void k3_client_task(void) {
    int my_tid = MyTid();
    int parent_tid = MyParentTid();

    ClientParams_t params;
    Send(parent_tid, NULL, 0, (char *)&params, sizeof(params));

    int clock_tid = WhoIs(CLOCK_SERVER_NAME);
    if (clock_tid < 0) {
        uart_printf(CONSOLE, "Client %d: Failed to find clock server\r\n", my_tid);
        Exit();
    }

    for (int i = 0; i < params.num_delays; i++) {
        Delay(clock_tid, params.delay_interval);
        int current_tick = Time(clock_tid);
        uart_printf(CONSOLE, "Client tid=%d, interval=%d, completed=%d/%d, tick=%d\r\n",
                    my_tid, params.delay_interval, i + 1, params.num_delays, current_tick);
    }

    uart_printf(CONSOLE, "Client %d: Finished all delays\r\n", my_tid);
    Exit();
}


void first_user_task(void) {
    uart_printf(CONSOLE, "========================================\r\n");
    uart_printf(CONSOLE, "K3 Clock Server Test\r\n");
    uart_printf(CONSOLE, "========================================\r\n");

    int ns_tid = Create(NAMESERVER_PRIORITY, nameserver_task);
    if (ns_tid != NAMESERVER_TID) {
        uart_printf(CONSOLE, "ERROR: NameServer tid=%d (expected %d)\r\n", ns_tid, NAMESERVER_TID);
        Exit();
    }
    uart_printf(CONSOLE, "Created NameServer, tid=%d\r\n", ns_tid);

    int clock_tid = Create(CLOCK_SERVER_PRIORITY, clock_server_task); // Create Clock Server
    uart_printf(CONSOLE, "Created ClockServer, tid=%d\r\n", clock_tid);
    int idle_tid = Create(IDLE_TASK_PRIORITY, idle_task);     // Create Idle Task
    uart_printf(CONSOLE, "Created IdleTask, tid=%d\r\n", idle_tid);

    // Create 4 client tasks with different priorities
    int client1 = Create(K3_CLIENT_PRIORITY_3, k3_client_task);
    int client2 = Create(K3_CLIENT_PRIORITY_4, k3_client_task);
    int client3 = Create(K3_CLIENT_PRIORITY_5, k3_client_task);
    int client4 = Create(K3_CLIENT_PRIORITY_6, k3_client_task);

    uart_printf(CONSOLE, "Created clients: %d, %d, %d, %d\r\n",
                client1, client2, client3, client4);

    ClientParams_t params[4] = {
        { .delay_interval = 10, .num_delays = 20 },  // Priority 3
        { .delay_interval = 23, .num_delays = 9 },   // Priority 4
        { .delay_interval = 33, .num_delays = 6 },   // Priority 5
        { .delay_interval = 71, .num_delays = 3 },   // Priority 6
    };

    // Receive parameter requests from each client and reply
    for (int i = 0; i < 4; i++) {
        int sender_tid;
        Receive(&sender_tid, NULL, 0);

        int param_index;
        if (sender_tid == client1) param_index = 0;
        else if (sender_tid == client2) param_index = 1;
        else if (sender_tid == client3) param_index = 2;
        else param_index = 3;

        Reply(sender_tid, (const char *)&params[param_index], sizeof(ClientParams_t));
        uart_printf(CONSOLE, "Sent params to client %d: interval=%d, num=%d\r\n",
                    sender_tid, params[param_index].delay_interval,
                    params[param_index].num_delays);
    }

    uart_printf(CONSOLE, "FirstUserTask: All clients started, waiting for completion\r\n");

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

	init_interrupts();

	kern_Create(FIRST_USER_TASK_PRIORITY, first_user_task);

	// transfer control flow to the syscalls
	activate(&tasks[0]);

    return 0;
}
