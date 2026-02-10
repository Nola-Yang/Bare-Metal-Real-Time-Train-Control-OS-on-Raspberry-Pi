#include "client_tasks.h"
#include "uart.h"
#include "clock_server.h"
#include "nameserver.h"
#include "task_manager.h"
#include "idle_task.h"
#include "syscall.h"
#include "min_heap.h"


typedef struct {
    int delay_interval;  // Ticks to delay each time
    int num_delays;      // Number of delays to perform
} ClientParams_t;


#define CLIENT_COUNT 4

static int Clients[CLIENT_COUNT];
static int Client_Prioirities[CLIENT_COUNT] = {3, 4, 5, 6};


// Client task - delays n times for t ticks each, printing after each delay
void client_task() {
    int my_tid = MyTid();
    int parent_tid = MyParentTid();

    ClientParams_t params;
    Send(parent_tid, NULL, 0, (char *)&params, sizeof(params));

    int clock_tid = WhoIs(CLOCK_SERVER_NAME);
    if (clock_tid < 0) {
        uart_printf(CONSOLE, "Client tid=%d: Failed to find clock server\r\n", my_tid);
        Exit();
    }

    for (int i = 0; i < params.num_delays; i++) {
        Delay(clock_tid, params.delay_interval);
        int current_tick = Time(clock_tid);
        uart_printf(CONSOLE, "Client tid=%d: interval=%d, completed=%d/%d, tick=%d\r\n",
                    my_tid, params.delay_interval, i + 1, params.num_delays, current_tick);
    }

    uart_printf(CONSOLE, "Client tid=%d: Finished all delays\r\n", my_tid);
    Exit();
}

void first_user_task() {
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

    ClientParams_t params[CLIENT_COUNT] = {
        { .delay_interval = 10, .num_delays = 20 },  // Priority 3
        { .delay_interval = 23, .num_delays = 9 },   // Priority 4
        { .delay_interval = 33, .num_delays = 6 },   // Priority 5
        { .delay_interval = 71, .num_delays = 3 },   // Priority 6
    };

    uart_printf(CONSOLE, "\r\n----- Created Clients -----\r\n");

    // Create 4 client tasks with different priorities
    ClientParams_t *param;
    for (uint8_t i = 0; i < CLIENT_COUNT; ++i) {
        Clients[i] = Create(Client_Prioirities[i], client_task);
        param = &(params[i]);
        uart_printf(CONSOLE, "tid: %d, priority: %d, delay interval: %d, no. of delays: %d\r\n", Clients[i], Client_Prioirities[i], param->delay_interval, param->num_delays);
    }

    uart_printf(CONSOLE, "\r\n");

    // Receive parameter requests from each client and reply
    for (int i = 0; i < 4; i++) {
        int sender_tid;
        Receive(&sender_tid, NULL, 0);

        int param_index;
        if (sender_tid == Clients[0]) param_index = 0;
        else if (sender_tid == Clients[1]) param_index = 1;
        else if (sender_tid == Clients[2]) param_index = 2;
        else param_index = 3;

        Reply(sender_tid, (const char *)&params[param_index], sizeof(ClientParams_t));

        uart_debug_printf(CONSOLE, "Sent params to client %d: interval=%d, num=%d\r\n",
                          sender_tid, params[param_index].delay_interval,
                          params[param_index].num_delays);
    }

    uart_printf(CONSOLE, "FirstUserTask: All clients started, waiting for completion\r\n");

    Exit();
}

