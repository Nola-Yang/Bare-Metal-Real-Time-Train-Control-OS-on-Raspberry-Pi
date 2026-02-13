#include "client_tasks.h"
#include "clock_server.h"
#include "nameserver.h"
#include "task_manager.h"
#include "idle_task.h"
#include "syscall.h"
#include "min_heap.h"
#include "terminal_server.h"
#include "can_server.h"
#include "train_control.h"
#include "spi.h"
#include "mcp2515.h"
#include "rpi.h"
#include "uart.h"
#include "kassert.h"


typedef struct {
    int delay_interval;  // Ticks to delay each time
    int num_delays;      // Number of delays to perform
} ClientParams_t;


// Train Control System
void first_user_task() {
    int ns_tid = Create(NAMESERVER_PRIORITY, nameserver_task);
    KASSERT(ns_tid == NAMESERVER_TID);

    int term_tid = Create(TERMINAL_SERVER_PRIORITY, terminal_server_task);
    KASSERT(term_tid >= 0);

    int clock_tid = Create(CLOCK_SERVER_PRIORITY, clock_server_task);
    KASSERT(clock_tid >= 0);

    int idle_tid = Create(IDLE_TASK_PRIORITY, idle_task);
    KASSERT(idle_tid >= 0);

    spi_init();
    mcp2515_init();
    mcp2515_clear_interrupts();

    int can_tid = Create(CAN_SERVER_PRIORITY, can_server_task);
    KASSERT(can_tid >= 0);

    int train_tid = Create(TRAIN_CONTROL_PRIORITY, train_control_task);
    KASSERT(train_tid >= 0);

    Exit();
}
