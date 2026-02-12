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


typedef struct {
    int delay_interval;  // Ticks to delay each time
    int num_delays;      // Number of delays to perform
} ClientParams_t;



static void debug_log_created(const char *name, int tid) {
    uart_debug_printf(CONSOLE, "Created %s, tid=%d\r\n", name, tid);
}

// Train Control System
void first_user_task() {
    // assert Name Server TID=0
    int ns_tid = Create(NAMESERVER_PRIORITY, nameserver_task);
    if (ns_tid != NAMESERVER_TID) {
        Exit();
    }

    int term_tid = Create(TERMINAL_SERVER_PRIORITY, terminal_server_task);

    debug_log_created("NameServer", ns_tid);
    debug_log_created("TerminalServer", term_tid);

    //Clock Server
    int clock_tid = Create(CLOCK_SERVER_PRIORITY, clock_server_task);
    debug_log_created("ClockServer", clock_tid);

    //Idle Task
    int idle_tid = Create(IDLE_TASK_PRIORITY, idle_task);
    debug_log_created("IdleTask", idle_tid);

    spi_init();
    mcp2515_init();
    mcp2515_clear_interrupts();       
    uart_debug_printf(CONSOLE, "Initialized SPI and MCP2515\r\n");

    // CAN Server
    int can_tid = Create(CAN_SERVER_PRIORITY, can_server_task);
    debug_log_created("CANServer", can_tid);

    // Train Control Task
    int train_tid = Create(TRAIN_CONTROL_PRIORITY, train_control_task);
    debug_log_created("TrainControl", train_tid);

    uart_debug_printf(CONSOLE, "FirstUserTask: All tasks created, exiting\r\n");
    Exit();
}
