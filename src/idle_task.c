#include "idle_task.h"
#include "syscall.h"
#include "timer.h"
#include "uart.h"
#include "task_scheduler.h"
#include <stdint.h>

// These variables are updated by the KERNEL (in task_manager.c)
// when switching to/from idle task
volatile uint64_t Idle_Total_Time = 0;
volatile uint64_t Idle_Enter_Time = 0;

static uint64_t Last_Report_Time = 0;
static uint64_t Last_Idle_Total = 0;

#define REPORT_INTERVAL_US 1000000


// Called by kernel when switching TO idle task
void idle_enter(void) {
    Idle_Enter_Time = read_timer();
}

// Called by kernel when switching AWAY FROM idle task
void idle_exit(void) {
    uint64_t now = read_timer();
    Idle_Total_Time += (now - Idle_Enter_Time);
}

void idle_task(void) {
    Last_Report_Time = read_timer();
    Last_Idle_Total = 0;

    for (;;) {
        // Put CPU into low-power state until next interrupt
        __asm__ volatile("wfi");

        // check if it's time to report
        uint64_t now = read_timer();
        uint64_t elapsed = now - Last_Report_Time;

        if (elapsed >= REPORT_INTERVAL_US) {
            uint64_t idle_in_period = Idle_Total_Time - Last_Idle_Total;
            int idle_percent = (int)((idle_in_period * 100) / elapsed);

            uart_printf(CONSOLE, "Idle: %d%%\r\n", idle_percent);

            Last_Report_Time = now;
            Last_Idle_Total = Idle_Total_Time;
        }
    }
}

// Get current idle percentage 
int get_idle_percentage(void) {
    uint64_t now = read_timer();
    uint64_t elapsed = now - Last_Report_Time;

    if (elapsed == 0) return 0;

    uint64_t idle_in_period = Idle_Total_Time - Last_Idle_Total;
    return (int)((idle_in_period * 100) / elapsed);
}