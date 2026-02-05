#include "idle_task.h"
#include "syscall.h"
#include "timer.h"
#include "uart.h"
#include <stdint.h>

// These variables are updated by the KERNEL (in task_manager.c)
// when switching to/from idle task
volatile uint64_t idle_total_time = 0;
volatile uint64_t idle_enter_time = 0;

static uint64_t last_report_time = 0;
static uint64_t last_idle_total = 0;

#define REPORT_INTERVAL_US 1000000

// Called by kernel when switching TO idle task
void idle_enter(void) {
    idle_enter_time = read_timer();
}

// Called by kernel when switching AWAY FROM idle task
void idle_exit(void) {
    uint64_t now = read_timer();
    idle_total_time += (now - idle_enter_time);
}

void idle_task(void) {
    last_report_time = read_timer();
    last_idle_total = 0;

    for (;;) {
        // Put CPU into low-power state until next interrupt
        __asm__ volatile("wfi");

        // check if it's time to report
        uint64_t now = read_timer();
        uint64_t elapsed = now - last_report_time;

        if (elapsed >= REPORT_INTERVAL_US) {
            uint64_t idle_in_period = idle_total_time - last_idle_total;
            int idle_percent = (int)((idle_in_period * 100) / elapsed);

            uart_printf(CONSOLE, "Idle: %d%%\r\n", idle_percent);

            last_report_time = now;
            last_idle_total = idle_total_time;
        }
    }
}

// Get current idle percentage 
int get_idle_percentage(void) {
    uint64_t now = read_timer();
    uint64_t elapsed = now - last_report_time;

    if (elapsed == 0) return 0;

    uint64_t idle_in_period = idle_total_time - last_idle_total;
    return (int)((idle_in_period * 100) / elapsed);
}