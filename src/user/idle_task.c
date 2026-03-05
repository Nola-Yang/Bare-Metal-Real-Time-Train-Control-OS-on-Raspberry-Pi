#include "idle_task.h"
#include "syscall.h"
#include "timer.h"
#include "task_scheduler.h"
#include <stdint.h>

// These variables are updated by the KERNEL (in task_manager.c)
// when switching to/from idle task
volatile uint64_t Idle_Total_Time = 0;
volatile uint64_t Idle_Enter_Time = 0;

static uint64_t Last_Report_Time = 0;
static uint64_t Last_Idle_Total = 0;

// Rolling window ring buffer
static int idle_ring[IDLE_WINDOW_TICKS];
static int idle_ring_head  = 0;
static int idle_ring_count = 0;
static int idle_ring_sum   = 0;


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
    for (;;) {
        // Put CPU into low-power state until next interrupt
        __asm__ volatile("wfi");
    }
}

// Returns rolling-average idle percentage over the last IDLE_WINDOW_TICKS ticks.
// Call once per 100ms tick.
int get_idle_percentage(void) {
    uint64_t now = read_timer();

    if (Last_Report_Time == 0) {
        Last_Report_Time = now;
        Last_Idle_Total  = Idle_Total_Time;
        return 0;
    }

    uint64_t elapsed = now - Last_Report_Time;
    if (elapsed == 0) {
        return (idle_ring_count > 0) ? (idle_ring_sum / idle_ring_count) : 0;
    }

    uint64_t idle_in_period = Idle_Total_Time - Last_Idle_Total;
    int raw = (int)((idle_in_period * 100) / elapsed);
    if (raw > 100) raw = 100;

    Last_Report_Time = now;
    Last_Idle_Total  = Idle_Total_Time;

    if (idle_ring_count == IDLE_WINDOW_TICKS) {
        idle_ring_sum -= idle_ring[idle_ring_head];
    } else {
        idle_ring_count++;
    }
    idle_ring[idle_ring_head] = raw;
    idle_ring_sum += raw;
    idle_ring_head = (idle_ring_head + 1) % IDLE_WINDOW_TICKS;

    return idle_ring_sum / idle_ring_count;
}
