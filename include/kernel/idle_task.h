#ifndef _idle_task_h_
#define _idle_task_h_ 1

// Rolling window size in 100ms ticks.  50 ticks = 5 seconds.
#ifndef IDLE_WINDOW_TICKS
#define IDLE_WINDOW_TICKS 50
#endif

void idle_task(void);

// Called by kernel when switching TO idle task
void idle_enter(void);

// Called by kernel when switching AWAY FROM idle task
void idle_exit(void);

// Returns rolling-average idle percentage over the last IDLE_WINDOW_TICKS ticks
int get_idle_percentage(void);

#endif