#ifndef _idle_task_h_
#define _idle_task_h_ 1


void idle_task(void);

// Called by kernel when switching TO idle task
void idle_enter(void);

// Called by kernel when switching AWAY FROM idle task
void idle_exit(void);

// Get idle percentage
int get_idle_percentage(void);

#endif