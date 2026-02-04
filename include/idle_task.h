#ifndef _idle_task_h_
#define _idle_task_h_ 1

// lowest priority (runs when nothing else can)
#define IDLE_TASK_PRIORITY 0

void idle_task(void);

// Get idle percentage (0-100)
int get_idle_percentage(void);

#endif