#ifndef _train_runtime_h_
#define _train_runtime_h_ 1

#include "task_scheduler.h"

// Runtime coordinator: owns startup/shutdown and routes structured events.
void runtime_core_task(void);

#endif /* _train_runtime_h_ */
