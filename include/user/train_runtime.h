#ifndef _train_runtime_h_
#define _train_runtime_h_ 1

#include "task_scheduler.h"

// Runtime worker: owns track/pos/demo/rv state changes.
void train_runtime_task(void);

#endif /* _train_runtime_h_ */
