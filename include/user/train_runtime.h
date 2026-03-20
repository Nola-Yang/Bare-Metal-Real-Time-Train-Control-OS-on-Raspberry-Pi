#ifndef _train_runtime_h_
#define _train_runtime_h_ 1

#include "task_scheduler.h"

// Runtime worker: owns track/pos/demo/rv/retry state changes.
void train_runtime_task(void);

// Dead-train retry helpers used by position tracking.
void retry_dead_train_task(void);
void add_dead_train_to_retry(int train_num);

#endif /* _train_runtime_h_ */
