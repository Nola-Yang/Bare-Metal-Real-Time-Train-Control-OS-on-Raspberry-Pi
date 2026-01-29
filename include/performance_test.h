#ifndef PERFORMANCE_TEST_H
#define PERFORMANCE_TEST_H


// Only 2 tasks for performance testing (created directly by first_user_task)
// Task A: receiver in phase 1, sender in phase 2
// Task B: sender in phase 1, receiver in phase 2
#define TASK_A_TID 1
#define TASK_B_TID 2


// perform_test_run: Performs performance testing on the SRR pattern
void perform_test_run();


#endif