/*
 * Minimal task descriptor structure
 * Todo: extend this with additional fields as needed.
 *
 * The trapframe MUST be at offset 0 within the TD.
 * This allows TPIDR_EL1 to point directly to the trapframe.
 */

#ifndef TASK_H
#define TASK_H

#include "trapframe.h"

/* Task states */
#define TASK_STATE_FREE     0
#define TASK_STATE_READY    1
#define TASK_STATE_RUNNING  2
#define TASK_STATE_EXITED   3

struct task_descriptor;

typedef struct task_descriptor {
    trapframe_t tf;                     

    int tid;                            
    int parent_tid;                     
    int priority;                      
    int state;                          
    struct task_descriptor *next;       
} task_descriptor_t;

/* Verify trapframe is at offset 0 */
_Static_assert(__builtin_offsetof(task_descriptor_t, tf) == 0,
               "trapframe must be at offset 0 in task_descriptor");


static inline void set_current_task(task_descriptor_t *td) {
    __asm__ volatile("msr tpidr_el1, %0" :: "r"(td) : "memory");
}

static inline task_descriptor_t *get_current_task(void) {
    task_descriptor_t *td;
    __asm__ volatile("mrs %0, tpidr_el1" : "=r"(td));
    return td;
}

#endif /* TASK_H */