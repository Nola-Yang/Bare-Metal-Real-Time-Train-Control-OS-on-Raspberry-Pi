#include "task.h"
#include <stdint.h>

#define SPSR_MASK_ALL 0x3C0
#define SPSR_EL0t     0x0
#define SPSR_FOR_EL0t (SPSR_MASK_ALL | SPSR_EL0t)

extern void activate_first_task(void);

static task_descriptor_t first_task;
static uint8_t user_stack[4096] __attribute__((aligned(16)));

#if !defined(MMU)
#include <stddef.h>

// define our own memset to avoid SIMD instructions emitted from the compiler
void* memset(void *s, int c, size_t n) {
	for (char* it = (char*)s; n > 0; --n) *it++ = c;
	return s;
}

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void* memcpy(void* dest, const void* src, size_t n) {
	char* sit = (char*)src;
	char* cdest = (char*)dest;
	for (size_t i = 0; i < n; ++i) *cdest++ = *sit++;
	return dest;
}
#endif


//Todo
static void user_entry(void) {
    for (;;) {
        __asm__ volatile("svc #0");
    }
}

static task_descriptor_t *create_first_task(void) {
    task_descriptor_t *t = &first_task;

    memset(t, 0, sizeof(*t));
    t->tid = 1;
    t->parent_tid = 0;
    t->priority = 0;
    t->state = TASK_STATE_READY;

    t->tf.elr_el1 = (uint64_t)user_entry;
    t->tf.sp_el0 = (uint64_t)(user_stack + sizeof(user_stack));
    t->tf.spsr_el1 = SPSR_FOR_EL0t;

    return t;
}

/*
 * Temporary syscall_dispatch for compilation.
 */
void syscall_dispatch(trapframe_t *tf) {
    (void)tf;
}

int kmain(void) {
    task_descriptor_t *t = create_first_task();

    set_current_task(t);
    activate_first_task();

    for (;;) {}
    return 0;
}
