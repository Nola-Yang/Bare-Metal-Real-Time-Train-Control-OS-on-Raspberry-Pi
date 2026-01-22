#ifndef SYSCALL_H
#define SYSCALL_H

#include <stdint.h>



// Syscall numbers 
#define SYS_CREATE      0
#define SYS_MYTID       1
#define SYS_MYPARENTTID 2
#define SYS_YIELD       3
#define SYS_EXIT        4

static inline int Create(int priority, void (*function)()) {
    register int64_t r0 __asm__("x0") = priority;
    register uint64_t r1 __asm__("x1") = (uint64_t)function;
    register int64_t r8 __asm__("x8") = SYS_CREATE;
    __asm__ volatile("svc #0" : "+r"(r0) : "r"(r1), "r"(r8) : "memory");
    return (int)r0;
}

static inline int MyTid(void) {
    register int64_t r0 __asm__("x0");
    register int64_t r8 __asm__("x8") = SYS_MYTID;
    __asm__ volatile("svc #0" : "=r"(r0) : "r"(r8) : "memory");
    return (int)r0;
}

static inline int MyParentTid(void) {
    register int64_t r0 __asm__("x0");
    register int64_t r8 __asm__("x8") = SYS_MYPARENTTID;
    __asm__ volatile("svc #0" : "=r"(r0) : "r"(r8) : "memory");
    return (int)r0;
}

static inline void Yield(void) {
    register int64_t r8 __asm__("x8") = SYS_YIELD;
    __asm__ volatile("svc #0" : : "r"(r8) : "memory");
}

static inline void Exit(void) {
    register int64_t r8 __asm__("x8") = SYS_EXIT;
    __asm__ volatile("svc #0" : : "r"(r8) : "memory");
    __builtin_unreachable();
}

#endif /* SYSCALL_H */