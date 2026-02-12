#ifndef SYSCALL_H
#define SYSCALL_H

#include <stdint.h>



// Syscall numbers
#define SYS_CREATE      0
#define SYS_MYTID       1
#define SYS_MYPARENTTID 2
#define SYS_YIELD       3
#define SYS_EXIT        4
#define SYS_SEND        5
#define SYS_RECEIVE     6
#define SYS_REPLY       7
#define SYS_AWAITEVENT  8
#define SYS_SHUTDOWN    9

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

static inline int Send(int tid, const char *msg, int msglen, char *reply, int rplen) {
    register int64_t r0 __asm__("x0") = tid;
    register uint64_t r1 __asm__("x1") = (uint64_t)msg;
    register int64_t r2 __asm__("x2") = msglen;
    register uint64_t r3 __asm__("x3") = (uint64_t)reply;
    register int64_t r4 __asm__("x4") = rplen;
    register int64_t r8 __asm__("x8") = SYS_SEND;
    __asm__ volatile("svc #0" : "+r"(r0) : "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r8) : "memory");
    return (int)r0;
}

static inline int Receive(int *tid, char *msg, int msglen) {
    register uint64_t r0 __asm__("x0") = (uint64_t)tid;
    register uint64_t r1 __asm__("x1") = (uint64_t)msg;
    register int64_t r2 __asm__("x2") = msglen;
    register int64_t r8 __asm__("x8") = SYS_RECEIVE;
    __asm__ volatile("svc #0" : "+r"(r0) : "r"(r1), "r"(r2), "r"(r8) : "memory");
    return (int)r0;
}

static inline int Reply(int tid, const char *reply, int rplen) {
    register int64_t r0 __asm__("x0") = tid;
    register uint64_t r1 __asm__("x1") = (uint64_t)reply;
    register int64_t r2 __asm__("x2") = rplen;
    register int64_t r8 __asm__("x8") = SYS_REPLY;
    __asm__ volatile("svc #0" : "+r"(r0) : "r"(r1), "r"(r2), "r"(r8) : "memory");
    return (int)r0;
}

static inline int AwaitEvent(int eventid) {
    register int64_t r0 __asm__("x0") = eventid;
    register int64_t r8 __asm__("x8") = SYS_AWAITEVENT;
    __asm__ volatile("svc #0" : "+r"(r0) : "r"(r8) : "memory");
    return (int)r0;
}

static inline void Shutdown(void) {
    register int64_t r8 __asm__("x8") = SYS_SHUTDOWN;
    __asm__ volatile("svc #0" : : "r"(r8) : "memory");
    __builtin_unreachable();
}

#endif /* SYSCALL_H */