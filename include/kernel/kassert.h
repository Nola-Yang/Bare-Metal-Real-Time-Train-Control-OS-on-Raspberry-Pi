#ifndef KASSERT_H
#define KASSERT_H

#include <stdint.h>

__attribute__((noreturn)) void panic(const char *fmt, ...);
__attribute__((noreturn)) void kassert_fail_kernel(const char *file, int line, const char *cond);

#ifdef USER_SPACE
#include "syscall.h"

static inline __attribute__((noreturn)) void kassert_fail_user(const char *file, int line, const char *cond) {
	register uint64_t r0 __asm__("x0") = (uint64_t)file;
	register int64_t r1 __asm__("x1") = (int64_t)line;
	register uint64_t r2 __asm__("x2") = (uint64_t)cond;
	register int64_t r8 __asm__("x8") = SYS_KASSERT_FAIL;
	__asm__ volatile("svc #0" : : "r"(r0), "r"(r1), "r"(r2), "r"(r8) : "memory");
	for (;;) {
	}
}

#define KASSERT_FAIL(file, line, cond) kassert_fail_user((file), (line), (cond))
#else
#define KASSERT_FAIL(file, line, cond) kassert_fail_kernel((file), (line), (cond))
#endif

#define KASSERT(cond) \
	do { \
		if (!(cond)) { \
			KASSERT_FAIL(__FILE__, __LINE__, #cond); \
		} \
	} while (0)

#endif /* KASSERT_H */
