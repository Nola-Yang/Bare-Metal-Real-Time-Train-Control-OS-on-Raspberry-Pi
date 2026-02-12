#ifndef KASSERT_H
#define KASSERT_H

#include <stdarg.h>

__attribute__((noreturn)) void panic(const char *fmt, ...);

#define KASSERT(cond) \
	do { \
		if (!(cond)) { \
			panic("KASSERT %s:%d: %s\r\n", __FILE__, __LINE__, #cond); \
		} \
	} while (0)

#endif /* KASSERT_H */
