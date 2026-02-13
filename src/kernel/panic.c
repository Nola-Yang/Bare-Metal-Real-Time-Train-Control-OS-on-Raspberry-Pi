#include <stdarg.h>
#include <stdint.h>
#include "kassert.h"
#include "uart.h"

/* GCC stack protector support */
uintptr_t __stack_chk_guard = 0x595e9fbd94fda766ULL;

__attribute__((noreturn)) void __stack_chk_fail(void) {
	panic("Stack smashing detected!\r\n");
}

__attribute__((noreturn)) void panic(const char *fmt, ...) {
	__asm__ volatile("msr daifset, #0xf" ::: "memory");

	va_list va;
	va_start(va, fmt);
	uart_panic_vprintf(CONSOLE, fmt, va);
	va_end(va);

	for (;;) {
		__asm__ volatile("wfi");
	}
}
