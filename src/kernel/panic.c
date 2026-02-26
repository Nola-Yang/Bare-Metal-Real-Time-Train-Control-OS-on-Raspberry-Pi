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

	/* Print panic in command area (rows 27-47).*/
	uart_panic_printf(CONSOLE, "\033[27;47r");
	uart_panic_printf(CONSOLE, "\033[27;1H\033[2K");
	uart_panic_printf(CONSOLE, "PANIC: ");

	va_list va;
	va_start(va, fmt);
	uart_panic_vprintf(CONSOLE, fmt, va);
	va_end(va);
	uart_panic_printf(CONSOLE, "\r\n");

	uart_panic_flush(CONSOLE);

	for (;;) {
		__asm__ volatile("wfi");
	}
}
