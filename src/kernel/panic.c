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

	uart_config_and_enable(CONSOLE);
	uart_panic_printf(CONSOLE, "\r\n\r\nPANIC: ");

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

__attribute__((noreturn)) void panic_exception(const char *reason,
                                               uint64_t esr,
                                               uint64_t elr,
                                               uint64_t far) {
    panic("Unhandled exception: %s esr=0x%lx elr=0x%lx far=0x%lx\r\n",
          reason, esr, elr, far);
}
