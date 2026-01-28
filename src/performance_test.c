#include "performance_test.h"
#include "syscall.h"
#include "uart.h"


static void print_test_spec() {
    #ifdef OPT
    uart_printf(CONSOLE, "OPT: enabled\r\n");
    #else
    uart_printf(CONSOLE, "OPT: disabled\r\n");
    #endif

    #ifdef DCACHE
	uart_printf(CONSOLE, "D cache: enabled\r\n");
	#else
	uart_printf(CONSOLE, "D cache: disabled\r\n");
	#endif

	#ifdef ICACHE
	uart_printf(CONSOLE, "I cache: enabled\r\n");
	#else
	uart_printf(CONSOLE, "I cache: disabled\r\n");
	#endif
}


void perform_test_run() {

}