#ifndef _uart_h_
#define _uart_h_ 1

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#define CONSOLE 0

// UART interrupt bits
#define UART_INT_RX  (1 << 4)
#define UART_INT_TX  (1 << 5)
#define UART_INT_RT  (1 << 6)  // RX timeout
#define UART_INT_FE  (1 << 7)  // Framing error
#define UART_INT_PE  (1 << 8)  // Parity error
#define UART_INT_BE  (1 << 9)  // Break error
#define UART_INT_OE  (1 << 10) // Overrun error

// Modem/status interrupts may useleess? keep for possible UART_FLOWCTL
#define UART_INT_DSR (1 << 3)
#define UART_INT_DCD (1 << 2)
#define UART_INT_CTS (1 << 1)
#define UART_INT_RI  (1 << 0)

#define UART_INT_ERROR_MASK  (UART_INT_FE | UART_INT_PE | UART_INT_BE | UART_INT_OE)
#define UART_INT_STATUS_MASK (UART_INT_DSR | UART_INT_DCD | UART_INT_CTS | UART_INT_RI)

void uart_config_and_enable(size_t line);
void uart_panic_vprintf(size_t line, const char *fmt, va_list va);
void uart_panic_printf(size_t line, const char *fmt, ... );


// Interrupt control functions
void uart_enable_rx_interrupt(size_t line);
void uart_disable_rx_interrupt(size_t line);
void uart_enable_tx_interrupt(size_t line);
void uart_disable_tx_interrupt(size_t line);
void uart_clear_rx_interrupt(size_t line);
void uart_clear_tx_interrupt(size_t line);
void uart_enable_error_interrupts(size_t line);
void uart_disable_error_interrupts(size_t line);
void uart_clear_error_interrupts(size_t line);
void uart_enable_status_interrupts(size_t line);
void uart_disable_status_interrupts(size_t line);
void uart_clear_status_interrupts(size_t line);
uint32_t uart_read_mis(size_t line);

// Non-blocking I/O
int uart_rx_ready(size_t line);
int uart_tx_ready(size_t line);
int uart_getc_nonblocking(size_t line);
int uart_putc_nonblocking(size_t line, char c);

#endif /* uart.h */
