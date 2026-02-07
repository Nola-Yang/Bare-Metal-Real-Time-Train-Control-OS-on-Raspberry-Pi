#ifndef _uart_h_
#define _uart_h_ 1

#include <stddef.h>
#include <stdint.h>

#define CONSOLE 0

// UART interrupt bits
#define UART_INT_RX  (1 << 4)
#define UART_INT_TX  (1 << 5)

void uart_config_and_enable(size_t line);
char uart_getc(size_t line);
void uart_putc(size_t line, char c);
void uart_putl(size_t line, const char *buf, size_t blen);
void uart_puts(size_t line, const char *buf);
void uart_printf(size_t line, const char *fmt, ...);

void uart_debug_printf(size_t line, const char *fmt, ... );

// Interrupt control functions
void uart_enable_rx_interrupt(size_t line);
void uart_disable_rx_interrupt(size_t line);
void uart_enable_tx_interrupt(size_t line);
void uart_disable_tx_interrupt(size_t line);
void uart_clear_rx_interrupt(size_t line);
void uart_clear_tx_interrupt(size_t line);
uint32_t uart_read_mis(size_t line);

// Non-blocking I/O
int uart_rx_ready(size_t line);
int uart_tx_ready(size_t line);
int uart_getc_nonblocking(size_t line);
int uart_putc_nonblocking(size_t line, char c);

#endif /* uart.h */
