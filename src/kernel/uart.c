#include <stdarg.h>
#include <stdint.h>
#include "uart.h"
#include "util.h"
#include "rpi.h"

static char* const UART_BASE = (char*)(MMIO_BASE + 0x201000);
#define UART_REG(line, offset) (*(volatile uint32_t*)(UART_BASE + line * 0x200 + offset))

// UART register offsets
static const uint32_t UART_DR   = 0x00;
static const uint32_t UART_FR   = 0x18;
static const uint32_t UART_IBRD = 0x24;
static const uint32_t UART_FBRD = 0x28;
static const uint32_t UART_LCRH = 0x2c;
static const uint32_t UART_CR   = 0x30;
static const uint32_t UART_IFLS = 0x34;

// masks for specific fields in the UART registers
static const uint32_t UART_FR_BUSY = 0x08;
static const uint32_t UART_FR_RXFE = 0x10;
static const uint32_t UART_FR_TXFF = 0x20;
static const uint32_t UART_FR_RXFF = 0x40;
static const uint32_t UART_FR_TXFE = 0x80;

// Interrupt registers
static const uint32_t UART_IMSC = 0x38;  // Interrupt Mask Set/Clear
static const uint32_t UART_MIS  = 0x40;  // Masked Interrupt Status
static const uint32_t UART_ICR  = 0x44;  // Interrupt Clear

// Interrupt mask bits 
static const uint32_t UART_INT_RX_BIT = (1 << 4);
static const uint32_t UART_INT_TX_BIT = (1 << 5);
static const uint32_t UART_INT_RT_BIT = (1 << 6);
static const uint32_t UART_INT_ERR_BITS = (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);
static const uint32_t UART_INT_STATUS_BITS = (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);

static const uint32_t UART_CR_UARTEN =   0x01;
static const uint32_t UART_CR_LBE    =   0x80;
static const uint32_t UART_CR_TXE    =  0x100;
static const uint32_t UART_CR_RXE    =  0x200;
static const uint32_t UART_CR_RTS    =  0x800;
static const uint32_t UART_CR_RTSEN  = 0x4000;
static const uint32_t UART_CR_CTSEN  = 0x8000;

static const uint32_t UART_LCRH_PEN       = 0x02;
static const uint32_t UART_LCRH_EPS       = 0x04;
static const uint32_t UART_LCRH_STP2      = 0x08;
static const uint32_t UART_LCRH_FEN       = 0x10;
static const uint32_t UART_LCRH_WLEN_LOW  = 0x20;
static const uint32_t UART_LCRH_WLEN_HIGH = 0x40;

// FIFO interrupt trigger level encodings
static const uint32_t UART_IFLS_1_2 = 0x2;

static const uint32_t UART_IFLS_RX_SHIFT = 3;
static const uint32_t UART_IFLS_TX_SHIFT = 0;

// Default FIFO trigger levels
static const uint32_t UART_IFLS_RX_LEVEL = UART_IFLS_1_2;
static const uint32_t UART_IFLS_TX_LEVEL = UART_IFLS_1_2;

static void uart_putc_blocking(size_t line, char c) {
	while (UART_REG(line, UART_FR) & UART_FR_TXFF);
	UART_REG(line, UART_DR) = c;
}

static void uart_puts_blocking(size_t line, const char *buf) {
	if (buf == NULL) {
		buf = "(null)";
	}
	while (*buf) {
		uart_putc_blocking(line, *buf++);
	}
}

static void u64_to_hex(uint64_t num, char *buf) {
	static const char hexdigits[] = "0123456789ABCDEF";
	char tmp[16];
	size_t i = 0;

	if (num == 0) {
		buf[0] = '0';
		buf[1] = '\0';
		return;
	}

	while (num != 0 && i < sizeof(tmp)) {
		tmp[i++] = hexdigits[num & 0xF];
		num >>= 4;
	}

	size_t j = 0;
	while (i > 0) {
		buf[j++] = tmp[--i];
	}
	buf[j] = '\0';
}

static void u64_to_dec(uint64_t num, char *buf) {
	char tmp[20];
	size_t i = 0;

	if (num == 0) {
		buf[0] = '0';
		buf[1] = '\0';
		return;
	}

	while (num != 0 && i < sizeof(tmp)) {
		tmp[i++] = (char)('0' + (num % 10));
		num /= 10;
	}

	size_t j = 0;
	while (i > 0) {
		buf[j++] = tmp[--i];
	}
	buf[j] = '\0';
}

static void i64_to_dec(int64_t num, char *buf) {
	uint64_t mag;

	if (num < 0) {
		*buf++ = '-';
		mag = (uint64_t)(-(num + 1)) + 1;
	} else {
		mag = (uint64_t)num;
	}

	u64_to_dec(mag, buf);
}

static void uart_internal_printf(size_t line, const char *fmt, va_list va) {
	char ch, buf[32];

	while ((ch = *(fmt++))) {
		if (ch != '%') {
			uart_putc_blocking(line, ch);
			continue;
		}

		ch = *(fmt++);
		if (ch == '\0') {
			return;
		}

		if (ch == 'l') {
			char next = *(fmt++);
			if (next == '\0') {
				return;
			}
			switch (next) {
				case 'x':
					u64_to_hex(va_arg(va, uint64_t), buf);
					uart_puts_blocking(line, buf);
					break;
				case 'u':
					u64_to_dec(va_arg(va, uint64_t), buf);
					uart_puts_blocking(line, buf);
					break;
				case 'd':
					i64_to_dec(va_arg(va, int64_t), buf);
					uart_puts_blocking(line, buf);
					break;
				default:
					uart_putc_blocking(line, '%');
					uart_putc_blocking(line, 'l');
					uart_putc_blocking(line, next);
					break;
			}
			continue;
		}

		switch (ch) {
			case 'u':
				ui2a(va_arg(va, unsigned int), 10, buf);
				uart_puts_blocking(line, buf);
				break;
			case 'd':
				i2a(va_arg(va, int), buf);
				uart_puts_blocking(line, buf);
				break;
			case 'x':
				ui2a(va_arg(va, unsigned int), 16, buf);
				uart_puts_blocking(line, buf);
				break;
			case 'p':
				uart_puts_blocking(line, "0x");
				u64_to_hex((uint64_t)(uintptr_t)va_arg(va, void *), buf);
				uart_puts_blocking(line, buf);
				break;
			case 's':
				uart_puts_blocking(line, va_arg(va, char *));
				break;
			case 'c':
				uart_putc_blocking(line, (char)va_arg(va, int));
				break;
			case '%':
				uart_putc_blocking(line, ch);
				break;
			default:
				uart_putc_blocking(line, '%');
				uart_putc_blocking(line, ch);
				break;
		}
	}
}

// Configure the line properties (e.g, parity, baud rate) of a UART and ensure that it is enabled
void uart_config_and_enable(size_t line) {
	uint32_t baud_ival, baud_fval;
	uint32_t flag = UART_LCRH_FEN;

	switch (line) {
		// setting baudrate to approx. 115246.09844 (best we can do); 1 stop bit
		case CONSOLE: baud_ival =   26; baud_fval = 2; break;
		default: return;
	}

	// line control registers should not be changed while the UART is enabled, so disable it
	uint32_t cr_state = UART_REG(line, UART_CR);
	UART_REG(line, UART_CR) = cr_state & ~UART_CR_UARTEN;

	// set the baud rate
	UART_REG(line, UART_IBRD) = baud_ival;
	UART_REG(line, UART_FBRD) = baud_fval;

	// set the line control registers: 8 bit, no parity, 1 or 2 stop bits, FIFOs enabled
	UART_REG(line, UART_LCRH) = UART_LCRH_WLEN_HIGH | UART_LCRH_WLEN_LOW | flag;

	// Configure FIFO interrupt trigger levels
	UART_REG(line, UART_IFLS) = (UART_IFLS_RX_LEVEL << UART_IFLS_RX_SHIFT) |
	                            (UART_IFLS_TX_LEVEL << UART_IFLS_TX_SHIFT);

	// re-enable the UART; enable both transmit and receive regardless of previous state
	uint32_t cr = cr_state | UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;

	UART_REG(line, UART_CR) = cr;

	// Enable error interrupts to detect UART faults
	UART_REG(line, UART_IMSC) |= UART_INT_ERR_BITS;
}

void uart_panic_vprintf(size_t line, const char *fmt, va_list va) {
	uart_internal_printf(line, fmt, va);
}

void uart_panic_printf(size_t line, const char *fmt, ... ) {
	va_list va;

	va_start(va, fmt);
	uart_internal_printf(line, fmt, va);
	va_end(va);
}

void uart_debug_printf(size_t line, const char *fmt, ... ) {
#ifdef VERBOSE
	va_list va;

	va_start(va, fmt);
	uart_internal_printf(line, fmt, va);
	va_end(va);
#else
	(void)line;
	(void)fmt;
	// for complier warnings
#endif
}

void uart_debug_move_cursor(size_t line, uint32_t line_pos, uint32_t cursor_pos) {
	uart_debug_printf(line, "\033[%u;%uH", line_pos, cursor_pos);
}

// Interrupt control functions

void uart_enable_rx_interrupt(size_t line) {
	UART_REG(line, UART_IMSC) |= (UART_INT_RX_BIT | UART_INT_RT_BIT);
}

void uart_disable_rx_interrupt(size_t line) {
	UART_REG(line, UART_IMSC) &= ~(UART_INT_RX_BIT | UART_INT_RT_BIT);
}

void uart_enable_tx_interrupt(size_t line) {
	UART_REG(line, UART_IMSC) |= UART_INT_TX_BIT;
}

void uart_disable_tx_interrupt(size_t line) {
	UART_REG(line, UART_IMSC) &= ~UART_INT_TX_BIT;
}

void uart_clear_rx_interrupt(size_t line) {
	UART_REG(line, UART_ICR) = (UART_INT_RX_BIT | UART_INT_RT_BIT);
}

void uart_clear_tx_interrupt(size_t line) {
	UART_REG(line, UART_ICR) = UART_INT_TX_BIT;
}

void uart_enable_error_interrupts(size_t line) {
	UART_REG(line, UART_IMSC) |= UART_INT_ERR_BITS;
}

void uart_disable_error_interrupts(size_t line) {
	UART_REG(line, UART_IMSC) &= ~UART_INT_ERR_BITS;
}

void uart_clear_error_interrupts(size_t line) {
	UART_REG(line, UART_ICR) = UART_INT_ERR_BITS;
}

void uart_enable_status_interrupts(size_t line) {
	UART_REG(line, UART_IMSC) |= UART_INT_STATUS_BITS;
}

void uart_disable_status_interrupts(size_t line) {
	UART_REG(line, UART_IMSC) &= ~UART_INT_STATUS_BITS;
}

void uart_clear_status_interrupts(size_t line) {
	UART_REG(line, UART_ICR) = UART_INT_STATUS_BITS;
}

uint32_t uart_read_mis(size_t line) {
	return UART_REG(line, UART_MIS);
}

// Non-blocking I/O

int uart_rx_ready(size_t line) {
	return !(UART_REG(line, UART_FR) & UART_FR_RXFE);
}

int uart_tx_ready(size_t line) {
	return !(UART_REG(line, UART_FR) & UART_FR_TXFF);
}

int uart_getc_nonblocking(size_t line) {
	if (UART_REG(line, UART_FR) & UART_FR_RXFE) {
		return -1;  
	}
	return UART_REG(line, UART_DR);
}

int uart_putc_nonblocking(size_t line, char c) {
	if (UART_REG(line, UART_FR) & UART_FR_TXFF) {
		return -1;  
	}
	UART_REG(line, UART_DR) = c;
	return 0;
}