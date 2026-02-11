#ifndef _rpi_h_
#define _rpi_h_ 1

#include <stdint.h>

static char* const MMIO_BASE = (char*)0xFE000000;

void gpio_init(void);

// GPIO interrupt functions for MCP2515
void gpio_init_interrupt(void);
uint32_t gpio_get_event_detect_status(uint32_t pin);
void gpio_clr_event_detect_status(uint32_t pin);
void gpio_enable_can_interrupt(void);
void gpio_disable_can_interrupt(void);

#endif /* _rpi_h_ */
