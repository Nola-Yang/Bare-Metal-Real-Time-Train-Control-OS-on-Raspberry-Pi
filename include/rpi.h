#ifndef _rpi_h_
#define _rpi_h_ 1

#include <stdint.h>

static char* const MMIO_BASE = (char*)0xFE000000;

void gpio_init(void);

// GPIO interrupt functions for MCP2515
void gpio_enable_falling_edge_detect(uint32_t pin);
void gpio_clear_event(uint32_t pin);
uint32_t gpio_check_event(uint32_t pin);

// Get event detect status for GPIO pin.
uint32_t gpio_get_event_detect_status(uint32_t pin);

// Clear event detect status for GPIO pin.
void gpio_clr_event_detect_status(uint32_t pin);

#endif /* _rpi_h_ */
