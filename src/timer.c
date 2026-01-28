#include <stdint.h>
#include "rpi.h"
#include "timer.h"


static char* const TIMER_BASE = (char*)(MMIO_BASE + 0x3000);
#define TIMER_REG(offset) (*(volatile uint32_t*)(TIMER_BASE + offset));

static const uint32_t TIMER_CLO = 0x04;


uint32_t read_timer() {
    return TIMER_REG(TIMER_CLO);
}