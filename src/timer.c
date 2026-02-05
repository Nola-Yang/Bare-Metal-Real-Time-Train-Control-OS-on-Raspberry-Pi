#include <stdint.h>
#include "rpi.h"
#include "timer.h"


static char* const TIMER_BASE = (char*)(MMIO_BASE + 0x3000);
#define TIMER_REG(offset) (*(volatile uint32_t*)(TIMER_BASE + (offset)))

// System Timer register offsets
static const uint32_t TIMER_CS  = 0x00;  // Control/Status
static const uint32_t TIMER_CLO = 0x04;  // Counter Lower 32 bits
static const uint32_t TIMER_CHI = 0x08;  // Counter Upper 32 bits
static const uint32_t TIMER_C1  = 0x10;  // Compare 1
static const uint32_t TIMER_C3  = 0x18;  // Compare 3


uint64_t read_timer(void) {
    uint32_t hi, lo;

    do {
        hi = TIMER_REG(TIMER_CHI);
        lo = TIMER_REG(TIMER_CLO);
    } while (hi != TIMER_REG(TIMER_CHI));

    return ((uint64_t)hi << 32) | lo;
}

void timer_set_c1(uint32_t value) {
    TIMER_REG(TIMER_C1) = value;
}

void timer_set_c3(uint32_t value) {
    TIMER_REG(TIMER_C3) = value;
}

void timer_clear_c1(void) {
    TIMER_REG(TIMER_CS) = TIMER_CS_M1;
}

void timer_clear_c3(void) {
    TIMER_REG(TIMER_CS) = TIMER_CS_M3;
}

uint32_t timer_get_c1(void) {
    return TIMER_REG(TIMER_C1);
}

uint32_t timer_get_c3(void) {
    return TIMER_REG(TIMER_C3);
}
