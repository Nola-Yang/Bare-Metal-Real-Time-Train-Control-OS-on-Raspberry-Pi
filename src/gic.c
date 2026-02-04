#include "gic.h"

void gic_init(void) {    
    GICD_CTLR = 1;  // Enable distributor
    GICC_CTLR = 1; // Enable CPU interface
    GICC_PMR = 0xFF;   // priority mask
}

void gic_enable_interrupt(uint32_t irq_id) {
    uint32_t reg_index = irq_id / 32;
    uint32_t bit = irq_id % 32;
    GICD_ISENABLER(reg_index) = (1U << bit);
}

void gic_disable_interrupt(uint32_t irq_id) {
    uint32_t reg_index = irq_id / 32;
    uint32_t bit = irq_id % 32;
    GICD_ICENABLER(reg_index) = (1U << bit);
}

void gic_route_interrupt_to_cpu0(uint32_t irq_id) {
    // Each ITARGETSR register handles 4 interrupts (8 bits each)
    uint32_t reg_index = irq_id / 4;
    uint32_t byte_offset = irq_id % 4;

    // Read current value
    uint32_t val = GICD_ITARGETSR(reg_index);

    // Clear the byte for this interrupt and set CPU 0 (bit 0 = CPU 0)
    val &= ~(0xFF << (byte_offset * 8));
    val |= (1 << (byte_offset * 8));  // Route to CPU 0

    GICD_ITARGETSR(reg_index) = val;
}

uint32_t gic_read_iar(void) {
    return GICC_IAR;
}

void gic_write_eoir(uint32_t irq_id) {
    GICC_EOIR = irq_id;
}