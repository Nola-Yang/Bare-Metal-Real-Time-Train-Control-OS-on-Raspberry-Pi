#include "gic.h"

void gic_init(void) {    
    // Disable while configuring
    GICD_CTLR = 0;
    GICC_CTLR = 0;

    // Route timer interrupts through Group 1 
    uint32_t c1_reg = TIMER_C1_IRQ_ID / 32;
    uint32_t c1_bit = TIMER_C1_IRQ_ID % 32;
    uint32_t c3_reg = TIMER_C3_IRQ_ID / 32;
    uint32_t c3_bit = TIMER_C3_IRQ_ID % 32;
    uint32_t arch_reg = ARCH_TIMER_IRQ_ID / 32;
    uint32_t arch_bit = ARCH_TIMER_IRQ_ID % 32;
    GICD_IGROUPR(c1_reg) |= (1U << c1_bit);
    GICD_IGROUPR(c3_reg) |= (1U << c3_bit);
    GICD_IGROUPR(arch_reg) |= (1U << arch_bit);

    // Accept all priorities
    GICC_PMR = 0xFF;

    // Enable both Group 0 and Group 1 at both distributor and CPU interface.
    GICD_CTLR = 0x3;
    GICC_CTLR = 0x3;
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

void gic_set_priority(uint32_t irq_id, uint8_t priority) {
    uint32_t reg_index = irq_id / 4;
    uint32_t byte_offset = irq_id % 4;

    uint32_t val = GICD_IPRIORITYR(reg_index);
    val &= ~(0xFF << (byte_offset * 8));
    val |= ((uint32_t)priority << (byte_offset * 8));
    GICD_IPRIORITYR(reg_index) = val;
}

uint32_t gic_read_iar(void) {
    return GICC_IAR;
}

void gic_write_eoir(uint32_t irq_id) {
    GICC_EOIR = irq_id;
}