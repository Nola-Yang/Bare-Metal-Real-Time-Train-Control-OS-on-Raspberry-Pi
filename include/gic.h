#ifndef _gic_h_
#define _gic_h_ 1

#include <stdint.h>

// GIC-400 Base addresses 
#define GIC_BASE    0xFF840000
#define GICD_BASE   (GIC_BASE + 0x1000)
#define GICC_BASE   (GIC_BASE + 0x2000)

// GICD (Distributor) registers
#define GICD_CTLR           (*(volatile uint32_t *)(GICD_BASE + 0x000))
#define GICD_ISENABLER(n)   (*(volatile uint32_t *)(GICD_BASE + 0x100 + 4 * (n)))
#define GICD_ICENABLER(n)   (*(volatile uint32_t *)(GICD_BASE + 0x180 + 4 * (n)))
#define GICD_ITARGETSR(n)   (*(volatile uint32_t *)(GICD_BASE + 0x800 + 4 * (n)))

// GICC (CPU Interface) registers
#define GICC_CTLR           (*(volatile uint32_t *)(GICC_BASE + 0x000))
#define GICC_PMR            (*(volatile uint32_t *)(GICC_BASE + 0x004))
#define GICC_IAR            (*(volatile uint32_t *)(GICC_BASE + 0x00C))
#define GICC_EOIR           (*(volatile uint32_t *)(GICC_BASE + 0x010))

// System Timer interrupt IDs 
// C0-C3 are the 4 System Timer Compare registers
// C1 interrupt ID = 97, C3 interrupt ID = 99
#define TIMER_C1_IRQ_ID     97
#define TIMER_C3_IRQ_ID     99

// Event IDs for AwaitEvent
#define EVENT_TIMER_C1      0
#define EVENT_TIMER_C3      1
#define EVENT_COUNT         2

// Initialize GIC for interrupt handling
void gic_init(void);

// Enable a specific interrupt
void gic_enable_interrupt(uint32_t irq_id);

// Disable a specific interrupt
void gic_disable_interrupt(uint32_t irq_id);

// Route interrupt to CPU 0
void gic_route_interrupt_to_cpu0(uint32_t irq_id);

// Read interrupt acknowledge register (returns interrupt ID)
uint32_t gic_read_iar(void);

// End of interrupt - signal completion
void gic_write_eoir(uint32_t irq_id);

#endif /* _gic_h_ */