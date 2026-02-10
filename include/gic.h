#ifndef _gic_h_
#define _gic_h_ 1

#include <stdint.h>

// GIC-400 Base addresses 
#define GIC_BASE    ((uintptr_t)0xFF840000UL)
#define GICD_BASE   (GIC_BASE + 0x1000UL)
#define GICC_BASE   (GIC_BASE + 0x2000UL)

// GICD (Distributor) registers
#define GICD_CTLR           (*(volatile uint32_t *)(GICD_BASE + 0x000UL))
#define GICD_IGROUPR(n)     (*(volatile uint32_t *)(GICD_BASE + 0x080UL + 4UL * (n)))
#define GICD_ISENABLER(n)   (*(volatile uint32_t *)(GICD_BASE + 0x100UL + 4UL * (n)))
#define GICD_ICENABLER(n)   (*(volatile uint32_t *)(GICD_BASE + 0x180UL + 4UL * (n)))
#define GICD_IPRIORITYR(n)  (*(volatile uint32_t *)(GICD_BASE + 0x400UL + 4UL * (n)))
#define GICD_ITARGETSR(n)   (*(volatile uint32_t *)(GICD_BASE + 0x800UL + 4UL * (n)))

// GICC (CPU Interface) registers
#define GICC_CTLR           (*(volatile uint32_t *)(GICC_BASE + 0x000UL))
#define GICC_PMR            (*(volatile uint32_t *)(GICC_BASE + 0x004UL))
#define GICC_IAR            (*(volatile uint32_t *)(GICC_BASE + 0x00CUL))
#define GICC_EOIR           (*(volatile uint32_t *)(GICC_BASE + 0x010UL))

// System Timer interrupt IDs 
// C0-C3 are the 4 System Timer Compare registers
// C1 interrupt ID = 97, C3 interrupt ID = 99
#define TIMER_C1_IRQ_ID     97
#define TIMER_C3_IRQ_ID     99

// UART0 interrupt ID (PL011)
#define UART0_IRQ_ID        57

// GPIO Bank 0 interrupt ID (GPIO 0-31, the GPIO17 is used for MCP2515 INT)
#define GPIO_BANK0_IRQ_ID   145

// Event IDs for AwaitEvent
#define EVENT_TIMER_C1      0
#define EVENT_TIMER_C3      1
#define EVENT_UART_RX       2   // UART RX data available
#define EVENT_UART_TX       3   // UART TX ready
#define EVENT_CAN_RX        4   // MCP2515 INT
#define EVENT_COUNT         5

// Initialize GIC for interrupt handling
void gic_init(void);

// Enable a specific interrupt
void gic_enable_interrupt(uint32_t irq_id);

// Disable a specific interrupt
void gic_disable_interrupt(uint32_t irq_id);

// Route interrupt to CPU 0
void gic_route_interrupt_to_cpu0(uint32_t irq_id);

// Set interrupt priority (0 = highest, 255 = lowest)
void gic_set_priority(uint32_t irq_id, uint8_t priority);

// Read interrupt acknowledge register (returns interrupt ID)
uint32_t gic_read_iar(void);

// End of interrupt - signal completion
void gic_write_eoir(uint32_t irq_id);

#endif /* _gic_h_ */