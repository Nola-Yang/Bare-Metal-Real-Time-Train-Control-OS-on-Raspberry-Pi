#ifndef _mcp2515_h_
#define _mcp2515_h_ 1

#include <stdint.h>

typedef struct {
	uint32_t id;        // CAN identifier (11-bit or 29-bit)
	uint8_t  dlc;
	uint8_t  data[8];
	uint8_t  ext;
} can_frame_t;

// CANINTF flags
#define MCP2515_CANINTF_RX0IF 0x01
#define MCP2515_CANINTF_RX1IF 0x02
#define MCP2515_CANINTF_TX0IF 0x04

/** Initialize the MCP2515 CAN controller. Should be called after initializing GPIO and SPI. */
void mcp2515_init(void);

/** Non-blocking CAN frame reception. Returns 1 if frame received, 0 if no frame available. */
int can_try_recv(can_frame_t *frame);

/** Send a CAN frame directly. Returns 1 on success, 0 on failure (TX buffer full). */
int can_send(const can_frame_t *frame);

/** Queue a CAN frame for transmission. Returns 1 on success, 0 if queue is full. */
int can_queue_frame(const can_frame_t *frame);

/** Service the TX queue. */
void can_queue_send(void);

/** Clear MCP2515 interrupt flags. */
void mcp2515_clear_interrupts(void);

/** Enable MCP2515 RX interrupts (RX0/RX1). */
void mcp2515_enable_rx_interrupts(void);

/** Disable MCP2515 RX interrupts (RX0/RX1). */
void mcp2515_disable_rx_interrupts(void);

/** Enable MCP2515 TX interrupts (TX0). */
void mcp2515_enable_tx_interrupts(void);

/** Disable MCP2515 TX interrupts (TX0). */
void mcp2515_disable_tx_interrupts(void);

/** Read MCP2515 interrupt flags (CANINTF). */
uint8_t mcp2515_read_interrupt_flags(void);

/** Clear selected MCP2515 interrupt flags (CANINTF). */
void mcp2515_clear_interrupt_flags(uint8_t mask);

/** Disable all MCP2515 interrupts. */
void mcp2515_disable_interrupts(void);

/** Check if TX buffer is busy. Returns 1 if busy, 0 if ready. */
int can_tx_busy(void);

#endif /* _mcp2515_h_ */
