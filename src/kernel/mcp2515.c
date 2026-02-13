#include "mcp2515.h"
#include "spi.h"
#include "ring_buffer.h"
#include "can_data.h"

// configuration registers
static const uint8_t CNF3 = 0x28;
static const uint8_t CNF2 = 0x29;
static const uint8_t CNF1 = 0x2A;

// MCP2515 configuration for 16 MHz clock and 250 kbit/s bitrate
// Chapter 5.0 Bit Timing in MCP2515 datasheet
// and/or https://kvaser.com/support/calculators/bit-timing-calculator/.
static const uint8_t MCP_16MHz_250kbPS_CFG1 = 0x41;
static const uint8_t MCP_16MHz_250kbPS_CFG2 = 0xF1;
static const uint8_t MCP_16MHz_250kbPS_CFG3 = 0x85;

// MCP2515 normal operation mode
static const uint8_t OPMODE_NORMAL = 0x00;

// MCP2515 instruction set
static const uint8_t INSTRUCTION_WRITE       = 0x02;
static const uint8_t INSTRUCTION_READ        = 0x03;
static const uint8_t INSTRUCTION_BIT_MODIFY  = 0x05;
static const uint8_t INSTRUCTION_READ_STATUS = 0xA0;

// MCP2515 status mask
static const uint8_t STATUS_RX0 = 0x01;
static const uint8_t STATUS_RX1 = 0x02;

// TX Buffer 0 registers
static const uint8_t TXB0CTRL = 0x30;
static const uint8_t TXB0SIDH = 0x31;  // Standard ID High
static const uint8_t TXB0SIDL = 0x32;  // Standard ID Low
static const uint8_t TXB0DLC  = 0x35;  // Data Length Code
static const uint8_t TXB0D0   = 0x36;  // Data byte 0
static const uint8_t TXB0EID8 = 0x33;  // Extended Identifier High (Bits 15-8)
static const uint8_t TXB0EID0 = 0x34;  // Extended Identifier Low (Bits 7-0)

// RX Buffer 0 registers
static const uint8_t RXB0SIDH = 0x61;
static const uint8_t RXB0DLC = 0x65;
static const uint8_t RXB0D0 = 0x66;

// RX Buffer 1 registers
static const uint8_t RXB1SIDH = 0x71;

// Control register flags
static const uint8_t TXB_TXREQ = 0x08;  // Message Transmit Request bit

// MCP2515 buffer registers
static const uint8_t RXBnCTRL0 = 0x60;
static const uint8_t RXBnCTRL1 = 0x70;

// control and status registers
static const uint8_t CANSTAT = 0x0E;
static const uint8_t CANCTRL = 0x0F;
// OPMOD mask for CANSTAT register
static const uint8_t CANSTAT_OPMOD = 0xE0;
// REQOP mask for CANCTRL register
static const uint8_t CANCTRL_REQOP = 0xE0;

// flags register
static const uint8_t CANINTF = 0x2C;
// interrupt enable register
static const uint8_t CANINTE = 0x2B;

// CANINTE bits
static const uint8_t CANINTE_RX0IE = 0x01;
static const uint8_t CANINTE_RX1IE = 0x02;

//TX Queue Management
#define TX_QUEUE_SIZE 64 
RING_BUFFER_DECLARE(CANTxQueue_t, CanData_t, TX_QUEUE_SIZE);
static CANTxQueue_t tx_queue;

/** Read n consecutive registers starting from the specified one. */
static void mcp2515_read_regs(uint8_t reg, uint8_t values[], const uint8_t n) {
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_READ);
	spi_transfer_one(reg);
	for (uint8_t i = 0; i < n; i++) {
		// during transaction, address pointer is automatically incremented after each byte transfer.
		values[i] = spi_transfer_one(0x00);
	}
	spi_end_transaction();
}

/** Read the value of a single register. */
static uint8_t mcp2515_read_reg(uint8_t reg) {
	uint8_t ret = 0;
	mcp2515_read_regs(reg, &ret, 1);
	return ret;
}

/** Write values to n consecutive registers starting from the specified one. */
static void mcp2515_write_regs(uint8_t reg, const uint8_t values[], const uint8_t n) {
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_WRITE);
	spi_transfer_one(reg);
	for (uint8_t i = 0; i < n; i++) {
		spi_transfer_one(values[i]);
	}
	spi_end_transaction();
}

/** Write a value to a single register. */
static void mcp2515_write_reg(uint8_t reg, const uint8_t value) {
	mcp2515_write_regs(reg, &value, 1);
}

/** Modify individual bits of a register according to mask. */
static void mcp2515_modify_reg(uint8_t reg, const uint8_t mask, const uint8_t data) {
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_BIT_MODIFY);
	spi_transfer_one(reg);
	spi_transfer_one(mask);
	spi_transfer_one(data);
	spi_end_transaction();
}

/** Read the status of the MCP2515, including RX and TX buffers. */
static uint8_t mcp2515_read_status(void) {
	uint8_t ret = 0;
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_READ_STATUS);
	ret = spi_transfer_one(0x00);
	spi_end_transaction();
	return ret;
}

void mcp2515_init(void) {
	ring_buffer_init(&tx_queue);

	// No need to reset MCP2515 here as a hardware reset is done during boot.
	// MCP2515 automatically enters config mode after hardware reset.

	// Set the bitrate configuration registers
	mcp2515_write_reg(CNF1, MCP_16MHz_250kbPS_CFG1);
	mcp2515_write_reg(CNF2, MCP_16MHz_250kbPS_CFG2);
	mcp2515_write_reg(CNF3, MCP_16MHz_250kbPS_CFG3);

	// do not filter messages. Allow rollover of RXB0 to RXB1.
	mcp2515_write_reg(RXBnCTRL0, 0x64);
	mcp2515_write_reg(RXBnCTRL1, 0x60);

	// Enable RX interrupts 
	mcp2515_write_reg(CANINTE, CANINTE_RX0IE | CANINTE_RX1IE);
	mcp2515_write_reg(CANINTF, 0x00);  // clear any pending flags

	// start MCP2515 by setting operation mode to normal
	mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP, OPMODE_NORMAL);
	while ((mcp2515_read_reg(CANSTAT) & CANSTAT_OPMOD) != OPMODE_NORMAL); // wait until mode is set
}

// mcp2515_put_can_tx_data(can_data): Writes a CAN data into the appropriate registers in the MCP2515 controller
static void mcp2515_put_can_tx_data(CanData_t *can_data) {
	mcp2515_write_regs(TXB0SIDH, can_data->id, CAN_ID_BYTE_LEN);
	mcp2515_write_reg(TXB0DLC, can_data->length);
	mcp2515_write_regs(TXB0D0, can_data->data, can_data->length);
}

// send_can_data(can_data): Action for sending out a CAN data
static void send_can_data(CanData_t *can_data) {
	mcp2515_put_can_tx_data(can_data);
	mcp2515_modify_reg(TXB0CTRL, TXB_TXREQ, TXB_TXREQ);
}

int can_send(const CanData_t *frame)
{
    uint8_t ctrl = mcp2515_read_reg(TXB0CTRL);
    if (ctrl & TXB_TXREQ) return 0;

    send_can_data(frame);
    return 1;
}

// mcp2515_get_can_rx_data(can_data, offset): Reads a CAN data from the appropriate registers
// 	from the MCP2515 controller
static void mcp2515_get_can_rx_data(CanData_t *can_data, uint8_t offset) {
	mcp2515_read_regs(RXB0SIDH + offset, can_data->id, CAN_ID_BYTE_LEN);

	uint8_t length = mcp2515_read_reg(RXB0DLC + offset);
	can_data->length = (length > CAN_DATA_MAX_BYTE_LEN) ? CAN_DATA_MAX_BYTE_LEN : length;

	mcp2515_read_regs(RXB0D0 + offset, can_data->data, can_data->length);
}

int can_try_recv(CanData_t *frame) {
	uint8_t status = mcp2515_read_status();

	// Determine which buffer has data
	uint8_t offset = 0;
	uint8_t clear_flag;

	if (status & STATUS_RX0) {
		clear_flag = 0x01;
	} else if (status & STATUS_RX1) {
		offset = 0x10;
		clear_flag = 0x02;
	} else {
		return 0;
	}

	mcp2515_get_can_rx_data(frame, offset);

	// Clear interrupt flag to mark frame as read
	mcp2515_modify_reg(CANINTF, clear_flag, 0x00);

	return 1;
}

// TX Queue Functions

int can_queue_frame(const CanData_t *frame) {
	return (ring_buffer_put(&tx_queue, *frame) == 0) ? 1 : 0;
}

void can_queue_send(void) {
	if (ring_buffer_is_empty(&tx_queue)) {
		return;
	}
	CanData_t frame;
	if (ring_buffer_peek(&tx_queue, &frame) < 0) {
		return;
	}
	if (can_send(&frame)) {
		ring_buffer_get(&tx_queue, &frame);
	}
}

void mcp2515_clear_interrupts(void) {
	mcp2515_write_reg(CANINTF, 0x00);
}

void mcp2515_enable_rx_interrupts(void) {
	mcp2515_write_reg(CANINTE, CANINTE_RX0IE | CANINTE_RX1IE);
}

void mcp2515_disable_interrupts(void) {
	mcp2515_write_reg(CANINTE, 0x00);
}

int can_tx_busy(void) {
	uint8_t ctrl = mcp2515_read_reg(TXB0CTRL);
	return (ctrl & TXB_TXREQ) ? 1 : 0;
}
