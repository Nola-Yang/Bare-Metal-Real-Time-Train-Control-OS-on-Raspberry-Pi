#include "mcp2515.h"
#include "spi.h"

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

//TX Queue Management
#define TX_QUEUE_SIZE 64  // Increased to handle burst commands (init=22 frames, plus margin)
static can_frame_t tx_queue[TX_QUEUE_SIZE];
static int tx_head = 0;  // Next slot to write
static int tx_tail = 0;  // Next slot to read

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
	// No need to reset MCP2515 here as a hardware reset is done during boot.
	// MCP2515 automatically enters config mode after hardware reset.

	// Set the bitrate configuration registers
	mcp2515_write_reg(CNF1, MCP_16MHz_250kbPS_CFG1);
	mcp2515_write_reg(CNF2, MCP_16MHz_250kbPS_CFG2);
	mcp2515_write_reg(CNF3, MCP_16MHz_250kbPS_CFG3);

	// do not filter messages. Allow rollover of RXB0 to RXB1.
	mcp2515_write_reg(RXBnCTRL0, 0x64);
	mcp2515_write_reg(RXBnCTRL1, 0x60);

	// start MCP2515 by setting operation mode to normal
	mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP, OPMODE_NORMAL);
	while ((mcp2515_read_reg(CANSTAT) & CANSTAT_OPMOD) != OPMODE_NORMAL); // wait until mode is set
}

int can_send(const can_frame_t *frame)
{
    uint8_t ctrl = mcp2515_read_reg(TXB0CTRL);
    if (ctrl & TXB_TXREQ) return 0;

    if (frame->ext) {
        uint32_t id = frame->id & 0x1FFFFFFF;

        uint8_t sidh = (id >> 21) & 0xFF;              // id[28:21]
        uint8_t sidl = (uint8_t)(((id >> 18) & 0x07) << 5); // id[20:18] -> SIDL[7:5]
        sidl |= (1u << 3);                         // EXIDE = 1
        sidl |= (uint8_t)((id >> 16) & 0x03);           // id[17:16] -> SIDL[1:0]

        uint8_t eid8 = (id >> 8) & 0xFF;           // id[15:8]
        uint8_t eid0 = id & 0xFF;                  // id[7:0]

        mcp2515_write_reg(TXB0SIDH, sidh);
        mcp2515_write_reg(TXB0SIDL, sidl);
        mcp2515_write_reg(TXB0EID8, eid8);
        mcp2515_write_reg(TXB0EID0, eid0);
    }

    uint8_t dlc = frame->dlc;
    mcp2515_write_reg(TXB0DLC, dlc);
    mcp2515_write_regs(TXB0D0, frame->data, dlc);

    mcp2515_modify_reg(TXB0CTRL, TXB_TXREQ, TXB_TXREQ);

    return 1;
}


int can_try_recv(can_frame_t *frame) {
	uint8_t status = mcp2515_read_status();

	// Determine which buffer has data
	uint8_t buf_base;
	uint8_t clear_flag;

	if (status & STATUS_RX0) {
		buf_base = RXB0SIDH;
		clear_flag = 0x01;
	} else if (status & STATUS_RX1) {
		buf_base = RXB1SIDH;
		clear_flag = 0x02;
	} else {
		return 0;
	}

	uint8_t rx_sidh = mcp2515_read_reg(buf_base);      // SIDH
	uint8_t rx_sidl = mcp2515_read_reg(buf_base + 1);  // SIDL

	// Check if extended frame (SIDL[3])
	if (rx_sidl & (1 << 3)) {
		uint8_t rx_eid8 = mcp2515_read_reg(buf_base + 2);  // EID8
		uint8_t rx_eid0 = mcp2515_read_reg(buf_base + 3);  // EID0

		// SIDH[7:0] = ID[28:21]
		// SIDL[7:5] = ID[20:18]
		// SIDL[1:0] = ID[17:16]
		// EID8[7:0] = ID[15:8]
		// EID0[7:0] = ID[7:0]
		frame->id = ((uint32_t)rx_sidh << 21) |
		            ((uint32_t)(rx_sidl >> 5) << 18) |
		            ((uint32_t)(rx_sidl & 0x03) << 16) |
		            ((uint32_t)rx_eid8 << 8) |
		            ((uint32_t)rx_eid0);
		frame->ext = 1;
	} else {
		// Standard frame, useless, keep for completeness
		frame->id = ((uint32_t)rx_sidh << 3) | ((rx_sidl >> 5) & 0x07);
		frame->ext = 0;
	}

	// Read DLC
	uint8_t rx_dlc = mcp2515_read_reg(buf_base + 4);
	frame->dlc = rx_dlc & 0x0F;

	if (frame->dlc > 0) {
		uint8_t data_base = buf_base + 5;
		uint8_t read_len = (frame->dlc > 8) ? 8 : frame->dlc;
		mcp2515_read_regs(data_base, frame->data, read_len);
	}

	// Clear interrupt flag to mark frame as read
	mcp2515_modify_reg(CANINTF, clear_flag, 0x00);

	return 1;
}

// TX Queue Functions

int can_queue_frame(const can_frame_t *frame) {
	int next_head = (tx_head + 1) % TX_QUEUE_SIZE;

	if (next_head == tx_tail) {
		return 0;
	}

	tx_queue[tx_head] = *frame;
	tx_head = next_head;

	return 1;
}

void can_queue_send(void) {
	if (tx_head == tx_tail) {
		return;
	}
	if (can_send(&tx_queue[tx_tail])) {
		tx_tail = (tx_tail + 1) % TX_QUEUE_SIZE;
	}
}

void mcp2515_clear_interrupts(void) {
	mcp2515_write_reg(CANINTF, 0x00);  // prevent interrupt storm
}

int can_tx_busy(void) {
	uint8_t ctrl = mcp2515_read_reg(TXB0CTRL);
	return (ctrl & TXB_TXREQ) ? 1 : 0;
}
