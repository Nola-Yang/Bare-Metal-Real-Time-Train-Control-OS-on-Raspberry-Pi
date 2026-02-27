#ifndef _can_server_h_
#define _can_server_h_ 1

#include "mcp2515.h"

#define CAN_SERVER_NAME "CANServer"

// Message types
#define CAN_MSG_SEND       0   // Send CAN frame (queued)
#define CAN_MSG_RECV       1   // Receive CAN frame (blocks until available)
#define CAN_MSG_RX_NOTIFY  2   // From notifier: interrupt occurred
#define CAN_MSG_ENABLE_INT 3   // Enable MCP2515/GPIO interrupt handling
#define CAN_MSG_WAIT_IDLE        4   // Wait until TX queue is fully drained
#define CAN_MSG_CANCEL_IDLE_WAIT 5   // Cancel pending WAIT_IDLE (timeout)

typedef struct {
    int type;
    can_frame_t frame;
} CANRequest_t;

typedef struct {
    int status;
    can_frame_t frame;
} CANReply_t;

// CAN server task 
void can_server_task(void);

// User API

// Send a CAN frame 
// Returns: 0 on success (queued), negative on error
int CANSend(int tid, const can_frame_t *frame);

// Receive a CAN frame 
// Returns: 0 on success, negative on error
int CANReceive(int tid, can_frame_t *frame);

// Enable MCP2515/GPIO interrupt handling
int CANEnableInterrupts(int tid);

// Block until queued CAN sends have all completed (and replies received)
int CANWaitTxIdle(int tid);

// Like CANWaitTxIdle but gives up after timeout_ticks.
// Creates a courier task internally; caller must not hold any Send-locks.
// Returns 0 on success, -1 on timeout.
int CANFlushTxTimeout(int can_tid, int clock_tid, int timeout_ticks);

#endif /* _can_server_h_ */
