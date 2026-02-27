#include "server/can_server.h"
#include "syscall.h"
#include "server/nameserver.h"
#include "gic.h"
#include "uart.h"
#include "task_scheduler.h"
#include "spi.h"
#include "mcp2515.h"
#include "rpi.h"
#include "ring_buffer.h"
#include "kassert.h"


//I guess only one for now implementation
#define MAX_RECV_WAITERS 8

// server-side, receives frames from MCP2515
#define RX_QUEUE_SIZE 16
#define TX_QUEUE_SIZE 64

RING_BUFFER_DECLARE(CANRxQueue_t, can_frame_t, RX_QUEUE_SIZE);
RING_BUFFER_DECLARE(CANTxQueue_t, can_frame_t, TX_QUEUE_SIZE);

typedef struct {
    int active;
    uint8_t command;
    uint8_t key_len;
    uint8_t key[4];
} PendingReply_t;

static int can_rx_notifier_tid = -1;

static void set_pending_reply(PendingReply_t *pending, const can_frame_t *tx) {
    pending->active = 1;
    pending->command = (uint8_t)((tx->id >> 17) & 0xFF);
    pending->key_len = (tx->dlc >= 4) ? 4 : tx->dlc;
    for (uint8_t i = 0; i < pending->key_len; i++) {
        pending->key[i] = tx->data[i];
    }
}

static int is_pending_reply_match(const PendingReply_t *pending,
                                  const can_frame_t *rx) {
    if (!pending->active) return 0;
    if (!rx->ext) return 0;

    uint8_t rx_command = (uint8_t)((rx->id >> 17) & 0xFF);
    if (rx_command != pending->command) return 0;
    if (rx->dlc < pending->key_len) return 0;

    for (uint8_t i = 0; i < pending->key_len; i++) {
        if (rx->data[i] != pending->key[i]) return 0;
    }

    return 1;
}

static void reply_waiter_with_frame(int waiters[], int *waiter_count,
                                    const can_frame_t *frame) {
    if (*waiter_count <= 0) return;

    CANReply_t recv_reply;
    recv_reply.status = 0;
    recv_reply.frame = *frame;
    Reply(waiters[0], (const char *)&recv_reply, sizeof(recv_reply));

    for (int i = 1; i < *waiter_count; i++) {
        waiters[i - 1] = waiters[i];
    }
    (*waiter_count)--;
}

static void dispatch_rx_frame(CANRxQueue_t *rx_queue,
                              int waiters[], int *waiter_count,
                              const can_frame_t *frame) {
    if (*waiter_count > 0) {
        reply_waiter_with_frame(waiters, waiter_count, frame);
    } else {
        KASSERT(ring_buffer_put(rx_queue, *frame) == 0);
    }
}

static void try_send_next_queued(CANTxQueue_t *tx_queue, int *waiting_reply,
                                 PendingReply_t *pending_reply) {
    if (*waiting_reply) return;

    can_frame_t frame;
    if (ring_buffer_peek(tx_queue, &frame) < 0) return;
    if (!can_send(&frame)) return;

    ring_buffer_get(tx_queue, &frame);
    *waiting_reply = 1;
    set_pending_reply(pending_reply, &frame);
}

// CAN RX Notifier
static void can_rx_notifier_task(void) {
    int server_tid = MyParentTid();
    CANRequest_t req;
    CANReply_t reply;

    req.type = CAN_MSG_RX_NOTIFY;

    for (;;) {
        AwaitEvent(EVENT_CAN_RX);

        Send(server_tid, (const char *)&req, sizeof(req),
             (char *)&reply, sizeof(reply));
    }
}
 
void can_server_task(void) {
    int tid;
    CANRequest_t req;
    CANReply_t reply;

    CANRxQueue_t rx_queue;
    CANTxQueue_t tx_queue;
    ring_buffer_init(&rx_queue);
    ring_buffer_init(&tx_queue);

    int recv_waiters[MAX_RECV_WAITERS];
    int recv_waiter_count = 0;
    int tx_waiting_reply = 0;
    PendingReply_t pending_reply = {0};

    RegisterAs(CAN_SERVER_NAME);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        switch (req.type) {
            case CAN_MSG_RX_NOTIFY: {
                uint8_t flags = mcp2515_read_interrupt_flags();

                // Drain all available RX frames if RX interrupt fired.
                if (flags & (MCP2515_CANINTF_RX0IF | MCP2515_CANINTF_RX1IF)) {
                    can_frame_t frame;
                    while (can_try_recv(&frame)) {
                        if (tx_waiting_reply &&
                            is_pending_reply_match(&pending_reply, &frame)) {
                            tx_waiting_reply = 0;
                            pending_reply.active = 0;
                            try_send_next_queued(&tx_queue, &tx_waiting_reply,
                                                 &pending_reply);
                            continue;
                        }
                        dispatch_rx_frame(&rx_queue, recv_waiters,
                                          &recv_waiter_count, &frame);
                    }
                }

                if (flags & MCP2515_CANINTF_TX0IF) {
                    mcp2515_clear_interrupt_flags(MCP2515_CANINTF_TX0IF);
                    try_send_next_queued(&tx_queue, &tx_waiting_reply,
                                         &pending_reply);
                }

                // Keep RX interrupts enabled whenever there are waiters or RX queue has space.
                if (recv_waiter_count > 0 || !ring_buffer_is_full(&rx_queue)) {
                    mcp2515_enable_rx_interrupts();
                } else {
                    mcp2515_disable_rx_interrupts();
                }

                gpio_enable_can_interrupt();

                reply.status = 0;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case CAN_MSG_SEND: {
                int queued = (ring_buffer_put(&tx_queue, req.frame) == 0);
                if (queued) {
                    try_send_next_queued(&tx_queue, &tx_waiting_reply,
                                         &pending_reply);
                    mcp2515_enable_tx_interrupts();
                    gpio_enable_can_interrupt();
                }

                reply.status = queued ? 0 : -1;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case CAN_MSG_RECV: {
                can_frame_t frame;
                if (ring_buffer_get(&rx_queue, &frame) == 0) {
                    reply.status = 0;
                    reply.frame = frame;
                    Reply(tid, (const char *)&reply, sizeof(reply));

                    // If RX interrupts were disabled due to full queue, re-enable now.
                    if (can_rx_notifier_tid >= 0) {
                        mcp2515_enable_rx_interrupts();
                        gpio_enable_can_interrupt();
                    }
                } else {
                    if (can_try_recv(&frame)) {
                        reply.status = 0;
                        reply.frame = frame;
                        Reply(tid, (const char *)&reply, sizeof(reply));
                    } else {
                        // Enable RX interrupts and GPIO detect, then queue the client
                        mcp2515_enable_rx_interrupts();
                        gpio_enable_can_interrupt();
                        if (recv_waiter_count < MAX_RECV_WAITERS) {
                            recv_waiters[recv_waiter_count++] = tid;
                        } else {
                            // unreachable, so no error handler
                            reply.status = -1; 
                            Reply(tid, (const char *)&reply, sizeof(reply));
                        }
                    }
                }
                break;
            }

            case CAN_MSG_ENABLE_INT: {
                if (can_rx_notifier_tid < 0) {
                    can_rx_notifier_tid = Create(CAN_NOTIFIER_PRIORITY, can_rx_notifier_task);
                }
                
                mcp2515_clear_interrupts();
                mcp2515_enable_rx_interrupts();
                mcp2515_enable_tx_interrupts();
                gpio_enable_can_interrupt();
                reply.status = 0;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            default:
                reply.status = -1;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}

// User API implementations

int CANSend(int tid, const can_frame_t *frame) {
    CANRequest_t req;
    CANReply_t reply;

    req.type = CAN_MSG_SEND;
    req.frame = *frame;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;
    }
    return reply.status;
}

int CANReceive(int tid, can_frame_t *frame) {
    CANRequest_t req;
    CANReply_t reply;

    req.type = CAN_MSG_RECV;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;
    }
    if (reply.status == 0) {
        *frame = reply.frame;
    }
    return reply.status;
}

int CANEnableInterrupts(int tid) {
    CANRequest_t req;
    CANReply_t reply;

    req.type = CAN_MSG_ENABLE_INT;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;
    }
    return reply.status;
}
