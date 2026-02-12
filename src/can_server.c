#include "can_server.h"
#include "syscall.h"
#include "nameserver.h"
#include "gic.h"
#include "uart.h"
#include "task_scheduler.h"
#include "spi.h"
#include "mcp2515.h"
#include "rpi.h"


//I guess only one for now implementation
#define MAX_RECV_WAITERS 8

// server-side, receives frames from MCP2515
#define RX_QUEUE_SIZE 16

typedef struct {
    can_frame_t frames[RX_QUEUE_SIZE];
    int head;
    int tail;
    int count;
} CANRxQueue_t;

static int can_rx_notifier_tid = -1;

static inline void rx_queue_init(CANRxQueue_t *q) {
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

static inline int rx_queue_is_empty(CANRxQueue_t *q) {
    return q->count == 0;
}

static inline int rx_queue_is_full(CANRxQueue_t *q) {
    return q->count >= RX_QUEUE_SIZE;
}

static inline int rx_queue_put(CANRxQueue_t *q, const can_frame_t *frame) {
    if (q->count >= RX_QUEUE_SIZE) return -1; //todo: may need some error handling for overflow
    q->frames[q->head] = *frame;
    q->head = (q->head + 1) % RX_QUEUE_SIZE;
    q->count++;
    return 0;
}

static inline int rx_queue_get(CANRxQueue_t *q, can_frame_t *frame) {
    if (q->count == 0) return -1;
    *frame = q->frames[q->tail];
    q->tail = (q->tail + 1) % RX_QUEUE_SIZE;
    q->count--;
    return 0;
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
    rx_queue_init(&rx_queue);

    int recv_waiters[MAX_RECV_WAITERS];
    int recv_waiter_count = 0;

    RegisterAs(CAN_SERVER_NAME);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        switch (req.type) {
            case CAN_MSG_RX_NOTIFY: {
                // Drain all available RX frames
                can_frame_t frame;
                while (can_try_recv(&frame)) {
                    if (recv_waiter_count > 0) {
                        CANReply_t recv_reply;
                        recv_reply.status = 0;
                        recv_reply.frame = frame;
                        Reply(recv_waiters[0], (const char *)&recv_reply, sizeof(recv_reply));

                        // in fact, only one waiter, maybe optimized later
                        for (int i = 1; i < recv_waiter_count; i++) {
                            recv_waiters[i-1] = recv_waiters[i];
                        }
                        recv_waiter_count--;
                    } else {
                        // Buffer the frame
                        rx_queue_put(&rx_queue, &frame);
                    }
                }

                //Todo: TX done interrupt instead of polling there
                can_queue_send();

                // Re-enable only when a client needs data (in CAN_MSG_RECV)
                mcp2515_disable_interrupts(); // prevent notifier spinning
                mcp2515_clear_interrupts();

                // never be triggered? may be deleted?
                if (recv_waiter_count > 0) {
                    mcp2515_enable_rx_interrupts();
                    gpio_enable_can_interrupt();
                }

                reply.status = 0;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case CAN_MSG_SEND: {
                int queued = can_queue_frame(&req.frame);

                can_queue_send();

                reply.status = queued ? 0 : -1;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case CAN_MSG_RECV: {
                can_frame_t frame;
                if (rx_queue_get(&rx_queue, &frame) == 0) {
                    reply.status = 0;
                    reply.frame = frame;
                    Reply(tid, (const char *)&reply, sizeof(reply));
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
