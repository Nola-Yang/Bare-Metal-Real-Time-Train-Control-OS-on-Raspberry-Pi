#include "terminal_server.h"
#include "syscall.h"
#include "nameserver.h"
#include "gic.h"
#include "uart.h"
#include "ring_buffer.h"
#include "task_scheduler.h"
#include "kassert.h"
#include <string.h>

// only 1 in fact
#define MAX_GETC_WAITERS 8
#define MAX_PUTC_WAITERS 8

// maybe too large for kernel stack
#define TX_BUF_SIZE 2048
RING_BUFFER_DECLARE(TxBuffer_t, char, TX_BUF_SIZE);
static TxBuffer_t tx_buf;

// A blocked Puts/Putc caller waiting for TX buffer space
typedef struct {
    int tid;
    char data[TERM_MAX_STR_LEN];
    int len;    // total bytes to write
    int offset; // bytes already written to tx_buf
} PutcWaiter_t;

// Try to drain putc waiters into tx_buf. Returns number of waiters fully drained.
static int drain_putc_waiters(PutcWaiter_t *waiters, int count) {
    int drained = 0;
    for (int i = 0; i < count; i++) {
        while (waiters[i].offset < waiters[i].len) {
            if (ring_buffer_put(&tx_buf, waiters[i].data[waiters[i].offset]) < 0) {
                // tx_buf full, can't drain any more
                goto done;
            }
            waiters[i].offset++;
        }
        // This waiter is fully drained, reply to unblock it
        TermReply_t r;
        r.status = waiters[i].len;
        Reply(waiters[i].tid, (const char *)&r, sizeof(r));
        drained++;
    }
done:
    return drained;
}

//waits for UART RX interrupt, drains FIFO, notifies server
static void rx_notifier_task(void) {
    int server_tid = MyParentTid();
    TermRequest_t req;
    TermReply_t reply;

    req.type = TERM_MSG_RX_NOTIFY;

    for (;;) {
        // Drain the UART RX FIFO and send chars to server.
        while (uart_rx_ready(CONSOLE)) {
            int c = uart_getc_nonblocking(CONSOLE);
            if (c >= 0) {
                req.ch = (char)c;
                Send(server_tid, (const char *)&req, sizeof(req),
                     (char *)&reply, sizeof(reply));
            }
        }

        // No data available: enable RX interrupt and wait.
        uart_enable_rx_interrupt(CONSOLE);
        AwaitEvent(EVENT_UART_RX);
    }
}

// waits for UART TX interrupt, notifies server to send more
static void tx_notifier_task(void) {
    int server_tid = MyParentTid();
    TermRequest_t req;
    TermReply_t reply;

    req.type = TERM_MSG_TX_NOTIFY;

    for (;;) {
        if (!uart_tx_ready(CONSOLE)) {
            // TX not ready: enable interrupt and wait.
            uart_enable_tx_interrupt(CONSOLE);
            AwaitEvent(EVENT_UART_TX);
        }

        Send(server_tid, (const char *)&req, sizeof(req),
             (char *)&reply, sizeof(reply));
    }
}

// Terminal Server
void terminal_server_task(void) {
    int tid;
    TermRequest_t req;
    TermReply_t reply;

    RingBuffer_t rx_buffer;
    ring_buffer_init(&rx_buffer);
    ring_buffer_init(&tx_buf);

    int getc_waiters[MAX_GETC_WAITERS];
    int getc_waiter_count = 0;

    PutcWaiter_t putc_waiters[MAX_PUTC_WAITERS];
    int putc_waiter_count = 0;

    int tx_notifier_blocked_tid = -1;

    RegisterAs(TERMINAL_SERVER_NAME);
    Create(TERM_NOTIFIER_PRIORITY, rx_notifier_task);
    Create(TERM_NOTIFIER_PRIORITY, tx_notifier_task);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        switch (req.type) {
            case TERM_MSG_RX_NOTIFY:
                reply.status = 0;
                Reply(tid, (const char *)&reply, sizeof(reply));

                // If someone is waiting for Getc, give them the char
                if (getc_waiter_count > 0) {
                    TermReply_t getc_reply;
                    getc_reply.status = 0;
                    getc_reply.ch = req.ch;
                    Reply(getc_waiters[0], (const char *)&getc_reply, sizeof(getc_reply));

                    for (int i = 1; i < getc_waiter_count; i++) {
                        getc_waiters[i-1] = getc_waiters[i];
                    }
                    getc_waiter_count--;
                } else {
                    // Buffer the character, slightly drop if buffer is full
                    ring_buffer_put(&rx_buffer, req.ch);
                    // KASSERT(ring_buffer_put(&rx_buffer, req.ch) == 0); 
                }
                break;

            case TERM_MSG_TX_NOTIFY:
                // Drain tx_buf to UART hardware
                while (!ring_buffer_is_empty(&tx_buf) && uart_tx_ready(CONSOLE)) {
                    char c;
                    if (ring_buffer_get(&tx_buf, &c) < 0) {
                        break;
                    }
                    uart_putc_nonblocking(CONSOLE, c);
                }

                // Buffer has space now — try to flush blocked putc waiters
                if (putc_waiter_count > 0) {
                    int drained = drain_putc_waiters(putc_waiters, putc_waiter_count);
                    for (int i = drained; i < putc_waiter_count; i++) {
                        putc_waiters[i - drained] = putc_waiters[i];
                    }
                    putc_waiter_count -= drained;
                }

                // If buffer is now empty, keep notifier blocked until we have more data
                if (ring_buffer_is_empty(&tx_buf) && putc_waiter_count == 0) {
                    tx_notifier_blocked_tid = tid;
                } else {
                    // Still have data, reply to let notifier wait for next TX interrupt
                    reply.status = 0;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                    tx_notifier_blocked_tid = -1;
                }
                break;

            case TERM_MSG_GETC:
                if (!ring_buffer_is_empty(&rx_buffer)) {
                    reply.status = 0;
                    ring_buffer_get(&rx_buffer, &reply.ch);
                    Reply(tid, (const char *)&reply, sizeof(reply));
                } else {
                    // Queue the client
                    if (getc_waiter_count < MAX_GETC_WAITERS) {
                        getc_waiters[getc_waiter_count++] = tid;
                    } else {
                        reply.status = -1; 
                        reply.ch = 0;
                        Reply(tid, (const char *)&reply, sizeof(reply));
                    }
                }
                break;

            case TERM_MSG_PUTC:
                if (ring_buffer_put(&tx_buf, req.ch) < 0) {
                    // Buffer full — block caller until space available
                    if (putc_waiter_count < MAX_PUTC_WAITERS) {
                        putc_waiters[putc_waiter_count].tid = tid;
                        putc_waiters[putc_waiter_count].data[0] = req.ch;
                        putc_waiters[putc_waiter_count].len = 1;
                        putc_waiters[putc_waiter_count].offset = 0;
                        putc_waiter_count++;
                    } else {
                        reply.status = -1;
                        Reply(tid, (const char *)&reply, sizeof(reply));
                    }
                } else {
                    reply.status = 0;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                }

                // Try to send immediately
                while (!ring_buffer_is_empty(&tx_buf) && uart_tx_ready(CONSOLE)) {
                    char c;
                    if (ring_buffer_get(&tx_buf, &c) < 0) {
                        break;
                    }
                    uart_putc_nonblocking(CONSOLE, c);
                }

                // If there's pending data or blocked waiters, wake notifier
                if (tx_notifier_blocked_tid >= 0 &&
                    (!ring_buffer_is_empty(&tx_buf) || putc_waiter_count > 0)) {
                    TermReply_t tx_reply;
                    tx_reply.status = 0;
                    Reply(tx_notifier_blocked_tid, (const char *)&tx_reply, sizeof(tx_reply));
                    tx_notifier_blocked_tid = -1;
                }
                break;

            case TERM_MSG_PUTS: {
                int copy_len = (req.len < TERM_MAX_STR_LEN) ? req.len : TERM_MAX_STR_LEN;
                int written = 0;
                for (int i = 0; i < copy_len; i++) {
                    if (ring_buffer_put(&tx_buf, req.str[i]) < 0) {
                        break;
                    }
                    written++;
                }

                if (written < copy_len) {
                    // Buffer full before all data written — block caller
                    if (putc_waiter_count < MAX_PUTC_WAITERS) {
                        putc_waiters[putc_waiter_count].tid = tid;
                        memcpy(putc_waiters[putc_waiter_count].data, req.str, copy_len);
                        putc_waiters[putc_waiter_count].len = copy_len;
                        putc_waiters[putc_waiter_count].offset = written;
                        putc_waiter_count++;
                    } else {
                        reply.status = -1;
                        Reply(tid, (const char *)&reply, sizeof(reply));
                    }
                } else {
                    reply.status = copy_len;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                }

                // Try to send immediately
                while (!ring_buffer_is_empty(&tx_buf) && uart_tx_ready(CONSOLE)) {
                    char c;
                    if (ring_buffer_get(&tx_buf, &c) < 0) {
                        break;
                    }
                    uart_putc_nonblocking(CONSOLE, c);
                }

                // If there's pending data or blocked waiters, wake notifier
                if (tx_notifier_blocked_tid >= 0 &&
                    (!ring_buffer_is_empty(&tx_buf) || putc_waiter_count > 0)) {
                    TermReply_t tx_reply;
                    tx_reply.status = 0;
                    Reply(tx_notifier_blocked_tid, (const char *)&tx_reply, sizeof(tx_reply));
                    tx_notifier_blocked_tid = -1;
                }
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

int Getc(int tid, int channel) {
    TermRequest_t req;
    TermReply_t reply;

    if (channel != TERM_CHANNEL_CONSOLE) {
        return -1;
    }

    req.type = TERM_MSG_GETC;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;
    }
    if (reply.status < 0) {
        return reply.status;
    }
    return (unsigned char)reply.ch;
}

int Putc(int tid, int channel, char ch) {
    TermRequest_t req;
    TermReply_t reply;

    if (channel != TERM_CHANNEL_CONSOLE) {
        return -1;
    }

    req.type = TERM_MSG_PUTC;
    req.ch = ch;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;
    }
    return reply.status;
}

int Puts(int tid, int channel, const char *str, int len) {
    TermRequest_t req;
    TermReply_t reply;

    if (channel != TERM_CHANNEL_CONSOLE) {
        return -1;
    }

    req.type = TERM_MSG_PUTS;

    // Copy string data into message (safe even if str is on stack)
    int copy_len = (len < TERM_MAX_STR_LEN) ? len : TERM_MAX_STR_LEN;
    for (int i = 0; i < copy_len; i++) {
        req.str[i] = str[i];
    }
    req.len = copy_len;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;
    }
    return reply.status;
}
