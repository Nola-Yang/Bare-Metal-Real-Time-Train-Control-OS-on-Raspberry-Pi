#include "terminal_server.h"
#include "syscall.h"
#include "nameserver.h"
#include "gic.h"
#include "uart.h"
#include "ring_buffer.h"
#include "task_scheduler.h"
#include <string.h>
#include "task_scheduler.h"

// only 1 in fact
#define MAX_GETC_WAITERS 8

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

// todo: maybe too large for kernel stack
#define TX_BUF_SIZE 2048
static char tx_buf_data[TX_BUF_SIZE];
static int tx_buf_head, tx_buf_tail, tx_buf_count;

static void tx_buf_init(void) { tx_buf_head = tx_buf_tail = tx_buf_count = 0; }
static int  tx_buf_is_empty(void) { return tx_buf_count == 0; }
static int  tx_buf_put(char c) {
    if (tx_buf_count >= TX_BUF_SIZE) return -1;
    tx_buf_data[tx_buf_head] = c;
    tx_buf_head = (tx_buf_head + 1) % TX_BUF_SIZE;
    tx_buf_count++;
    return 0;
}
static int tx_buf_get(void) {
    if (tx_buf_count == 0) return -1;
    char c = tx_buf_data[tx_buf_tail];
    tx_buf_tail = (tx_buf_tail + 1) % TX_BUF_SIZE;
    tx_buf_count--;
    return (unsigned char)c;
}

// Terminal Server
void terminal_server_task(void) {
    int tid;
    TermRequest_t req;
    TermReply_t reply;

    RingBuffer_t rx_buffer;
    ring_buffer_init(&rx_buffer);
    tx_buf_init();

    int getc_waiters[MAX_GETC_WAITERS];
    int getc_waiter_count = 0;

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
                    // Buffer the character
                    ring_buffer_put(&rx_buffer, req.ch);
                }
                break;

            case TERM_MSG_TX_NOTIFY:
                while (!tx_buf_is_empty() && uart_tx_ready(CONSOLE)) {
                    int c = tx_buf_get();
                    uart_putc_nonblocking(CONSOLE, (char)c);
                }

                // If buffer is now empty, keep notifier blocked until we have more data
                if (tx_buf_is_empty()) {
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
                    reply.ch = (char)ring_buffer_get(&rx_buffer);
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
                tx_buf_put(req.ch);

                // Try to send immediately
                while (!tx_buf_is_empty() && uart_tx_ready(CONSOLE)) {
                    int c = tx_buf_get();
                    uart_putc_nonblocking(CONSOLE, (char)c);
                }

                // If there's still data and notifier is blocked, wake it up, to wait for next TX interrupt
                if (tx_notifier_blocked_tid >= 0 && !tx_buf_is_empty()) {
                    TermReply_t tx_reply;
                    tx_reply.status = 0;
                    Reply(tx_notifier_blocked_tid, (const char *)&tx_reply, sizeof(tx_reply));
                    tx_notifier_blocked_tid = -1;
                }

                reply.status = 0;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TERM_MSG_PUTS:
                for (int i = 0; i < req.len && i < TERM_MAX_STR_LEN; i++) {
                    tx_buf_put(req.str[i]);
                }

                // Try to send immediately
                while (!tx_buf_is_empty() && uart_tx_ready(CONSOLE)) {
                    int c = tx_buf_get();
                    uart_putc_nonblocking(CONSOLE, (char)c);
                }

                //still data and notifier is blocked, wake it up
                if (tx_notifier_blocked_tid >= 0 && !tx_buf_is_empty()) {
                    TermReply_t tx_reply;
                    tx_reply.status = 0;
                    Reply(tx_notifier_blocked_tid, (const char *)&tx_reply, sizeof(tx_reply));
                    tx_notifier_blocked_tid = -1;
                }

                reply.status = req.len;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

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
