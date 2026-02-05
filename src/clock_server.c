#include "clock_server.h"
#include "syscall.h"
#include "nameserver.h"
#include "gic.h"
#include "uart.h"
#include "task_scheduler.h"
#include <stdint.h>


#define MAX_DELAY_TASKS 64

// Delay queue entry
typedef struct {
    int tid;
    int wake_tick;  // Tick at which to wake up
} DelayEntry_t;

// waits for timer events and notifies the server
static void clock_notifier_task(void) {
    int server_tid = MyParentTid();
    ClockRequest_t req;
    ClockReply_t reply;

    req.type = CLOCK_MSG_NOTIFY;
    req.ticks = 0;

    for (;;) {
        AwaitEvent(EVENT_TIMER_C1);

        Send(server_tid, (const char *)&req, sizeof(req),
             (char *)&reply, sizeof(reply));
    }
}

// Insert into delay queue maintaining sorted order 
static void insert_delay_entry(DelayEntry_t *queue, int *count, int tid, int wake_tick) {
    if (*count >= MAX_DELAY_TASKS) {
        uart_printf(CONSOLE, "ClockServer: Delay queue full!\r\n");
        return;
    }

    // sorted by wake_tick, ascending
    int i = *count;
    while (i > 0 && queue[i - 1].wake_tick > wake_tick) {
        queue[i] = queue[i - 1];
        i--;
    }
    queue[i].tid = tid;
    queue[i].wake_tick = wake_tick;
    (*count)++;
}

// Clock Server task
void clock_server_task(void) {
    int tid;
    ClockRequest_t req;
    ClockReply_t reply;

    // a priority queues for the delays
    DelayEntry_t delay_queue[MAX_DELAY_TASKS];
    int delay_count = 0;
    int current_tick = 0;

    RegisterAs(CLOCK_SERVER_NAME);
    Create(CLOCK_NOTIFIER_PRIORITY, clock_notifier_task);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;  

        switch (req.type) {
            case CLOCK_MSG_NOTIFY:
                current_tick++;

                // so it can wait for next tick
                reply.ticks = current_tick;
                Reply(tid, (const char *)&reply, sizeof(reply));

                // Wake up any tasks whose delay has expired
                while (delay_count > 0 && delay_queue[0].wake_tick <= current_tick) {
                    reply.ticks = current_tick;
                    Reply(delay_queue[0].tid, (const char *)&reply, sizeof(reply));

                    // Remove from queue (shift remaining entries)
                    for (int i = 0; i < delay_count - 1; i++) {
                        delay_queue[i] = delay_queue[i + 1];
                    }
                    delay_count--;
                }
                break;

            case CLOCK_MSG_TIME:
                reply.ticks = current_tick;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case CLOCK_MSG_DELAY:
                if (req.ticks <= 0) {
                    reply.ticks = current_tick;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                } else {
                    int wake_tick = current_tick + req.ticks;
                    insert_delay_entry(delay_queue, &delay_count, tid, wake_tick);
                }
                break;

            case CLOCK_MSG_DELAY_UNTIL:
                if (req.ticks <= current_tick) {
                    reply.ticks = current_tick;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                } else {
                    insert_delay_entry(delay_queue, &delay_count, tid, req.ticks);
                }
                break;

            default:
                reply.ticks = -1;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}

// User API implementations

int Time(int tid) {
    ClockRequest_t req;
    ClockReply_t reply;

    req.type = CLOCK_MSG_TIME;
    req.ticks = 0;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1; 
    }
    return reply.ticks;
}

int Delay(int tid, int ticks) {
    if (ticks < 0) {
        return -2;
    }

    ClockRequest_t req;
    ClockReply_t reply;

    req.type = CLOCK_MSG_DELAY;
    req.ticks = ticks;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;  
    }
    return reply.ticks;
}

int DelayUntil(int tid, int ticks) {
    if (ticks < 0) {
        return -2;
    }

    ClockRequest_t req;
    ClockReply_t reply;

    req.type = CLOCK_MSG_DELAY_UNTIL;
    req.ticks = ticks;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;  
    }
    return reply.ticks;
}