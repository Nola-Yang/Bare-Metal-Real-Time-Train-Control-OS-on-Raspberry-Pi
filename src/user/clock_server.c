#include "clock_server.h"
#include "syscall.h"
#include "nameserver.h"
#include "gic.h"
#include "terminal_server.h"
#include "task_scheduler.h"
#include "min_heap.h"
#include "uart.h"
#include <stdint.h>


#define MAX_DELAY_TASKS 64

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
static void insert_delay_entry(MinHeap_t *queue, int tid, int wake_tick) {
    if (min_heap_is_full(queue)) {
        // uart_debug_printf("Clock Server delay queue full! Cannot delay task %d\r\n", tid);
        return;
    }

    min_heap_insert(queue, wake_tick, &tid);
}

// Clock Server task
void clock_server_task(void) {
    int tid;
    ClockRequest_t req;
    ClockReply_t reply;

    // priority queue for delays
    uint32_t wake_ticks[MAX_DELAY_TASKS];
    uint32_t delay_task_ids[MAX_DELAY_TASKS];
    MinHeap_t delay_queue;
    init_min_heap(&delay_queue, wake_ticks, delay_task_ids, sizeof(uint32_t), MAX_DELAY_TASKS);

    int current_tick = 0;
    int earliest_wake_tick;
    uint32_t earliest_delay_task_id;
    uint32_t earliest_delay;

    RegisterAs(CLOCK_SERVER_NAME);
    Create(CLOCK_NOTIFIER_PRIORITY, clock_notifier_task);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;  

        switch (req.type) {
            case CLOCK_MSG_NOTIFY:
                current_tick++;

                reply.ticks = current_tick;
                Reply(tid, (const char *)&reply, sizeof(reply));

                // Wake up any tasks whose delay has expired
                while (!min_heap_is_empty(&delay_queue)) {
                    earliest_wake_tick = min_heap_get_top_key(&delay_queue);
                    if (earliest_wake_tick > current_tick) break;

                    min_heap_pop(&delay_queue, &earliest_delay, &earliest_delay_task_id);

                    reply.ticks = current_tick;
                    Reply(earliest_delay_task_id, (const char *)&reply, sizeof(reply));
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
                    insert_delay_entry(&delay_queue, tid, wake_tick);
                }
                break;

            case CLOCK_MSG_DELAY_UNTIL:
                if (req.ticks <= current_tick) {
                    reply.ticks = current_tick;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                } else {
                    insert_delay_entry(&delay_queue, tid, req.ticks);
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