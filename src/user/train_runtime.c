#include "train_runtime.h"
#include "train_control.h"
#include "syscall.h"
#include "server/nameserver.h"
#include "server/clock_server.h"
#include "server/can_server.h"
#include "track.h"
#include "train_tracking/position.h"
#include "train_tracking/traffic_manager.h"
#include "ui.h"
#include "command.h"
#include "timer.h"
#include "ring_buffer.h"
#include "kassert.h"
#include "demo_manager.h"

#define RV_QUEUE_MAX 8
RING_BUFFER_DECLARE(RVQueue_t, int, RV_QUEUE_MAX);
static RVQueue_t rv_queue;

static int train_nums[MAX_ACTIVE_TRAINS] = {13, 14, 15, 17, 18};

static void can_rx_courier_task(void) {
    int parent = MyParentTid();
    int can_tid = WhoIs(CAN_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    KASSERT(can_tid >= 0);
    msg.type = TRAIN_MSG_CAN_FRAME;

    for (;;) {
        if (CANReceive(can_tid, &msg.frame, &msg.arrival_us) == 0) {
            Send(parent, (const char *)&msg, sizeof(msg),
                 (char *)&reply, sizeof(reply));
        }
    }
}

void rv_delay_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    KASSERT(clock_tid >= 0);

    msg.type = TRAIN_MSG_RV_REQUEST;
    Send(parent, (const char *)&msg, sizeof(msg),
         (char *)&reply, sizeof(reply));

    Delay(clock_tid, reply.delay_ticks);

    msg.type = TRAIN_MSG_RV_COMPLETE;
    msg.train = reply.train;
    Send(parent, (const char *)&msg, sizeof(msg),
         (char *)&reply, sizeof(reply));

    Exit();
}

static void demo_tick_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    KASSERT(clock_tid >= 0);
    msg.type = TRAIN_MSG_DEMO_TICK;

    for (;;) {
        Delay(clock_tid, 100);
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

static void pos_tick_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    KASSERT(clock_tid >= 0);
    msg.type = TRAIN_POS_TICK;

    for (;;) {
        Delay(clock_tid, 10);
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

static void pos_replan_tick_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    KASSERT(clock_tid >= 0);
    msg.type = TRAIN_POS_REPLAN_TICK;

    for (;;) {
        Delay(clock_tid, 100);
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

static void pos_switch_settle_tick_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    KASSERT(clock_tid >= 0);
    msg.type = TRAIN_POS_SWITCH_SETTLE_TICK;

    for (;;) {
        Delay(clock_tid, 1);
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

static void process_can_frame(const can_frame_t *frame, uint64_t now) {
    uint8_t command = (uint8_t)((frame->id >> 17) & 0xFF);
    int is_response = (int)((frame->id >> 16) & 1);

    if (command == 0x11 && frame->dlc >= 5) {
        uint16_t sensor_id = ((uint16_t)frame->data[2] << 8) | frame->data[3];
        uint8_t state = frame->data[4];

        if (sensor_id > 0) {
            track_log_sensor(sensor_id, now, state);
            ui_mark_sensors_dirty();
            if (state == 1) {
                pos_on_sensor_trigger(sensor_id, now);
            }
        }
    } else if (command == 0x0B && is_response && frame->dlc >= 5) {
        uint32_t sw_id_raw = ((uint32_t)frame->data[0] << 24) |
                             ((uint32_t)frame->data[1] << 16) |
                             ((uint32_t)frame->data[2] << 8)  |
                              (uint32_t)frame->data[3];
        int sw_num = (int)(sw_id_raw - 0x3000 + 1);
        char dir = (frame->data[4] == 0x01) ? 'S' : 'C';
        if (track_is_valid_switch(sw_num)) {
            track_update_switch(sw_num, dir);
            pos_mark_routes_dirty();
            ui_mark_switches_dirty();
        }
    }
}

static void init_trains(void) {
    for (int i = 0; i < MAX_ACTIVE_TRAINS; ++i) {
        track_set_speed(train_nums[i], 0);
        track_set_light(train_nums[i], 1);
    }
}

static void init_runtime_state(int can_tid) {
    track_init(can_tid);
    pos_init();
    demo_init();

    ring_buffer_init(&rv_queue);

    for (int sw = 1; sw <= 18; sw++) {
        track_set_switch(sw, 'S');
    }
    for (int sw = 153; sw <= 156; sw++) {
        char state = (sw == 153 || sw == 155) ? 'C' : 'S';
        track_set_switch(sw, state);
    }

    ui_mark_switches_dirty();
    init_trains();
}

void train_runtime_task(void) {
    int parent = MyParentTid();
    int tid;
    int can_tid = WhoIs(CAN_SERVER_NAME);
    int rv_pending_count = 0;
    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    KASSERT(can_tid >= 0);

    init_runtime_state(can_tid);

    msg.type = TRAIN_MSG_RUNTIME_READY;
    Send(parent, (const char *)&msg, sizeof(msg),
         (char *)&reply, sizeof(reply));

    CANEnableInterrupts(can_tid);

    Create(TRAIN_COURIER_PRIORITY, can_rx_courier_task);
    Create(TRAIN_COURIER_PRIORITY, demo_tick_task);
    Create(TRAIN_CONTROL_PRIORITY, pos_tick_task);
    Create(TRAIN_CONTROL_PRIORITY, pos_replan_tick_task);
    Create(TRAIN_CONTROL_PRIORITY, pos_switch_settle_tick_task);

    for (;;) {
        int msglen = Receive(&tid, (char *)&msg, sizeof(msg));
        (void)msglen;

        reply.status = 0;

        switch (msg.type) {
            case TRAIN_MSG_COMMAND: {
                int rv_train = -1;
                int result = execute_it(msg.cmdline, &rv_train, rv_pending_count > 0);
                reply.status = result;
                if (rv_train >= 0) {
                    KASSERT(ring_buffer_put(&rv_queue, rv_train) == 0);
                    rv_pending_count++;
                    Create(TRAIN_COURIER_PRIORITY, rv_delay_task);
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_CAN_FRAME:
                process_can_frame(&msg.frame, msg.arrival_us);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRAIN_POS_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                pos_on_tick(read_timer());
                break;

            case TRAIN_POS_REPLAN_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                pos_replan_on_tick(read_timer());
                break;

            case TRAIN_POS_SWITCH_SETTLE_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                pos_on_switch_settle_tick(read_timer());
                break;

            case TRAIN_MSG_RV_REQUEST: {
                int train = -1;
                int speed_level = 0;

                if (ring_buffer_get(&rv_queue, &train) < 0) {
                    train = -1;
                }
                if (train >= 0) {
                    train_pos_t *pos = pos_get(train);
                    if (pos) {
                        speed_level = pos->user_speed;
                    }
                }

                reply.train = train;
                reply.delay_ticks = (6000 * speed_level) / 10000;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_RV_COMPLETE:
                track_complete_reverse(msg.train);
                pos_on_reverse(msg.train);
                if (rv_pending_count > 0) {
                    rv_pending_count--;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRAIN_MSG_DEMO_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                demo_on_tick(read_timer());
                break;

            default:
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}
