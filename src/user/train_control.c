#include "train_control.h"
#include "syscall.h"
#include "server/nameserver.h"
#include "server/clock_server.h"
#include "server/terminal_server.h"
#include "server/can_server.h"
#include "track.h"
#include "train_tracking/position.h"
#include "train_tracking/traffic_manager.h"
#include "ui.h"
#include "idle_task.h"
#include "command.h"
#include "timer.h"
#include "task_manager.h"
#include "ring_buffer.h"
#include "kassert.h"
#include "demo_manager.h"

// Reverse delay pending queue
#define RV_QUEUE_MAX 8
RING_BUFFER_DECLARE(RVQueue_t, int, RV_QUEUE_MAX);
static RVQueue_t rv_queue;

#define KEYBOARD_QUEUE_SIZE 1024
#define KEYBOARD_MSG_PUT_CHAR 0
#define KEYBOARD_MSG_GET_CHAR 1

RING_BUFFER_DECLARE(KeyboardQueue_t, char, KEYBOARD_QUEUE_SIZE);

typedef struct {
    int type;
    char ch;
} KeyboardMsg_t;

typedef struct {
    int status;
    char ch;
} KeyboardReply_t;

RING_BUFFER_DECLARE(DeadTrainQueue_t, int, MAX_ACTIVE_TRAINS + 1);
static DeadTrainQueue_t DeadTrainQueue;

static int Train_Nums[MAX_ACTIVE_TRAINS] = {13, 14, 15, 17, 18};
static uint64_t ui_start_us;
static int keyboard_buffer_tid = -1;

static void keyboard_buffer_task(void);
static void keyboard_rx_task(void);

static int keyboard_buffer_putc(int tid, char ch) {
    KeyboardMsg_t req;
    KeyboardReply_t reply;

    req.type = KEYBOARD_MSG_PUT_CHAR;
    req.ch = ch;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;
    }
    return reply.status;
}

static int keyboard_buffer_getc(int tid) {
    KeyboardMsg_t req;
    KeyboardReply_t reply;

    req.type = KEYBOARD_MSG_GET_CHAR;
    req.ch = 0;

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0 || reply.status < 0) {
        return -1;
    }
    return (unsigned char)reply.ch;
}

static void keyboard_buffer_task(void) {
    int tid;
    KeyboardMsg_t req;
    KeyboardReply_t reply;
    KeyboardQueue_t queue;
    int waiter_tid = -1;

    ring_buffer_init(&queue);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        switch (req.type) {
            case KEYBOARD_MSG_PUT_CHAR:
                if (waiter_tid >= 0) {
                    KeyboardReply_t waiter_reply;
                    waiter_reply.status = 0;
                    waiter_reply.ch = req.ch;
                    Reply(waiter_tid, (const char *)&waiter_reply, sizeof(waiter_reply));
                    waiter_tid = -1;
                } else {
                    if (ring_buffer_is_full(&queue)) {
                        char dropped;
                        ring_buffer_get(&queue, &dropped);
                    }
                    KASSERT(ring_buffer_put(&queue, req.ch) == 0);
                }

                reply.status = 0;
                reply.ch = 0;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case KEYBOARD_MSG_GET_CHAR:
                if (ring_buffer_get(&queue, &reply.ch) == 0) {
                    reply.status = 0;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                } else if (waiter_tid < 0) {
                    waiter_tid = tid;
                } else {
                    reply.status = -1;
                    reply.ch = 0;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                }
                break;

            default:
                reply.status = -1;
                reply.ch = 0;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}

static void keyboard_rx_task(void) {
    int term_tid = WhoIs(TERMINAL_SERVER_NAME);

    KASSERT(term_tid >= 0);
    KASSERT(keyboard_buffer_tid >= 0);

    for (;;) {
        int c = Getc(term_tid, TERM_CHANNEL_CONSOLE);
        if (c >= 0) {
            keyboard_buffer_putc(keyboard_buffer_tid, (char)c);
        }
    }
}

void rv_delay_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    msg.type = TRAIN_MSG_RV_REQUEST;
    Send(parent, (const char *)&msg, sizeof(msg),
         (char *)&reply, sizeof(reply));

    int train = reply.train;
    int delay_ticks = reply.delay_ticks;

    Delay(clock_tid, delay_ticks);

    msg.type = TRAIN_MSG_RV_COMPLETE;
    msg.train = train;
    Send(parent, (const char *)&msg, sizeof(msg),
         (char *)&reply, sizeof(reply));

    Exit();
}

// receives frames and sends to parent
static void can_rx_courier_task(void) {
    int parent = MyParentTid();
    int can_tid = WhoIs(CAN_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    msg.type = TRAIN_MSG_CAN_FRAME;

    for (;;) {
        if (CANReceive(can_tid, &msg.frame, &msg.arrival_us) == 0) {
            Send(parent, (const char *)&msg, sizeof(msg),
                 (char *)&reply, sizeof(reply));
        }
    }
}

void demo_tick_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(clock_tid >= 0);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;
    msg.type = TRAIN_MSG_DEMO_TICK;

    for (;;) {
        Delay(clock_tid, 100);  // 100 * 10ms = 1s
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

void ui_tick_task(void) {
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(clock_tid >= 0);

    const int tick_interval = 4;  // 4 ticks * 10ms = 40ms

    for (;;) {
        Delay(clock_tid, tick_interval);

        uint64_t tick_now = read_timer();
        ui_update_clock(ui_start_us, tick_now);
        ui_update_idle(get_idle_percentage());

        if (ui_is_switches_dirty()) {
            ui_switches();
            ui_mark_switches_clean();
        }
        if (ui_is_sensors_dirty()) {
            ui_draw_sensors(ui_start_us);
            ui_mark_sensors_clean();
        }
        if (ui_is_position_dirty()) {
            ui_draw_position();
            ui_draw_prediction_error();
            ui_draw_offroute();
            ui_mark_position_clean();
        }
    }
}

void pos_tick_task() {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(clock_tid >= 0);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;
    msg.type = TRAIN_POS_TICK;

    const int tick_interval = 10;

    for (;;) {
        Delay(clock_tid, tick_interval);
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

void add_dead_train_to_retry(int train_num) {
    ring_buffer_put(&DeadTrainQueue, train_num);
}

void retry_dead_train_task() {
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(clock_tid >= 0);

    const int tick_interval = 500;
    Delay(clock_tid, tick_interval);

    if (ring_buffer_is_empty(&DeadTrainQueue)) {
        Exit();
    }

    int train_num = -1;
    if (ring_buffer_get(&DeadTrainQueue, &train_num) != 0) {
        Exit();
    }

    int demo_train_ind = get_demo_train_ind(train_num);
    pos_reset_dead_train(train_num);
    demo_retry_train_by_ind(demo_train_ind);

    Exit();
}

void pos_replan_tick_task() {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(clock_tid >= 0);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;
    msg.type = TRAIN_POS_REPLAN_TICK;

    const int tick_interval = 100;

    for (;;) {
        Delay(clock_tid, tick_interval);
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

void pos_switch_settle_tick_task() {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(clock_tid >= 0);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;
    msg.type = TRAIN_POS_SWITCH_SETTLE_TICK;

    for (;;) {
        Delay(clock_tid, 1);
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

// Parse CAN frame for sensor data
static void process_can_frame(const can_frame_t *frame, uint64_t now) {
    uint8_t command     = (uint8_t)((frame->id >> 17) & 0xFF);
    int     is_response = (int)((frame->id >> 16) & 1);

    if (command == 0x11 && frame->dlc >= 5) {
        // Sensor event
        // data[0,1]: unused identification
        // data[2,3]: sensor ID = (bank-'A')*16 + (number-1) + 1 
        // data[4]: state (1=entering, 0=leaving)
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
        // physical switch has been commanded.
        // Update software state here so predict_next_sensor reflects reality.
        uint32_t sw_id_raw = ((uint32_t)frame->data[0] << 24) |
                             ((uint32_t)frame->data[1] << 16) |
                             ((uint32_t)frame->data[2] << 8)  |
                              (uint32_t)frame->data[3];
        int  sw_num = (int)(sw_id_raw - 0x3000 + 1);
        char dir    = (frame->data[4] == 0x01) ? 'S' : 'C';
        if (track_is_valid_switch(sw_num)) {
            track_update_switch(sw_num, dir);
            pos_mark_routes_dirty();
            ui_mark_switches_dirty();
        }
    }
}

static void init_trains() {
    int train_num;
    for (int i = 0; i < MAX_ACTIVE_TRAINS; ++i) {
        train_num = Train_Nums[i];
        track_set_speed(train_num, 0);
        track_set_light(train_num, 1);
    }
}

void train_control_task(void) {
    int tid;
    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    int term_tid = WhoIs(TERMINAL_SERVER_NAME);
    int can_tid = WhoIs(CAN_SERVER_NAME);
    KASSERT(term_tid >= 0);
    KASSERT(can_tid >= 0);

    track_init(can_tid);
    pos_init();
    demo_init();
    ui_init(term_tid);

    ring_buffer_init(&rv_queue);
    ring_buffer_init(&DeadTrainQueue);

    CANEnableInterrupts(can_tid);

    // set all switches to straight
    for (int sw = 1; sw <= 18; sw++) {
        track_set_switch(sw, 'S');
    }
    for (int sw = 153; sw <= 156; sw++) {
        char state = (sw == 153 || sw == 155) ? 'C' : 'S';
        track_set_switch(sw, state);
    }
    ui_mark_switches_dirty(); 
    init_trains();

    keyboard_buffer_tid = Create(TRAIN_CONTROL_PRIORITY, keyboard_buffer_task);
    KASSERT(keyboard_buffer_tid >= 0);

    Create(TRAIN_COURIER_PRIORITY, can_rx_courier_task);
    Create(TRAIN_COURIER_PRIORITY, keyboard_rx_task);
    Create(TRAIN_COURIER_PRIORITY, command_input_task);
    Create(TRAIN_COURIER_PRIORITY, ui_tick_task);
    Create(TRAIN_COURIER_PRIORITY, demo_tick_task);
    Create(TRAIN_CONTROL_PRIORITY, pos_tick_task);
    Create(TRAIN_CONTROL_PRIORITY, pos_replan_tick_task);
    Create(TRAIN_CONTROL_PRIORITY, pos_switch_settle_tick_task);

    int rv_pending_count = 0;  /* # rv still in progress */

    ui_start_us = read_timer();
    int running = 1;

    while (running) {
        // Receive message from command input, CAN courier, or timer
        int msglen = Receive(&tid, (char *)&msg, sizeof(msg));
        (void)msglen;

        reply.status = 0;

        switch (msg.type) {
            case TRAIN_MSG_COMMAND: {
                int rv_train = -1;
                int result = execute_it(msg.cmdline, &rv_train, rv_pending_count > 0);
                if (result == 0) {
                    running = 0;  // for 'q' command
                }
                if (rv_train >= 0) {
                    // design limit, only allow 8 pending reversals. use kassert to check is not good, but for simplicity
                    KASSERT(ring_buffer_put(&rv_queue, rv_train) == 0);
                    rv_pending_count++;
                    Create(TRAIN_COURIER_PRIORITY, rv_delay_task);
                }

                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_CAN_FRAME: {
                process_can_frame(&msg.frame, msg.arrival_us);

                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_POS_TICK: {
                Reply(tid, (const char *)&reply, sizeof(reply));
                uint64_t tick_now = read_timer();
                pos_on_tick(tick_now);
                break;
            }

            case TRAIN_POS_REPLAN_TICK: {
                Reply(tid, (const char *)&reply, sizeof(reply));
                uint64_t tick_now = read_timer();
                pos_replan_on_tick(tick_now);
                break;
            }

            case TRAIN_POS_SWITCH_SETTLE_TICK: {
                Reply(tid, (const char *)&reply, sizeof(reply));
                uint64_t tick_now = read_timer();
                pos_on_switch_settle_tick(tick_now);
                break;
            }

            case TRAIN_MSG_RV_REQUEST: {
                int train = -1;
                if (ring_buffer_get(&rv_queue, &train) < 0) {
                    train = -1;
                }
                reply.train = train;
                int speed_level = 0;
                if (train >= 0) {
                    train_pos_t *pos = pos_get(train);
                    if (pos) speed_level = pos->user_speed;
                }
                // delay_us = 6000 * speed_level (0-14); convert to 10ms ticks
                reply.delay_ticks = (6000 * speed_level) / 10000;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_RV_COMPLETE: {
                track_complete_reverse(msg.train);
                pos_on_reverse(msg.train);
                if (rv_pending_count > 0) rv_pending_count--;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_DEMO_TICK: {
                Reply(tid, (const char *)&reply, sizeof(reply));
                demo_on_tick(read_timer());
                break;
            }

            default:
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }

    Shutdown();
}


void command_input_task(void) {
    int parent = MyParentTid();
    KASSERT(keyboard_buffer_tid >= 0);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;
    char cmdline[TRAIN_CMD_MAX_LEN];
    int cmdlen = 0;

    msg.type = TRAIN_MSG_COMMAND;

    for (;;) {
        int c = keyboard_buffer_getc(keyboard_buffer_tid);
        if (c < 0) {
            continue;
        }

        if (c == '\r' || c == '\n') {
            cmdline[cmdlen] = '\0';
            ui_scroll_cmd();

            if (cmdlen > 0) {
                for (int i = 0; i <= cmdlen; i++) {
                    msg.cmdline[i] = cmdline[i];
                }
                Send(parent, (const char *)&msg, sizeof(msg),
                     (char *)&reply, sizeof(reply));
            }

            cmdlen = 0;
            ui_cmd_newprompt();
        } else if (c == 127 || c == '\b') {
            if (cmdlen > 0) {
                cmdlen--;
                ui_cmd_backspace();
            }
        } else if (c >= ' ' && c < 127 && cmdlen < TRAIN_CMD_MAX_LEN - 2) {
            cmdline[cmdlen++] = (char)c;
            ui_cmd_putc((char)c);
        }
    }
}
