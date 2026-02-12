#include "train_control.h"
#include "syscall.h"
#include "nameserver.h"
#include "clock_server.h"
#include "terminal_server.h"
#include "can_server.h"
#include "track.h"
#include "ui.h"
#include "idle_task.h"
#include "command.h"
#include "timer.h"
#include "task_manager.h"
#include "ring_buffer.h"
#include "kassert.h"

// Reverse delay pending queue
#define RV_QUEUE_MAX 4
RING_BUFFER_DECLARE(RVQueue_t, int, RV_QUEUE_MAX);
static RVQueue_t rv_queue;

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
        if (CANReceive(can_tid, &msg.frame) == 0) {
            Send(parent, (const char *)&msg, sizeof(msg),
                 (char *)&reply, sizeof(reply));
        }
    }
}

void ui_tick_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(clock_tid >= 0);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;
    msg.type = TRAIN_MSG_TICK;

    const int tick_interval = 10;  // 10 ticks * 10ms = 100ms

    for (;;) {
        Delay(clock_tid, tick_interval);
        Send(parent, (const char *)&msg, sizeof(msg),
             (char *)&reply, sizeof(reply));
    }
}

// Parse CAN frame for sensor data
static void process_can_frame(const can_frame_t *frame, uint64_t now) {
    uint8_t command = (frame->id >> 17) & 0xFF;

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
        }
    }
}


void train_control_task(void) {
    int tid;
    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    int term_tid = WhoIs(TERMINAL_SERVER_NAME);
    int can_tid = WhoIs(CAN_SERVER_NAME);
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(term_tid >= 0);
    KASSERT(can_tid >= 0);
    KASSERT(clock_tid >= 0);

    track_init(can_tid, term_tid);
    ui_init(term_tid);
    ring_buffer_init(&rv_queue);

    Create(TRAIN_COURIER_PRIORITY, can_rx_courier_task);
    Create(TRAIN_COURIER_PRIORITY, keyboard_courier_task);
    Create(TRAIN_COURIER_PRIORITY, ui_tick_task);

    CANEnableInterrupts(can_tid);

    char cmdline[80];
    int cmdlen = 0;

    uint64_t start_us = read_timer();
    int running = 1;

    Putc(term_tid, TERM_CHANNEL_CONSOLE, '\0');  

    while (running) {
        // Receive message from keyboard courier, CAN courier, or timer
        int msglen = Receive(&tid, (char *)&msg, sizeof(msg));
        (void)msglen;

        reply.status = 0;

        switch (msg.type) {
            case TRAIN_MSG_CHAR: {
                char c = msg.ch;

                if (c == '\r' || c == '\n') {
                    cmdline[cmdlen] = '\0';
                    ui_scroll_cmd();

                    if (cmdlen > 0) {
                        int rv_train = -1;
                        int result = execute_it(cmdline, &rv_train);
                        if (result == 0) {
                            running = 0;  // for 'q' command
                        }
                        if (rv_train >= 0) {
                            ring_buffer_put(&rv_queue, rv_train);
                            Create(TRAIN_COURIER_PRIORITY, rv_delay_task);
                        }
                    }

                    cmdlen = 0;
                    ui_cmd_newprompt();
                } else if (c == 127 || c == '\b') {
                    // Backspace
                    if (cmdlen > 0) {
                        cmdlen--;
                        ui_cmd_backspace();
                    }
                } else if (c >= ' ' && c < 127 && cmdlen < 78) {
                    // Printable character
                    cmdline[cmdlen++] = c;
                    ui_cmd_putc(c);  // Echo
                }

                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_CAN_FRAME: {
                uint64_t now = read_timer();
                process_can_frame(&msg.frame, now);

                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }
            case TRAIN_MSG_TICK: {
                // Periodic UI updates
                Reply(tid, (const char *)&reply, sizeof(reply));

                uint64_t tick_now = read_timer();
                ui_update_clock(start_us, tick_now);

                int idle_percent = get_idle_percentage();
                ui_update_idle(idle_percent);

                if (ui_is_switches_dirty()) {
                    ui_puts("\033[s");
                    ui_switches();
                    ui_puts("\033[u");
                    ui_mark_switches_clean();
                }
                if (ui_is_sensors_dirty()) {
                    ui_puts("\033[s");
                    ui_draw_sensors(start_us);
                    ui_puts("\033[u");
                    ui_mark_sensors_clean();
                }
                break;
            }

            case TRAIN_MSG_RV_REQUEST: {
                int train = -1;
                if (ring_buffer_get(&rv_queue, &train) < 0) {
                    train = -1;
                }
                reply.train = train;
                reply.delay_ticks = 100;  // 100 ticks * 10ms = 1s
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_RV_COMPLETE: {
                track_complete_reverse(msg.train);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            default:
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }

    Shutdown();
}


void keyboard_courier_task(void) {
    int parent = MyParentTid();
    int term_tid = WhoIs(TERMINAL_SERVER_NAME);

    TrainControlMsg_t msg;
    TrainControlReply_t reply;

    msg.type = TRAIN_MSG_CHAR;

    for (;;) {
        int c = Getc(term_tid, TERM_CHANNEL_CONSOLE);
        if (c >= 0) {
            msg.ch = (char)c;
            Send(parent, (const char *)&msg, sizeof(msg),
                 (char *)&reply, sizeof(reply));
        }
    }
}
