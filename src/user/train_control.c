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
#include "uart.h"
#include "task_manager.h"
#include "ring_buffer.h"
#include "text_util.h"
#include "kassert.h"


// Reverse delay pending queue
#define RV_QUEUE_MAX 8
RING_BUFFER_DECLARE(RVQueue_t, int, RV_QUEUE_MAX);
static RVQueue_t Rv_Queue;

static CmdQueue_t Cmd_Queue;

static const uint32_t REV_COOLDOWN_MULTIPLIER_NUMERATOR = 7;
static const uint32_t REV_COOLDOWN_MULTIPLIER_DENOMINATOR = 10;


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

    msg.type = TRAIN_MSG_RV_ACCEL_REQUEST;
    msg.train = train;
    Send(parent, (const char *)&msg, sizeof(msg),
         (char *)&reply, sizeof(reply));

    delay_ticks = reply.delay_ticks;
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
static void process_can_frame(CanData_t *frame, uint64_t now) {
    if (!is_marklin_sensor_data(frame)) return;

    SensorData_t sensor_data;
    can_data_get_sensor(frame, &sensor_data);

    if (!sensor_data_is_valid(&sensor_data)) return;
    
    track_log_sensor(&sensor_data, now);
    ui_mark_sensors_dirty();
}

static int get_train_rv_delay(int train) {
    int result = get_train_rv_prev_speed(train);
    if (result != -1) {
        result =  result * REV_COOLDOWN_MULTIPLIER_NUMERATOR / REV_COOLDOWN_MULTIPLIER_DENOMINATOR; // 1 tick about 10ms
    } else {
        result = 0;
    }

    return result;
}

// train_init_states(): Physically initialize the state of the track
static void train_init_states() {
    uint32_t switch_no;
    char direction;

    // remember to not stop all possible trains (255) at once to not hang MARKLIN CS3
    uint32_t physical_trains[PHYSICAL_TRAINS_COUNT] = {13, 14, 15, 17, 18, 55};
    for (int i = 0; i < PHYSICAL_TRAINS_COUNT; ++i) {
        track_set_speed(physical_trains[i], 0);
    }

    char switch_positions[SWITCH_COUNT] = {SWITCH_CURVED, SWITCH_CURVED, SWITCH_STRAIGHT, SWITCH_STRAIGHT,
                                            SWITCH_STRAIGHT, SWITCH_STRAIGHT, SWITCH_STRAIGHT, SWITCH_STRAIGHT,
                                            SWITCH_STRAIGHT, SWITCH_STRAIGHT, SWITCH_CURVED, SWITCH_CURVED,
                                            SWITCH_STRAIGHT, SWITCH_STRAIGHT, SWITCH_STRAIGHT, SWITCH_STRAIGHT,
                                            SWITCH_STRAIGHT, SWITCH_CURVED, SWITCH_STRAIGHT, SWITCH_STRAIGHT,
                                            SWITCH_STRAIGHT, SWITCH_STRAIGHT};

    for (int i = 0; i <  MAX_SWITCHES; ++i) {
        switch_no = get_switch_no(i);
        direction = switch_positions[i];

        track_set_switch(switch_no, direction);
        track_update_switch(switch_no, direction);
    }

    ui_mark_switches_dirty();
}

// clean_cmd_execute: Perform any cleanup job after executing the command
static void clean_cmd_execute(int exec_result, int *running, int rv_train, uint32_t *rv_count) {
    if (exec_result == 0) {
        *running = 0;  // for 'q' command
    }

    if (rv_train >= 0) {
        // design limit, only allow 8 pending reversals. use kassert to check is not good, but for simplicity
        KASSERT(*rv_count <= MAX_ACTIVE_TRAINS && ring_buffer_put(&Rv_Queue, rv_train) == 0);
        (*rv_count)++;
        Create(TRAIN_COURIER_PRIORITY, rv_delay_task);
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
    ring_buffer_init(&Rv_Queue);
    ring_buffer_init(&Cmd_Queue);
    uint32_t rv_count = 0;

    Create(TRAIN_COURIER_PRIORITY, can_rx_courier_task);
    Create(TRAIN_COURIER_PRIORITY, keyboard_courier_task);
    Create(TRAIN_COURIER_PRIORITY, ui_tick_task);

    CANEnableInterrupts(can_tid);

    char cmdline[80];
    int cmdlen = 0;

    uint64_t start_us = read_timer();
    int running = 1;

    Putc(term_tid, TERM_CHANNEL_CONSOLE, '\0');  
    train_init_states();

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
                        int result = execute_cmd(cmdline, &rv_train, &Cmd_Queue);
                        clean_cmd_execute(result, &running, rv_train, &rv_count);
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
                if (ring_buffer_get(&Rv_Queue, &train) < 0) {
                    train = -1;
                }

                reply.train = train;
                reply.delay_ticks = get_train_rv_delay(train);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_RV_ACCEL_REQUEST: {
                track_complete_reverse(msg.train);
                reply.delay_ticks = get_train_rv_delay(msg.train);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case TRAIN_MSG_RV_COMPLETE: {
                track_reset_reverse(msg.train);
                --rv_count;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            default:
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
        
        CommandData_t buffed_cmd;
        int exec_result;
        int rv_train = -1;
        int buffed_cmds_count = Cmd_Queue.count;
        bool is_reversing;

        // try to execute the buffered commands
        for (int i = 0; i < buffed_cmds_count; ++i) {
            ring_buffer_peek(&Cmd_Queue, &buffed_cmd);
            is_reversing = is_train_reversing(buffed_cmd.arg1);
            if (is_reversing) continue;

            ring_buffer_get(&Cmd_Queue, &buffed_cmd);
            exec_result = execute_cmd_data(&buffed_cmd, &rv_train, &Cmd_Queue);
            clean_cmd_execute(exec_result, &running, rv_train, &rv_count);
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
