#include "train_control.h"
#include "train_runtime.h"
#include "runtime_protocol.h"
#include "command.h"
#include "syscall.h"
#include "server/nameserver.h"
#include "server/clock_server.h"
#include "server/terminal_server.h"
#include "ui.h"
#include "idle_task.h"
#include "timer.h"
#include "task_manager.h"
#include "ring_buffer.h"
#include "util.h"
#include "kassert.h"

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

static uint64_t ui_start_us;
static int keyboard_buffer_tid = -1;

static void keyboard_buffer_task(void);
static void keyboard_rx_task(void);

static int keyboard_buffer_putc(int tid, char ch) {
    KeyboardMsg_t req;
    KeyboardReply_t reply;

    req.type = KEYBOARD_MSG_PUT_CHAR;
    req.ch = ch;

    if (Send(tid, (const char *)&req, sizeof(req),
             (char *)&reply, sizeof(reply)) < 0) {
        return -1;
    }
    return reply.status;
}

static int keyboard_buffer_getc(int tid) {
    KeyboardMsg_t req;
    KeyboardReply_t reply;

    req.type = KEYBOARD_MSG_GET_CHAR;
    req.ch = 0;

    if (Send(tid, (const char *)&req, sizeof(req),
             (char *)&reply, sizeof(reply)) < 0 ||
        reply.status < 0) {
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
                    Reply(waiter_tid, (const char *)&waiter_reply,
                          sizeof(waiter_reply));
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

void ui_tick_task(void) {
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);

    KASSERT(clock_tid >= 0);

    for (;;) {
        Delay(clock_tid, 4);

        ui_update_clock(ui_start_us, read_timer());
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

void train_control_task(void) {
    int tid;
    int runtime_tid;
    int running = 1;
    runtime_event_t event;
    runtime_reply_t reply;
    runtime_reply_t runtime_reply;

    int term_tid = WhoIs(TERMINAL_SERVER_NAME);
    KASSERT(term_tid >= 0);

    runtime_tid = Create(RUNTIME_CORE_PRIORITY, runtime_core_task);
    KASSERT(runtime_tid >= 0);

    {
        int msglen = Receive(&tid, (char *)&event, sizeof(event));
        (void)msglen;
        KASSERT(tid == runtime_tid);
        KASSERT(event.type == RUNTIME_EVENT_RUNTIME_READY);
        reply.status = 0;
        Reply(tid, (const char *)&reply, sizeof(reply));
    }

    ui_start_us = read_timer();
    ui_init(term_tid);

    keyboard_buffer_tid = Create(KEYBOARD_BUFFER_PRIORITY, keyboard_buffer_task);
    KASSERT(keyboard_buffer_tid >= 0);

    Create(TRAIN_COURIER_PRIORITY, keyboard_rx_task);
    Create(TRAIN_COURIER_PRIORITY, command_input_task);
    Create(UI_TICK_PRIORITY, ui_tick_task);

    while (running) {
        int msglen = Receive(&tid, (char *)&event, sizeof(event));
        (void)msglen;

        reply.status = 0;

        switch (event.type) {
            case RUNTIME_EVENT_COMMAND:
                if (Send(runtime_tid, (const char *)&event, sizeof(event),
                         (char *)&runtime_reply, sizeof(runtime_reply)) < 0) {
                    reply.status = -1;
                } else {
                    reply.status = runtime_reply.status;
                    if (runtime_reply.status == 0) {
                        running = 0;
                    }
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            default:
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }

    Shutdown();
}

void command_input_task(void) {
    int parent = MyParentTid();
    runtime_event_t event;
    runtime_reply_t reply;
    char cmdline[TRAIN_CMD_MAX_LEN];
    char prompt_label[16];
    char last_prompt_label[16];
    int cmdlen = 0;

    KASSERT(keyboard_buffer_tid >= 0);

    event.type = RUNTIME_EVENT_COMMAND;
    ui_get_cmd_prompt_label(last_prompt_label, sizeof(last_prompt_label));

    for (;;) {
        int c = keyboard_buffer_getc(keyboard_buffer_tid);
        if (c < 0) continue;

        ui_get_cmd_prompt_label(prompt_label, sizeof(prompt_label));
        if (!str_eq(prompt_label, last_prompt_label)) {
            cmdlen = 0;
            for (int i = 0; i < (int)sizeof(last_prompt_label); i++) {
                last_prompt_label[i] = prompt_label[i];
                if (prompt_label[i] == '\0') break;
            }
        }

        if (c == '\r' || c == '\n') {
            cmdline[cmdlen] = '\0';
            ui_scroll_cmd();

            if (cmdlen > 0 && parse_train_command(cmdline, &event.command)) {
                Send(parent, (const char *)&event, sizeof(event),
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
