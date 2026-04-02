#include "train_runtime.h"
#include "runtime_protocol.h"
#include "syscall.h"
#include "server/nameserver.h"
#include "server/clock_server.h"
#include "server/can_server.h"
#include "server/track_server.h"
#include "server/position_server.h"
#include "server/demo_server.h"
#include "server/game_server.h"
#include "track.h"
#include "train_tracking/position.h"
#include "train_tracking/traffic_manager.h"
#include "demo_manager.h"
#include "game_manager.h"
#include "ui.h"
#include "timer.h"
#include "util.h"
#include "ring_buffer.h"
#include "kassert.h"

#define RV_QUEUE_MAX 8
#define DEADLOCK_PROMPT_LABEL "deadlock> "
#define DEADLOCK_PROMPT_LABEL_CAP 16
#define DEADLOCK_INPUT_MAX_TOKENS 16
RING_BUFFER_DECLARE(RVQueue_t, int, RV_QUEUE_MAX);
static RVQueue_t rv_queue;

typedef struct {
    int active;
    char saved_prompt_label[DEADLOCK_PROMPT_LABEL_CAP];
} runtime_deadlock_prompt_t;

static runtime_deadlock_prompt_t g_deadlock_prompt;

static void rv_delay_task(void) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);
    runtime_event_t event;
    runtime_reply_t reply;

    KASSERT(clock_tid >= 0);

    event.type = RUNTIME_EVENT_RV_REQUEST;
    Send(parent, (const char *)&event, sizeof(event),
         (char *)&reply, sizeof(reply));

    Delay(clock_tid, reply.delay_ticks);

    event.type = RUNTIME_EVENT_RV_COMPLETE;
    event.train = reply.train;
    Send(parent, (const char *)&event, sizeof(event),
         (char *)&reply, sizeof(reply));

    Exit();
}

static void runtime_tick_loop(int event_type, int delay_ticks) {
    int parent = MyParentTid();
    int clock_tid = WhoIs(CLOCK_SERVER_NAME);
    runtime_event_t event;
    runtime_reply_t reply;

    KASSERT(clock_tid >= 0);
    event.type = event_type;

    for (;;) {
        Delay(clock_tid, delay_ticks);
        event.now_us = read_timer();
        Send(parent, (const char *)&event, sizeof(event),
             (char *)&reply, sizeof(reply));
    }
}

static void runtime_fast_tick_task(void) {
    runtime_tick_loop(RUNTIME_EVENT_FAST_TICK, 3);
}

static void runtime_replan_tick_task(void) {
    runtime_tick_loop(RUNTIME_EVENT_REPLAN_TICK, 100);
}

static void runtime_demo_tick_task(void) {
    runtime_tick_loop(RUNTIME_EVENT_DEMO_TICK, 100);
}

static void runtime_game_tick_task(void) {
    runtime_tick_loop(RUNTIME_EVENT_GAME_TICK, 100);
}

static void runtime_switch_settle_tick_task(void) {
    runtime_tick_loop(RUNTIME_EVENT_SWITCH_SETTLE_TICK, 1);
}

static void runtime_deadlock_print_line(const char *text) {
    if (!text || !text[0]) return;
    ui_cmd_log_line(text);
}

static void runtime_deadlock_print_usage(void) {
    runtime_deadlock_print_line(
        "deadlock: enter up to 16 reservation nodes separated by spaces (e.g. A5 B6 C13*)");
}

static void runtime_deadlock_print_token_line(const char *prefix, const char *token) {
    char buf[128];
    char *p = buf;
    char *end = buf + sizeof(buf) - 1;

    p = buf_append_cap(p, end, prefix ? prefix : "deadlock:");
    if (token && token[0]) {
        p = buf_append_cap(p, end, " ");
        p = buf_append_cap(p, end, token);
    }
    *p = '\0';
    runtime_deadlock_print_line(buf);
}

static int runtime_deadlock_notice_unresolved(void) {
    pos_deadlock_notice_t notice;

    pos_get_deadlock_notice(&notice);
    return notice.active && notice.unresolved;
}

static void runtime_deadlock_prompt_activate(int redraw_prompt) {
    (void)redraw_prompt;

    if (g_deadlock_prompt.active) return;

    ui_get_cmd_prompt_label(g_deadlock_prompt.saved_prompt_label,
                            sizeof(g_deadlock_prompt.saved_prompt_label));
    if (!g_deadlock_prompt.saved_prompt_label[0]) {
        ui_set_cmd_prompt_label("cmd> ");
        ui_get_cmd_prompt_label(g_deadlock_prompt.saved_prompt_label,
                                sizeof(g_deadlock_prompt.saved_prompt_label));
    }

    g_deadlock_prompt.active = 1;
    ui_set_cmd_prompt_label(DEADLOCK_PROMPT_LABEL);
    runtime_deadlock_print_line(
        "deadlock: enter one or more reservation nodes separated by spaces (e.g. A5 B6 C13*)");
}

static void runtime_deadlock_prompt_deactivate(int redraw_prompt) {
    if (!g_deadlock_prompt.active) return;

    ui_set_cmd_prompt_label(g_deadlock_prompt.saved_prompt_label[0]
                                ? g_deadlock_prompt.saved_prompt_label
                                : "cmd> ");
    g_deadlock_prompt.active = 0;
    g_deadlock_prompt.saved_prompt_label[0] = '\0';
    if (redraw_prompt) ui_cmd_newprompt();
}

static void runtime_deadlock_prompt_refresh(int redraw_prompt) {
    if (runtime_deadlock_notice_unresolved()) {
        runtime_deadlock_prompt_activate(redraw_prompt);
    } else {
        runtime_deadlock_prompt_deactivate(redraw_prompt);
    }
}

static int runtime_deadlock_tokenize(char *cmd, char *argv[], int max_args,
                                     int *out_overflow) {
    int argc = 0;
    char *p = cmd;
    int overflow = 0;

    while (p && *p) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        if (argc >= max_args) {
            overflow = 1;
            break;
        }

        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }

    if (out_overflow) *out_overflow = overflow;
    return argc;
}

static void runtime_copy_small_token(char *dst, int cap, const char *src) {
    int i = 0;

    if (!dst || cap <= 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }

    while (src[i] && i + 1 < cap) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void runtime_deadlock_strip_trailing_star(char *token) {
    int len = 0;

    if (!token) return;
    while (token[len]) len++;
    if (len > 0 && token[len - 1] == '*') token[len - 1] = '\0';
}

static int runtime_deadlock_physical_key(track_node *node) {
    int idx;
    int ridx;

    if (!node) return -1;

    idx = (int)(node - g_track);
    if (idx < 0 || idx >= TRACK_MAX) return -1;

    ridx = node->reverse ? (int)(node->reverse - g_track) : -1;
    if (ridx >= 0 && ridx < TRACK_MAX && ridx < idx) return ridx;
    return idx;
}

static int runtime_deadlock_key_seen(const int *keys, int count, int key) {
    for (int i = 0; i < count; i++) {
        if (keys[i] == key) return 1;
    }
    return 0;
}

static int runtime_deadlock_node_reserved(track_node *node) {
    if (!node) return 0;
    if (traffic_get_node_owner(node) >= 0) return 1;
    return node->reverse != NULL && traffic_get_node_owner(node->reverse) >= 0;
}

static int runtime_handle_deadlock_prompt_command(const train_command_t *cmd,
                                                  int position_tid) {
    char raw_buf[TRAIN_CMD_MAX_LEN];
    char *argv[DEADLOCK_INPUT_MAX_TOKENS];
    int seen_keys[DEADLOCK_INPUT_MAX_TOKENS];
    int overflow = 0;
    int argc;
    int seen_count = 0;
    int released_any = 0;

    if (!cmd) return 2;
    if (cmd->type == TRAIN_CMD_QUIT) return 0;

    runtime_copy_small_token(raw_buf, sizeof(raw_buf), cmd->raw_cmdline);
    argc = runtime_deadlock_tokenize(raw_buf, argv, DEADLOCK_INPUT_MAX_TOKENS,
                                     &overflow);
    if (overflow || argc <= 0) {
        runtime_deadlock_print_usage();
        return 2;
    }

    for (int i = 0; i < argc; i++) {
        char typed_token[24];
        track_node *node;
        int key;

        runtime_copy_small_token(typed_token, sizeof(typed_token), argv[i]);
        runtime_deadlock_strip_trailing_star(argv[i]);
        if (!argv[i][0]) {
            runtime_deadlock_print_token_line("deadlock: unknown reservation token",
                                              typed_token);
            continue;
        }

        node = track_find_node(argv[i]);
        if (!node) {
            runtime_deadlock_print_token_line("deadlock: unknown reservation token",
                                              typed_token);
            continue;
        }

        key = runtime_deadlock_physical_key(node);
        if (key < 0 || runtime_deadlock_key_seen(seen_keys, seen_count, key)) {
            continue;
        }
        if (seen_count < DEADLOCK_INPUT_MAX_TOKENS) {
            seen_keys[seen_count++] = key;
        }

        if (!runtime_deadlock_node_reserved(node)) {
            runtime_deadlock_print_token_line("deadlock: token not reserved",
                                              typed_token);
            continue;
        }

        if (PositionServerReleaseNode(position_tid, (int)(node - g_track)) > 0) {
            runtime_deadlock_print_token_line("deadlock: released", typed_token);
            released_any = 1;
        } else {
            runtime_deadlock_print_token_line("deadlock: token not reserved",
                                              typed_token);
        }
    }

    if (released_any) {
        (void)PositionServerOnReplanTick(position_tid, read_timer());
        if (runtime_deadlock_notice_unresolved()) {
            runtime_deadlock_print_line("deadlock: still unresolved");
        }
    }

    return 2;
}

static void runtime_print_parse_error(const train_command_t *cmd) {
    if (!cmd) return;

    switch (cmd->error) {
        case TRAIN_CMD_ERR_USAGE_TR:
            ui_cmd_puts("Usage: tr <train> <speed 0-14>\r\n");
            break;
        case TRAIN_CMD_ERR_USAGE_SW:
            ui_cmd_puts("Usage: sw <switch> <S|C>\r\n");
            break;
        case TRAIN_CMD_ERR_USAGE_RV:
            ui_cmd_puts("Usage: rv <train>\r\n");
            break;
        case TRAIN_CMD_ERR_USAGE_LIGHT:
            ui_cmd_puts("Usage: li <train> <0|1>\r\n");
            break;
        case TRAIN_CMD_ERR_USAGE_GOTO:
            ui_cmd_puts("Usage: goto <train> <node> [offset_mm]\r\n");
            break;
        case TRAIN_CMD_ERR_USAGE_DEMO:
            ui_cmd_puts("Usage: demo <speed> <t1> [t2] [t3] [t4] [seed] | demo <locate|stop|tune|seed> ...\r\n");
            break;
        case TRAIN_CMD_ERR_USAGE_FINDPOS:
            ui_cmd_puts("Usage: findpos <t1> [t2] [t3] [t4]\r\n");
            break;
        case TRAIN_CMD_ERR_USAGE_GAME:
            ui_cmd_puts("Usage: game  (interactive) | game stop [force] | game status\r\n");
            break;
        case TRAIN_CMD_ERR_USAGE_PICK:
            ui_cmd_puts("Usage: pick <sensor>\r\n");
            break;
        case TRAIN_CMD_ERR_TRAIN_NOT_NUMBER:
            ui_cmd_puts("Train must be a number\r\n");
            break;
        case TRAIN_CMD_ERR_TRAIN_INVALID:
            ui_cmd_puts("Train must be one of: 13, 14, 15, 17, 18, 55\r\n");
            break;
        case TRAIN_CMD_ERR_SPEED_NOT_NUMBER:
            ui_cmd_puts("Speed must be a number\r\n");
            break;
        case TRAIN_CMD_ERR_SPEED_RANGE:
            ui_cmd_puts("Speed must be 0-14\r\n");
            break;
        case TRAIN_CMD_ERR_SWITCH_NOT_NUMBER:
            ui_cmd_puts("Switch must be a number\r\n");
            break;
        case TRAIN_CMD_ERR_SWITCH_INVALID:
            ui_cmd_puts("Invalid switch. Valid: 1-18, 153-156\r\n");
            break;
        case TRAIN_CMD_ERR_SWITCH_DIR:
            ui_cmd_puts("Direction must be S or C\r\n");
            break;
        case TRAIN_CMD_ERR_LIGHT_NOT_NUMBER:
            ui_cmd_puts("Light must be a number\r\n");
            break;
        case TRAIN_CMD_ERR_LIGHT_RANGE:
            ui_cmd_puts("Light must be 0 or 1\r\n");
            break;
        case TRAIN_CMD_ERR_OFFSET_NOT_NUMBER:
            ui_cmd_puts("Offset must be a number\r\n");
            break;
        case TRAIN_CMD_ERR_NODE_UNKNOWN:
            ui_cmd_puts("Unknown node name\r\n");
            break;
        case TRAIN_CMD_ERR_INVALID_GOTO_SPEED:
            ui_cmd_puts("Speed level not supported\r\n");
            break;
        default:
            if (cmd->type == TRAIN_CMD_UNKNOWN && cmd->argc > 0) {
                ui_cmd_puts("Unknown command: ");
                ui_cmd_puts(cmd->argv[0]);
                ui_cmd_puts("\r\n");
            }
            break;
    }
}

static int runtime_handle_command(const train_command_t *cmd,
                                  int position_tid,
                                  int demo_tid,
                                  int game_tid,
                                  int *rv_pending_count) {
    if (!cmd) return 2;

    if (g_deadlock_prompt.active) {
        if (cmd->type == TRAIN_CMD_GAME &&
            cmd->argc >= 2 &&
            str_eq(cmd->argv[1], "stop")) {
            return GameServerHandleCommand(game_tid, cmd);
        }
        return runtime_handle_deadlock_prompt_command(cmd, position_tid);
    }

    if (game_is_setup_active()) {
        switch (cmd->type) {
        case TRAIN_CMD_QUIT:
        case TRAIN_CMD_GAME:
        case TRAIN_CMD_UNKNOWN:
            break;
        default:
            ui_cmd_puts("game setup: enter train number (valid: 13 14 15 17 18 55)\r\n");
            return 2;
        }
    } else if (game_is_active()) {
        switch (cmd->type) {
        case TRAIN_CMD_QUIT:
        case TRAIN_CMD_GAME:
        case TRAIN_CMD_PICK:
        case TRAIN_CMD_UNKNOWN:
            break;
        default:
            ui_cmd_puts("game mode active: enter a sensor name\r\n");
            return 2;
        }
    }

    if (cmd->type == TRAIN_CMD_PARSE_ERROR) {
        runtime_print_parse_error(cmd);
        return 2;
    }
    if (cmd->type == TRAIN_CMD_UNKNOWN) {
        if (game_is_setup_active() || game_is_active()) {
            return GameServerHandleCommand(game_tid, cmd);
        }
        runtime_print_parse_error(cmd);
        return 2;
    }

    switch (cmd->type) {
        case TRAIN_CMD_QUIT:
            ui_cmd_puts("Rebooting...\r\n");
            return 0;

        case TRAIN_CMD_TR: {
            int can_speed = (cmd->value == 0) ? 0 : 1 + (cmd->value - 1) * 77;
            PositionServerSpeedChange(position_tid, cmd->train, cmd->value);
            track_set_speed(cmd->train, can_speed);
            return 1;
        }

        case TRAIN_CMD_SW: {
            int owner = traffic_can_set_switch(cmd->value, -1);
            if (owner >= 0) {
                char owner_buf[12];
                i2a(owner, owner_buf);
                ui_cmd_puts("Switch locked by tr ");
                ui_cmd_puts(owner_buf);
                ui_cmd_puts("\r\n");
                return 2;
            }

            track_set_switch(cmd->value, cmd->dir);
            return 1;
        }

        case TRAIN_CMD_RV: {
            int rv_result;

            if (pos_is_train_goto_active(cmd->train)) {
                ui_cmd_puts("Error: goto in progress for this train\r\n");
                return 2;
            }

            rv_result = track_start_reverse(cmd->train);
            if (rv_result == 1) {
                KASSERT(ring_buffer_put(&rv_queue, cmd->train) == 0);
                (*rv_pending_count)++;
                Create(TRAIN_COURIER_PRIORITY, rv_delay_task);
                return 1;
            }
            if (rv_result == 2) {
                PositionServerReverse(position_tid, cmd->train);
                return 1;
            }
            return 2;
        }

        case TRAIN_CMD_LIGHT:
            track_set_light(cmd->train, cmd->value);
            return 1;

        case TRAIN_CMD_GOTO:
            if (*rv_pending_count > 0) {
                ui_cmd_puts("goto: cannot execute while rv is in progress\r\n");
                return 2;
            }
            if (!PositionServerGoto(position_tid, cmd->train,
                                    cmd->target_idx, cmd->value, cmd->offset_mm)) {
                ui_cmd_puts("goto: no slot available\r\n");
                return 2;
            }
            return 1;

        case TRAIN_CMD_DEMO:
        case TRAIN_CMD_FINDPOS:
            return DemoServerHandleCommand(demo_tid, cmd);

        case TRAIN_CMD_GAME:
        case TRAIN_CMD_PICK:
            return GameServerHandleCommand(game_tid, cmd);

        default:
            runtime_print_parse_error(cmd);
            return 2;
    }
}

void runtime_core_task(void) {
    int parent = MyParentTid();
    int tid;
    int can_tid = WhoIs(CAN_SERVER_NAME);
    int rv_pending_count = 0;
    int track_tid;
    int position_tid;
    int demo_tid;
    int game_tid;
    runtime_event_t event;
    runtime_reply_t reply;

    KASSERT(can_tid >= 0);

    ring_buffer_init(&rv_queue);

    track_tid = Create(TRACK_SERVER_PRIORITY, track_server_task);
    KASSERT(track_tid >= 0);
    KASSERT(TrackServerInit(track_tid, can_tid) == 0);

    position_tid = Create(POSITION_SERVER_PRIORITY, position_server_task);
    KASSERT(position_tid >= 0);
    KASSERT(PositionServerInit(position_tid) == 0);

    demo_tid = Create(DEMO_SERVER_PRIORITY, demo_server_task);
    KASSERT(demo_tid >= 0);
    KASSERT(DemoServerInit(demo_tid) == 0);

    game_tid = Create(GAME_SERVER_PRIORITY, game_server_task);
    KASSERT(game_tid >= 0);
    KASSERT(GameServerInit(game_tid) == 0);

    event.type = RUNTIME_EVENT_RUNTIME_READY;
    Send(parent, (const char *)&event, sizeof(event),
         (char *)&reply, sizeof(reply));

    Create(RUNTIME_FAST_TICK_PRIORITY, runtime_fast_tick_task);
    Create(RUNTIME_FAST_TICK_PRIORITY, runtime_switch_settle_tick_task);
    Create(RUNTIME_SLOW_TICK_PRIORITY, runtime_replan_tick_task);
    Create(RUNTIME_SLOW_TICK_PRIORITY, runtime_demo_tick_task);
    Create(RUNTIME_SLOW_TICK_PRIORITY, runtime_game_tick_task);

    for (;;) {
        int msglen = Receive(&tid, (char *)&event, sizeof(event));
        (void)msglen;

        reply.status = 0;
        reply.train = -1;
        reply.delay_ticks = 0;

        switch (event.type) {
            case RUNTIME_EVENT_COMMAND:
                reply.status = runtime_handle_command(&event.command,
                                                      position_tid,
                                                      demo_tid,
                                                      game_tid,
                                                      &rv_pending_count);
                runtime_deadlock_prompt_refresh(0);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case RUNTIME_EVENT_SENSOR_HIT:
                Reply(tid, (const char *)&reply, sizeof(reply));
                /* Reply before forwarding so track_server is free to service
                 * any track_* calls triggered during sensor handling. */
                PositionServerOnSensor(position_tid, event.sensor_id, event.now_us);
                break;

            case RUNTIME_EVENT_SWITCH_ACK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                PositionServerMarkRoutesDirty(position_tid);
                break;

            case RUNTIME_EVENT_FAST_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                PositionServerOnFastTick(position_tid, event.now_us);
                break;

            case RUNTIME_EVENT_REPLAN_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                PositionServerOnReplanTick(position_tid, event.now_us);
                runtime_deadlock_prompt_refresh(1);
                break;

            case RUNTIME_EVENT_SWITCH_SETTLE_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                PositionServerOnSwitchSettleTick(position_tid, event.now_us);
                break;

            case RUNTIME_EVENT_DEMO_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                DemoServerOnTick(demo_tid, event.now_us);
                break;

            case RUNTIME_EVENT_GAME_TICK:
                Reply(tid, (const char *)&reply, sizeof(reply));
                GameServerOnTick(game_tid, event.now_us);
                break;

            case RUNTIME_EVENT_RV_REQUEST: {
                int train = -1;
                int speed_level = 0;

                if (ring_buffer_get(&rv_queue, &train) == 0) {
                    train_pos_t *pos = pos_get(train);
                    if (pos) speed_level = pos->user_speed;
                }

                reply.train = train;
                reply.delay_ticks = (6000 * speed_level) / 10000;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
            }

            case RUNTIME_EVENT_RV_COMPLETE:
                track_complete_reverse(event.train);
                PositionServerReverse(position_tid, event.train);
                if (rv_pending_count > 0) {
                    rv_pending_count--;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            default:
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}
