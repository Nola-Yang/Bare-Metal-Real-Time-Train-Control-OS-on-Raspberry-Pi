#include "server/ui_server.h"
#include "server/nameserver.h"
#include "ui_priv.h"
#include "util.h"
#include "syscall.h"
#include "kassert.h"

#define UI_SERVER_MAX_WAITERS 8

typedef struct {
    int tid;
    int mode;
} UIServerWaiter_t;

static int ui_server_send_simple(int tid, int type, int mode, char ch,
                                 const char *str, int len) {
    UIServerRequest_t req;
    UIServerReply_t reply;

    req.type = type;
    req.mode = mode;
    req.len = len;
    req.ch = ch;
    if (str != NULL && len > 0) {
        for (int i = 0; i < len; i++) {
            req.str[i] = str[i];
        }
    }

    int ret = Send(tid, (const char *)&req, sizeof(req),
                   (char *)&reply, sizeof(reply));
    if (ret < 0) {
        return -1;
    }
    return reply.status;
}

static int ui_server_begin_batch(int tid, int mode) {
    return ui_server_send_simple(tid, UI_SERVER_MSG_ACQUIRE, mode, 0, NULL, 0);
}

static int ui_server_end_batch(int tid, int mode) {
    return ui_server_send_simple(tid, UI_SERVER_MSG_RELEASE, mode, 0, NULL, 0);
}

static void term_puts_chunked(int term_tid, const char *str, int len) {
    int offset = 0;
    while (offset < len) {
        int chunk = len - offset;
        if (chunk > TERM_MAX_STR_LEN) {
            chunk = TERM_MAX_STR_LEN;
        }
        Puts(term_tid, TERM_CHANNEL_CONSOLE, str + offset, chunk);
        offset += chunk;
    }
}

void ui_server_task(void) {
    int tid;
    UIServerRequest_t req;
    UIServerReply_t reply;
    int term_tid = -1;
    int owner_tid = -1;
    int owner_mode = -1;
    int owner_depth = 0;
    int cmd_prepared = 0;
    UIServerWaiter_t waiters[UI_SERVER_MAX_WAITERS];
    int waiter_count = 0;

    while (term_tid < 0) {
        term_tid = WhoIs(TERMINAL_SERVER_NAME);
        if (term_tid < 0) {
            Yield();
        }
    }
    RegisterAs(UI_SERVER_NAME);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        switch (req.type) {
            case UI_SERVER_MSG_ACQUIRE:
                if (owner_tid < 0 || owner_tid == tid) {
                    owner_tid = tid;
                    owner_mode = req.mode;
                    owner_depth++;
                    if (owner_depth == 1 &&
                        owner_mode == UI_SERVER_BATCH_RAW &&
                        cmd_prepared) {
                        term_puts_chunked(term_tid, "\033[s", 3);
                    }
                    reply.status = 0;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                } else if (waiter_count < UI_SERVER_MAX_WAITERS) {
                    waiters[waiter_count].tid = tid;
                    waiters[waiter_count].mode = req.mode;
                    waiter_count++;
                } else {
                    reply.status = -1;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                }
                break;

            case UI_SERVER_MSG_RELEASE:
                if (owner_tid != tid || owner_depth <= 0) {
                    reply.status = -1;
                    Reply(tid, (const char *)&reply, sizeof(reply));
                    break;
                }

                owner_depth--;
                if (owner_depth == 0) {
                    if (owner_mode == UI_SERVER_BATCH_RAW && cmd_prepared) {
                        term_puts_chunked(term_tid, "\033[u", 3);
                    }
                    owner_tid = -1;
                    owner_mode = -1;

                    if (waiter_count > 0) {
                        owner_tid = waiters[0].tid;
                        owner_mode = waiters[0].mode;
                        owner_depth = 1;
                        if (owner_mode == UI_SERVER_BATCH_RAW && cmd_prepared) {
                            term_puts_chunked(term_tid, "\033[s", 3);
                        }

                        for (int i = 1; i < waiter_count; i++) {
                            waiters[i - 1] = waiters[i];
                        }
                        waiter_count--;

                        UIServerReply_t waiter_reply;
                        waiter_reply.status = 0;
                        Reply(owner_tid, (const char *)&waiter_reply, sizeof(waiter_reply));
                    }
                }

                reply.status = 0;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case UI_SERVER_MSG_RAW_PUTC:
                if (owner_tid != tid || owner_mode != UI_SERVER_BATCH_RAW) {
                    reply.status = -1;
                } else {
                    Putc(term_tid, TERM_CHANNEL_CONSOLE, req.ch);
                    reply.status = 0;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case UI_SERVER_MSG_RAW_PUTS:
                if (owner_tid != tid || owner_mode != UI_SERVER_BATCH_RAW) {
                    reply.status = -1;
                } else {
                    term_puts_chunked(term_tid, req.str, req.len);
                    reply.status = req.len;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case UI_SERVER_MSG_CMD_PUTC:
                if (owner_tid != tid || owner_mode != UI_SERVER_BATCH_COMMAND) {
                    reply.status = -1;
                } else {
                    Putc(term_tid, TERM_CHANNEL_CONSOLE, req.ch);
                    reply.status = 0;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case UI_SERVER_MSG_CMD_PUTS:
                if (owner_tid != tid || owner_mode != UI_SERVER_BATCH_COMMAND) {
                    reply.status = -1;
                } else {
                    term_puts_chunked(term_tid, req.str, req.len);
                    reply.status = req.len;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case UI_SERVER_MSG_CMD_BACKSPACE:
                if (owner_tid != tid || owner_mode != UI_SERVER_BATCH_COMMAND) {
                    reply.status = -1;
                } else {
                    term_puts_chunked(term_tid, "\b \b", 3);
                    reply.status = 0;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case UI_SERVER_MSG_CMD_ENTER:
                if (owner_tid != tid || owner_mode != UI_SERVER_BATCH_COMMAND) {
                    reply.status = -1;
                } else {
                    term_puts_chunked(term_tid, "\r\n", 2);
                    reply.status = 0;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case UI_SERVER_MSG_CMD_PROMPT:
                if (owner_tid != tid || owner_mode != UI_SERVER_BATCH_COMMAND) {
                    reply.status = -1;
                } else {
                    char buf[64];
                    char *p = buf;
                    char *end = buf + sizeof(buf) - 1;
                    p = buf_append_cap(p, end, "\r\033[2K");
                    if (req.len > 0) {
                        for (int i = 0; i < req.len && p < end; i++) {
                            *p++ = req.str[i];
                        }
                    } else {
                        p = buf_append_cap(p, end, "cmd> ");
                    }
                    *p = '\0';
                    term_puts_chunked(term_tid, buf, (int)(p - buf));
                    reply.status = 0;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case UI_SERVER_MSG_CMD_PREPARE:
                if (owner_tid != tid || owner_mode != UI_SERVER_BATCH_COMMAND) {
                    reply.status = -1;
                } else {
                    char buf[64];
                    char *p = buf;
                    char *end = buf + sizeof(buf) - 1;
                    p = buf_append_cap(p, end, "\033[");
                    p = buf_append_int_cap(p, end, UI_CMD_SCROLL_TOP);
                    p = buf_append_cap(p, end, ";");
                    p = buf_append_int_cap(p, end, UI_CMD_SCROLL_BOTTOM);
                    p = buf_append_cap(p, end, "r\033[");
                    p = buf_append_int_cap(p, end, UI_CMD_SCROLL_TOP);
                    p = buf_append_cap(p, end, ";1H");
                    if (req.len > 0) {
                        for (int i = 0; i < req.len && p < end; i++) {
                            *p++ = req.str[i];
                        }
                    } else {
                        p = buf_append_cap(p, end, "cmd> ");
                    }
                    *p = '\0';

                    term_puts_chunked(term_tid, buf, (int)(p - buf));
                    cmd_prepared = 1;
                    reply.status = 0;
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            default:
                reply.status = -1;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}

static int ui_server_write_chunked(int tid, int batch_mode, int type,
                                   const char *str, int len) {
    int written = 0;
    if (ui_server_begin_batch(tid, batch_mode) != 0) {
        return -1;
    }

    while (written < len) {
        int chunk = len - written;
        if (chunk > TERM_MAX_STR_LEN) {
            chunk = TERM_MAX_STR_LEN;
        }
        int ret = ui_server_send_simple(tid, type, batch_mode, 0, str + written, chunk);
        if (ret < 0) {
            ui_server_end_batch(tid, batch_mode);
            return -1;
        }
        written += chunk;
    }

    if (ui_server_end_batch(tid, batch_mode) != 0) {
        return -1;
    }
    return written;
}

int UIServerPutc(int tid, char ch) {
    if (ui_server_begin_batch(tid, UI_SERVER_BATCH_RAW) != 0) {
        return -1;
    }
    int ret = ui_server_send_simple(tid, UI_SERVER_MSG_RAW_PUTC,
                                    UI_SERVER_BATCH_RAW, ch, NULL, 0);
    ui_server_end_batch(tid, UI_SERVER_BATCH_RAW);
    return ret;
}

int UIServerPuts(int tid, const char *str, int len) {
    if (str == NULL || len <= 0) {
        return 0;
    }
    return ui_server_write_chunked(tid, UI_SERVER_BATCH_RAW,
                                   UI_SERVER_MSG_RAW_PUTS, str, len);
}

int UIServerCmdPutc(int tid, char ch) {
    if (ui_server_begin_batch(tid, UI_SERVER_BATCH_COMMAND) != 0) {
        return -1;
    }
    int ret = ui_server_send_simple(tid, UI_SERVER_MSG_CMD_PUTC,
                                    UI_SERVER_BATCH_COMMAND, ch, NULL, 0);
    ui_server_end_batch(tid, UI_SERVER_BATCH_COMMAND);
    return ret;
}

int UIServerCmdPuts(int tid, const char *str, int len) {
    if (str == NULL || len <= 0) {
        return 0;
    }
    return ui_server_write_chunked(tid, UI_SERVER_BATCH_COMMAND,
                                   UI_SERVER_MSG_CMD_PUTS, str, len);
}

int UIServerCmdBackspace(int tid) {
    if (ui_server_begin_batch(tid, UI_SERVER_BATCH_COMMAND) != 0) {
        return -1;
    }
    int ret = ui_server_send_simple(tid, UI_SERVER_MSG_CMD_BACKSPACE,
                                    UI_SERVER_BATCH_COMMAND, 0, NULL, 0);
    ui_server_end_batch(tid, UI_SERVER_BATCH_COMMAND);
    return ret;
}

int UIServerCmdEnter(int tid) {
    if (ui_server_begin_batch(tid, UI_SERVER_BATCH_COMMAND) != 0) {
        return -1;
    }
    int ret = ui_server_send_simple(tid, UI_SERVER_MSG_CMD_ENTER,
                                    UI_SERVER_BATCH_COMMAND, 0, NULL, 0);
    ui_server_end_batch(tid, UI_SERVER_BATCH_COMMAND);
    return ret;
}

int UIServerCmdPrompt(int tid) {
    return UIServerCmdPromptLabel(tid, NULL, 0);
}

int UIServerPrepareCmd(int tid) {
    return UIServerPrepareCmdLabel(tid, NULL, 0);
}

int UIServerCmdPromptLabel(int tid, const char *label, int len) {
    if (ui_server_begin_batch(tid, UI_SERVER_BATCH_COMMAND) != 0) {
        return -1;
    }
    int ret = ui_server_send_simple(tid, UI_SERVER_MSG_CMD_PROMPT,
                                    UI_SERVER_BATCH_COMMAND, 0, label, len);
    ui_server_end_batch(tid, UI_SERVER_BATCH_COMMAND);
    return ret;
}

int UIServerPrepareCmdLabel(int tid, const char *label, int len) {
    if (ui_server_begin_batch(tid, UI_SERVER_BATCH_COMMAND) != 0) {
        return -1;
    }
    int ret = ui_server_send_simple(tid, UI_SERVER_MSG_CMD_PREPARE,
                                    UI_SERVER_BATCH_COMMAND, 0, label, len);
    ui_server_end_batch(tid, UI_SERVER_BATCH_COMMAND);
    return ret;
}
