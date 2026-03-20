#include "ui.h"
#include "ui_priv.h"
#include "util.h"
#include "server/nameserver.h"
#include "server/ui_server.h"
#include "syscall.h"
#include "kassert.h"

static int ui_server_tid = -1;
static int ui_switches_dirty  = 1;
static int ui_sensors_dirty   = 1;
static int ui_position_dirty  = 1;

static char last_clock_buf[8] = "00:00.0";
static int last_idle_percent = -1;

static int ui_strlen(const char *str) {
    int len = 0;
    while (str != NULL && str[len]) {
        len++;
    }
    return len;
}

void ui_puts(const char *str) {
    if (ui_server_tid < 0 || str == NULL) {
        return;
    }

    UIServerPuts(ui_server_tid, str, ui_strlen(str));
}

void ui_cmd_puts(const char *str) {
    if (ui_server_tid < 0 || str == NULL) {
        return;
    }

    UIServerCmdPuts(ui_server_tid, str, ui_strlen(str));
}

void ui_init(int terminal_tid) {
    (void)terminal_tid;

    while (ui_server_tid < 0) {
        ui_server_tid = WhoIs(UI_SERVER_NAME);
        if (ui_server_tid < 0) {
            Yield();
        }
    }

    ui_puts("\033[2J\033[H\033[?25l");
    ui_puts("=== Train Control System CS652 K4 ===\r\n");
    ui_puts("Version: " __DATE__ " / " __TIME__ "\r\n");
    ui_puts("Cmds: tr|sw|rv|li|goto|findpos|demo|q\r\n");
    ui_puts("\r\n");
    ui_puts("Time: 00:00.0\r\n");
    ui_puts("Idle: 0%\r\n");
    ui_puts("\r\n");

    ui_switches();
    ui_draw_sensors(0);
    ui_draw_position();
    ui_draw_prediction_error();
    ui_draw_offroute();
    ui_prepare_cmd();

    last_idle_percent = 0;
}

void ui_prepare_cmd(void) {
    if (ui_server_tid < 0) {
        return;
    }

    UIServerPrepareCmd(ui_server_tid);
}

void ui_scroll_cmd(void) {
    if (ui_server_tid < 0) {
        return;
    }

    UIServerCmdEnter(ui_server_tid);
}

void ui_cmd_newprompt(void) {
    if (ui_server_tid < 0) {
        return;
    }

    UIServerCmdPrompt(ui_server_tid);
}

void ui_cmd_backspace(void) {
    if (ui_server_tid < 0) {
        return;
    }

    UIServerCmdBackspace(ui_server_tid);
}

void ui_cmd_putc(char c) {
    if (ui_server_tid < 0) {
        return;
    }

    UIServerCmdPutc(ui_server_tid, c);
}

void ui_update_clock(uint64_t start_us, uint64_t now) {
    uint64_t elapsed = now - start_us;
    char clock_buf[8];
    clock_render(elapsed, clock_buf);

    int has_changes = 0;
    for (int i = 0; i < 7; i++) {
        if (clock_buf[i] != last_clock_buf[i]) {
            has_changes = 1;
            break;
        }
    }

    if (has_changes) {
        char *temp_buf = buf_get_temp();
        char *p = temp_buf;

        for (int i = 0; i < 7; i++) {
            if (clock_buf[i] != last_clock_buf[i]) {
                p = buf_append(p, "\033[5;");
                p = buf_append_int(p, 7 + i);
                p = buf_append_char(p, 'H');
                p = buf_append_char(p, clock_buf[i]);
                last_clock_buf[i] = clock_buf[i];
            }
        }
        *p = '\0';
        ui_puts(temp_buf);
    }
}

void ui_update_idle(int percent) {
    if (percent < 0) {
        return;
    }
    if (percent > 100) {
        percent = 100;
    }
    if (percent == last_idle_percent) {
        return;
    }

    char *temp_buf = buf_get_temp();
    char *p = temp_buf;

    p = buf_append(p, "\033[6;1HIdle: ");
    p = buf_append_int(p, percent);
    p = buf_append(p, "%\033[K");
    *p = '\0';

    ui_puts(temp_buf);
    last_idle_percent = percent;
}

int ui_is_switches_dirty(void) {
    return ui_switches_dirty;
}

int ui_is_sensors_dirty(void) {
    return ui_sensors_dirty;
}

void ui_mark_switches_clean(void) {
    ui_switches_dirty = 0;
}

void ui_mark_sensors_clean(void) {
    ui_sensors_dirty = 0;
}

void ui_mark_switches_dirty(void) {
    ui_switches_dirty = 1;
}

void ui_mark_sensors_dirty(void) {
    ui_sensors_dirty = 1;
}

int ui_is_position_dirty(void) {
    return ui_position_dirty;
}

void ui_mark_position_clean(void) {
    ui_position_dirty = 0;
}

void ui_mark_position_dirty(void) {
    ui_position_dirty = 1;
}

void ui_mark_prediction_dirty(void) {
    ui_position_dirty = 1;
}
