#include "ui.h"
#include "util.h"
#include "track.h"
#include "position.h"
#include "terminal_server.h"

static int term_tid = -1;
static int ui_switches_dirty  = 1;
static int ui_sensors_dirty   = 1;
static int ui_position_dirty  = 1;

static char last_clock_buf[8] = "00:00.0";
static int last_idle_percent = -1;

// UI layout constants
enum {
    UI_CMD_SCROLL_TOP    = 27,   
    UI_CMD_SCROLL_BOTTOM = 47,
    UI_CMD_ROW           = 27,
    UI_CMD_COL           = 1
};

// Helper: send string via terminal server
void ui_puts(const char *str) {
    if (term_tid < 0 || str == NULL) {
        return;
    }

    const char *p = str;
    while (*p) {
        const char *chunk = p;
        int len = 0;
        while (*p && len < TERM_MAX_STR_LEN) {
            ++p;
            ++len;
        }
        if (len > 0) {
            Puts(term_tid, TERM_CHANNEL_CONSOLE, chunk, len);
        }
    }
}

void ui_switches(void) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;
    const switch_entry_t *switches = track_get_switch_state();

    // Row 7: Switches 1-9
    p = buf_append(p, "\033[7;1HSW: ");
    for (int i = 0; i < 9; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = buf_append_int(p, sw_num);
        p = buf_append_char(p, ':');
        p = buf_append_char(p, state);
        p = buf_append_char(p, ' ');
    }
    p = buf_append(p, "\033[K");

    // Row 8: Switches 10-18
    p = buf_append(p, "\033[8;1H    ");
    for (int i = 9; i < 18; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = buf_append_int(p, sw_num);
        p = buf_append_char(p, ':');
        p = buf_append_char(p, state);
        p = buf_append_char(p, ' ');
    }
    p = buf_append(p, "\033[K");

    // Row 9: Switches 153-156
    p = buf_append(p, "\033[9;1H    ");
    for (int i = 18; i < 22; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = buf_append_int(p, sw_num);
        p = buf_append_char(p, ':');
        p = buf_append_char(p, state);
        p = buf_append_char(p, ' ');
    }
    p = buf_append(p, "\033[K");

    *p = '\0';
    ui_puts(temp_buf);
}

/* ---- Position / route info (Row 22) ---- */
void ui_draw_position(void) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;

    p = buf_append(p, "\033[22;1H");

    /* Find first active train */
    int found = 0;
    for (int t = 0; t < MAX_POS_TRAINS; t++) {
        train_pos_t *pos = pos_get_by_index(t);
        if (!pos || pos->train_num < 0) continue;

        p = buf_append(p, "Pos: tr=");
        p = buf_append_int(p, pos->train_num);
        p = buf_append(p, " @ ");
        if (pos->cur_sensor && pos->cur_sensor->name) {
            p = buf_append(p, pos->cur_sensor->name);
        } else {
            p = buf_append(p, "?");
        }
        p = buf_append(p, " spd=");
        p = buf_append_int(p, pos->user_speed);
        p = buf_append(p, " st=");
        switch (pos->route_state) {
        case TRAIN_STATE_UNKNOWN:           p = buf_append(p, "UNK");   break;
        case TRAIN_STATE_KNOWN:             p = buf_append(p, "KNOW");  break;
        case TRAIN_STATE_STOPPING_TR:       p = buf_append(p, "STR");   break;
        case TRAIN_STATE_LOOP_FIND_DIR:     p = buf_append(p, "DIR?");  break;
        case TRAIN_STATE_LOOP_STABILIZE:    p = buf_append(p, "STAB");  break;
        case TRAIN_STATE_ON_ROUTE:          p = buf_append(p, "ROUTE"); break;
        case TRAIN_STATE_STOPPING:          p = buf_append(p, "STOP");  break;
        case TRAIN_STATE_STOPPED:           p = buf_append(p, "STPD");  break;
        case TRAIN_STATE_RECOVERY_STOPPING: p = buf_append(p, "REC");   break;
        case TRAIN_STATE_ENTER_LOOP:        p = buf_append(p, "ENT");   break;
        case TRAIN_STATE_STOPPING_GOTO:     p = buf_append(p, "SGT");   break;
        default:                            p = buf_append(p, "???");   break;
        }
        if (pos->target_sensor && pos->target_sensor->name) {
            p = buf_append(p, " tgt=");
            p = buf_append(p, pos->target_sensor->name);
        }
        if (pos->route_state == TRAIN_STATE_ON_ROUTE &&
            pos->target_sensor && pos->target_sensor->name) {
            p = buf_append(p, " rem=");
            p = buf_append_int(p, (int)pos->dist_to_target_mm);
            p = buf_append(p, "mm");
        }
        found = 1;
        break;
    }
    if (!found) {
        p = buf_append(p, "Pos: no train tracked");
    }
    p = buf_append(p, "\033[K");
    *p = '\0';
    ui_puts(temp_buf);
}

/* ---- Prediction error (Row 23) ---- */
void ui_draw_prediction_error(void) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;

    p = buf_append(p, "\033[23;1H");

    for (int t = 0; t < MAX_POS_TRAINS; t++) {
        train_pos_t *pos = pos_get_by_index(t);
        if (!pos || pos->train_num < 0) continue;

        p = buf_append(p, "Pred: next=");
        if (pos->pred_next_sensor && pos->pred_next_sensor->name) {
            p = buf_append(p, pos->pred_next_sensor->name);
        } else {
            p = buf_append(p, "?");
        }
        p = buf_append(p, " err_t=");
        int err_ms = (int)(pos->last_time_err_us / 1000);
        if (err_ms >= 0) p = buf_append_char(p, '+');
        p = buf_append_int(p, err_ms);
        p = buf_append(p, "ms err_d=");
        if (pos->last_dist_err_mm >= 0) p = buf_append_char(p, '+');
        p = buf_append_int(p, (int)pos->last_dist_err_mm);
        p = buf_append(p, "mm");
        break;
    }
    p = buf_append(p, "\033[K");
    *p = '\0';
    ui_puts(temp_buf);
}

void ui_draw_sensors(uint64_t start_us) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;
    int head;
    const sensor_entry_t *sensors = track_get_sensor_log(&head);

    p = buf_append(p, "\033[11;1HRecent Sensors:\033[K");

    int count = 0;
    // Display up to 10 most recent sensor events
    for (int i = 0; i < SENSOR_LOG_SIZE && count < 10; i++) {
        int idx = (head - 1 - i + SENSOR_LOG_SIZE) % SENSOR_LOG_SIZE;
        if (sensors[idx].sensor_id != 0) {
            uint64_t elapsed_us = sensors[idx].time_us - start_us;
            uint64_t elapsed_tenths = elapsed_us / 100000;
            uint32_t minutes = (elapsed_tenths / 600);
            uint32_t seconds = (elapsed_tenths / 10) % 60;
            uint32_t tenths = elapsed_tenths % 10;

            // Formula: sensor_id = (bank - 'A') * 16 + (number - 1) + 1
            // Reverse: bank = (sensor_id - 1) / 16, number = (sensor_id - 1) % 16 + 1
            int bank = (sensors[idx].sensor_id - 1) / 16;
            int number = (sensors[idx].sensor_id - 1) % 16 + 1;

            p = buf_append(p, "\033[");
            p = buf_append_int(p, 12 + count);
            p = buf_append(p, ";1H  ");
            p = buf_append_char(p, 'A' + bank);
            p = buf_append_uint(p, number);
            p = buf_append(p, sensors[idx].state ? " enter at " : " leave at ");
            if (minutes < 10) p = buf_append_char(p, '0');
            p = buf_append_uint(p, minutes);
            p = buf_append_char(p, ':');
            if (seconds < 10) p = buf_append_char(p, '0');
            p = buf_append_uint(p, seconds);
            p = buf_append_char(p, '.');
            p = buf_append_uint(p, tenths);
            p = buf_append(p, "\033[K");
            count++;
        }
    }

    for (int i = count; i < 10; i++) {
        p = buf_append(p, "\033[");
        p = buf_append_int(p, 12 + i);
        p = buf_append(p, ";1H\033[K");
    }

    *p = '\0';
    ui_puts(temp_buf);
}

void ui_init(int terminal_tid) {
    term_tid = terminal_tid;

    ui_puts("\033[2J\033[H");

    // Draw fixed header area (rows 1-5)
    ui_puts("=== Train Control System CS652 K4 ===\r\n");
    ui_puts("Version: " __DATE__ " / " __TIME__ "\r\n");
    ui_puts("Cmds: tr|sw|rv|li|goto <t> <s> [+mm]|q\r\n");
    ui_puts("\r\n");
    ui_puts("Time: 00:00.0\r\n");
    ui_puts("Idle: 0%\r\n");
    ui_puts("\r\n");

    // Switches area (rows 7-9)
    ui_switches();
    ui_puts("\r\n");

    // Sensors area (rows 11-21)
    ui_draw_sensors(0);

    // Position / prediction / reliability rows (22, 23, 24)
    ui_draw_position();
    ui_puts("\r\n");
    ui_draw_prediction_error();
    ui_puts("\r\n");
    ui_puts("======================================================================\r\n");

    // Command area (starts at row 27)
    ui_prepare_cmd();

    last_idle_percent = 0;
}

void ui_prepare_cmd(void) {
    if (term_tid < 0) {
        return;
    }

    char *temp_buf = buf_get_temp();
    char *p = temp_buf;

    // Set scroll region for command area
    p = buf_append(p, "\033[");
    p = buf_append_int(p, UI_CMD_SCROLL_TOP);
    p = buf_append(p, ";");
    p = buf_append_int(p, UI_CMD_SCROLL_BOTTOM);
    p = buf_append(p, "r");

    // Move cursor to start of command area and print prompt
    p = buf_append(p, "\033[");
    p = buf_append_int(p, UI_CMD_SCROLL_TOP);
    p = buf_append(p, ";1Hcmd> ");

    *p = '\0';
    ui_puts(temp_buf);
}

void ui_scroll_cmd(void) {
    if (term_tid < 0) {
        return;
    }
    ui_puts("\r\n");
}

void ui_cmd_newprompt(void) {
    if (term_tid < 0) {
        return;
    }
    ui_puts("\r\033[2Kcmd> ");
}

void ui_cmd_backspace(void) {
    ui_puts("\b \b");
}

void ui_cmd_putc(char c) {
    Putc(term_tid, TERM_CHANNEL_CONSOLE, c);
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
                p = buf_append(p, "\033[s\033[5;");
                p = buf_append_int(p, 7 + i);
                p = buf_append_char(p, 'H');
                p = buf_append_char(p, clock_buf[i]);
                p = buf_append(p, "\033[u");
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

    p = buf_append(p, "\033[s\033[6;1HIdle: ");
    p = buf_append_int(p, percent);
    p = buf_append(p, "%\033[K\033[u");
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
    ui_position_dirty = 1;  /* prediction uses same dirty flag as position */
}
