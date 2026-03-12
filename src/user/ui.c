#include "ui.h"
#include "util.h"
#include "track.h"
#include "train_tracking/position.h"
#include "train_tracking/traffic_manager.h"
#include "server/terminal_server.h"

static int term_tid = -1;
static int ui_switches_dirty  = 1;
static int ui_sensors_dirty   = 1;
static int ui_position_dirty  = 1;

static char last_clock_buf[8] = "00:00.0";
static int last_idle_percent = -1;

/* ---- Switch change log (Row 10) ---- */
#define SW_LOG_SIZE 8
typedef struct { int sw_num; char dir; } sw_log_entry_t;
static sw_log_entry_t sw_log[SW_LOG_SIZE];
static int sw_log_head = 0;   
static int sw_log_count = 0;  

// UI layout constants
enum {
    UI_CMD_SCROLL_TOP    = 30,
    UI_CMD_SCROLL_BOTTOM = 47,
    UI_CMD_ROW           = 30,
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

static void ui_draw_switch_log(void) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;
    p = buf_append(p, "\033[s\033[10;1HSW chg: ");
    if (sw_log_count == 0) {
        p = buf_append(p, "-");
    } else {
        int start = (sw_log_head - sw_log_count + SW_LOG_SIZE) % SW_LOG_SIZE;
        for (int i = 0; i < sw_log_count; i++) {
            int idx = (start + i) % SW_LOG_SIZE;
            p = buf_append_int(p, sw_log[idx].sw_num);
            p = buf_append_char(p, sw_log[idx].dir);
            if (i + 1 < sw_log_count) p = buf_append_char(p, ' ');
        }
    }
    p = buf_append(p, "\033[K\033[u");
    *p = '\0';
    ui_puts(temp_buf);
}

void ui_notify_switch_change(int sw_num, char dir) {
    sw_log[sw_log_head].sw_num = sw_num;
    sw_log[sw_log_head].dir    = dir;
    sw_log_head = (sw_log_head + 1) % SW_LOG_SIZE;
    if (sw_log_count < SW_LOG_SIZE) sw_log_count++;
    ui_draw_switch_log();
}

static char *ui_append_switch_cell(char *p, int sw_num, char state) {
    if (sw_num < 10) {
        p = buf_append(p, "  ");
    } else if (sw_num < 100) {
        p = buf_append(p, " ");
    }
    p = buf_append_int(p, sw_num);
    p = buf_append_char(p, ':');

    if (state == 'S') {
        p = buf_append(p, "\033[32m");
    } else if (state == 'C') {
        p = buf_append(p, "\033[33m");
    } else {
        p = buf_append(p, "\033[31m");
    }
    p = buf_append_char(p, state);
    p = buf_append(p, "\033[0m ");
    return p;
}

void ui_switches(void) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;
    const switch_entry_t *switches = track_get_switch_state();

    // Row 7: Switches 1-8
    p = buf_append(p, "\033[7;1HSW: ");
    for (int i = 0; i < 8; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = ui_append_switch_cell(p, sw_num, state);
    }
    p = buf_append(p, "\033[K");

    // Row 8: Switches 9-16
    p = buf_append(p, "\033[8;1H    ");
    for (int i = 8; i < 16; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = ui_append_switch_cell(p, sw_num, state);
    }
    p = buf_append(p, "\033[K");

    // Row 9: Switches 17-18 and 153-156
    p = buf_append(p, "\033[9;1H    ");
    for (int i = 16; i < 22; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = ui_append_switch_cell(p, sw_num, state);
    }
    p = buf_append(p, "\033[K");

    *p = '\0';
    ui_puts(temp_buf);
}

static const char *ui_state_short(train_route_state_t st) {
    switch (st) {
    case TRAIN_STATE_UNKNOWN:           return "UNK";
    case TRAIN_STATE_KNOWN:             return "KNW";
    case TRAIN_STATE_STOPPING_TR:       return "STR";
    case TRAIN_STATE_LOOP_FIND_DIR:     return "DIR";
    case TRAIN_STATE_LOOP_STABILIZE:    return "STB";
    case TRAIN_STATE_ON_ROUTE:          return "RTE";
    case TRAIN_STATE_STOPPING:          return "STP";
    case TRAIN_STATE_STOPPED:           return "SPD";
    case TRAIN_STATE_RECOVERY_STOPPING: return "REC";
    case TRAIN_STATE_ENTER_LOOP:        return "ENT";
    case TRAIN_STATE_STOPPING_GOTO:     return "SGT";
    case TRAIN_STATE_DEAD_TRACK:        return "DED";
    case TRAIN_STATE_WAIT_RESOURCE:     return "WAI";
    default:                            return "???";
    }
}

static int ui_train_in_list(const int *arr, int n, int train_num) {
    for (int i = 0; i < n; i++) {
        if (arr[i] == train_num) return 1;
    }
    return 0;
}

static void ui_sort_small(int *arr, int n) {
    for (int i = 1; i < n; i++) {
        int v = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > v) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = v;
    }
}

static void ui_build_train_rows(int out[5]) {
    static const int PREF[6] = {13, 14, 15, 17, 18, 55};
    int active[MAX_POS_TRAINS];
    int active_n = 0;

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = pos_get_by_index(i);
        if (!pos || pos->train_num < 0) continue;
        if (ui_train_in_list(active, active_n, pos->train_num)) continue;
        if (active_n < MAX_POS_TRAINS) active[active_n++] = pos->train_num;
    }
    ui_sort_small(active, active_n);

    int out_n = 0;
    for (int i = 0; i < active_n && out_n < 5; i++) {
        out[out_n++] = active[i];
    }
    for (int i = 0; i < 6 && out_n < 5; i++) {
        if (!ui_train_in_list(out, out_n, PREF[i])) {
            out[out_n++] = PREF[i];
        }
    }
    while (out_n < 5) out[out_n++] = -1;
}

/* ---- Multi-train position table (Rows 22-29) ---- */
void ui_draw_position(void) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;
    int trains[5];
    ui_build_train_rows(trains);

    p = buf_append(p, "\033[22;1HTR CUR  NEXT DEST     STATE REM   Q\033[K");

    for (int i = 0; i < 5; i++) {
        int row = 23 + i;
        int train = trains[i];
        train_pos_t *pos = (train > 0) ? pos_get(train) : NULL;

        p = buf_append(p, "\033[");
        p = buf_append_int(p, row);
        p = buf_append(p, ";1H");
        p = buf_append_int(p, train);
        p = buf_append(p, " ");

        if (!pos || pos->train_num < 0) {
            p = buf_append(p, "-    -    -        ---   -     -");
            p = buf_append(p, "\033[K");
            continue;
        }

        p = buf_append(p, (pos->cur_sensor && pos->cur_sensor->name) ? pos->cur_sensor->name : "-");
        p = buf_append(p, " ");
        p = buf_append(p, (pos->pred_next_sensor && pos->pred_next_sensor->name) ? pos->pred_next_sensor->name : "-");
        p = buf_append(p, " ");
        if (pos->target_sensor && pos->target_sensor->name) {
            p = buf_append(p, pos->target_sensor->name);
            if (pos->midrev_active && pos->midrev_final_target &&
                pos->midrev_final_target->name) {
                p = buf_append(p, ">");
                p = buf_append(p, pos->midrev_final_target->name);
            }
        } else {
            p = buf_append(p, "-");
        }
        p = buf_append(p, " ");
        p = buf_append(p, ui_state_short(pos->route_state));
        p = buf_append(p, " ");
        if (pos->route_state == TRAIN_STATE_ON_ROUTE ||
            pos->route_state == TRAIN_STATE_STOPPING) {
            p = buf_append_int(p, (int)pos->dist_to_target_mm);
            p = buf_append(p, "mm");
        } else {
            p = buf_append(p, "-");
        }
        p = buf_append(p, " ");
        if (pos->queued_valid && pos->queued_target && pos->queued_target->name) {
            p = buf_append(p, pos->queued_target->name);
        } else {
            p = buf_append(p, "-");
        }
        p = buf_append(p, "\033[K");
    }

    p = buf_append(p, "\033[28;1HWarn: ");
    int warned = 0;
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = pos_get_by_index(i);
        if (!pos || pos->train_num < 0) continue;
        if (pos->route_state == TRAIN_STATE_DEAD_TRACK) {
            p = buf_append(p, "tr");
            p = buf_append_int(p, pos->train_num);
            p = buf_append(p, " dead-track");
            warned = 1;
            break;
        }
        if (pos->offroute_valid &&
            pos->offroute_expected_sensor && pos->offroute_expected_sensor->name) {
            p = buf_append(p, "tr");
            p = buf_append_int(p, pos->train_num);
            p = buf_append(p, " off-route exp=");
            p = buf_append(p, pos->offroute_expected_sensor->name);
            warned = 1;
            break;
        }
    }
    if (!warned) {
        int spurious = 0, ambiguous = 0;
        traffic_get_sensor_stats(&spurious, &ambiguous);
        if (ambiguous > 0 || spurious > 0) {
            p = buf_append(p, "amb=");
            p = buf_append_int(p, ambiguous);
            p = buf_append(p, " spur=");
            p = buf_append_int(p, spurious);
            warned = 1;
        }
    }
    if (!warned) p = buf_append(p, "-");
    p = buf_append(p, "\033[K");

    p = buf_append(p, "\033[29;1H======================================================================\033[K");
    *p = '\0';
    ui_puts(temp_buf);
}

/* Legacy API retained; merged into ui_draw_position table. */
void ui_draw_prediction_error(void) {
    return;
}

/* Legacy API retained; merged into ui_draw_position table. */
void ui_draw_offroute(void) {
    return;
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
    ui_puts("Cmds: tr|sw|rv|li|goto <t> <node> [+mm]|q\r\n");
    ui_puts("\r\n");
    ui_puts("Time: 00:00.0\r\n");
    ui_puts("Idle: 0%\r\n");
    ui_puts("\r\n");

    // Switches area (rows 7-9)
    ui_switches();
    // Row 10: switch change log
    ui_draw_switch_log();

    // Sensors area (rows 11-21)
    ui_draw_sensors(0);

    // Position / prediction / off-route rows (22, 23, 24)
    ui_draw_position();
    ui_draw_prediction_error();
    ui_draw_offroute();
    ui_puts("\033[29;1H======================================================================\r\n");

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
