#include "ui.h"
#include "util.h"
#include "track.h"
#include "terminal_server.h"
#include "text_util.h"
#include "can_data.h"
#include <stddef.h>

static int term_tid = -1;
static int ui_switches_dirty = 1;
static int ui_sensors_dirty = 1;

static char last_clock_buf[8] = "00:00.0";
static int last_idle_percent = -1;

static const int Switch_No_Display_Len = 3;
static const int Switches_Per_Line = 5;
static const int Sensor_No_Display_Len = 2;

#ifdef MEASURE
static int UI_MEASURE_LINE_POS = 60;
#endif

// UI layout constants
enum {
    UI_CMD_SCROLL_TOP = 34,
    UI_CMD_SCROLL_BOTTOM = 54,
    UI_CMD_ROW = 24,
    UI_CMD_COL = 1,
    UI_TIME_LINE_POS = 3,
    UI_IDLE_LINE_POS = 4,
    UI_TIME_CHAR_START_POS = 7,
    UI_SWITCH_LINE_POS = 13,
    UI_SENSOR_LINE_POS = 20,
    UI_CMD_SEPERATOR_LINE_POS = 33
};

#ifdef MEASURE
static int UI_CMD_LINE_POS = UI_CMD_SCROLL_TOP;
#endif

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

static char *ui_print_switch_group(char *p, const switch_entry_t *switches, uint32_t line_no, uint32_t start_ind, uint32_t end_ind) {
    p = str_buf_move_cursor(p, line_no, 1);

    for (uint32_t i = start_ind; i < end_ind; i++) {
        int sw_num = get_switch_no(i);
        char state = switches[i].state;
        p = str_buf_append_padded_uint(p, sw_num, Switch_No_Display_Len, ' ');
        p = str_buf_append_char(p, ':');
        p = str_buf_append_char(p, state);
        p = str_buf_append_char(p, ' ');
    }

    p = str_buf_clear_to_line_end(p);
    return p;
}

void ui_switches(void) {
    char *temp_buf = str_buf_get_temp();
    char *p = temp_buf;
    const switch_entry_t *switches = track_get_switch_state();

    p = str_buf_move_cursor(p, UI_SWITCH_LINE_POS, 1);
    p = str_buf_append(p, "-------- Switches --------");

    uint32_t line_no;
    uint32_t end_ind;

    for (uint32_t i = 0; i < MAX_SWITCHES; i += Switches_Per_Line) {
        line_no = i  / Switches_Per_Line;

        end_ind = i + Switches_Per_Line;
        if (end_ind > MAX_SWITCHES) {
            end_ind = MAX_SWITCHES;
        }

        p = ui_print_switch_group(p, switches, UI_SWITCH_LINE_POS + line_no + 1, i, end_ind);
    }

    *p = '\0';
    ui_puts(temp_buf);
}

void ui_draw_sensors(uint64_t start_us) {
    char *temp_buf = str_buf_get_temp();
    char *p = temp_buf;
    int head;
    sensor_entry_t *sensors = track_get_sensor_log(&head);
    sensor_entry_t *sensor_entry;
    SensorData_t *sensor_data;

    p = str_buf_move_cursor(p, UI_SENSOR_LINE_POS, 1);
    p = str_buf_append(p, "----- Recent Sensors -----");
    p = str_buf_clear_to_line_end(p);

    int count = 0;
    // Display up to 10 most recent sensor events
    for (int i = 0; i < SENSOR_LOG_SIZE && count < 10; i++) {
        int idx = (head - 1 - i + SENSOR_LOG_SIZE) % SENSOR_LOG_SIZE;

        sensor_entry = sensors + idx;
        sensor_data = &(sensor_entry->sensor_data);
        if (!sensor_data_is_valid(sensor_data)) continue;

        uint64_t elapsed_us = sensor_entry->time_us - start_us;
        uint64_t elapsed_tenths = elapsed_us / 100000;
        uint32_t minutes = (elapsed_tenths / 600);
        uint32_t seconds = (elapsed_tenths / 10) % 60;
        uint32_t tenths = elapsed_tenths % 10;

        p = str_buf_move_cursor(p, UI_SENSOR_LINE_POS + count + 1, 1);
        p = str_buf_append_char(p, ' ');

        p = str_buf_append_char(p, sensor_data->bank);
        p = str_buf_append_padded_uint(p, sensor_data->sensor_no, Sensor_No_Display_Len, '0');

        p = str_buf_append(p, sensor_data->new_state ? " enter at " : " leave at ");
        if (minutes < 10) p = str_buf_append_char(p, '0');
        p = str_buf_append_uint(p, minutes);
        p = str_buf_append_char(p, ':');
        if (seconds < 10) p = str_buf_append_char(p, '0');
        p = str_buf_append_uint(p, seconds);
        p = str_buf_append_char(p, '.');
        p = str_buf_append_uint(p, tenths);
        p = str_buf_clear_to_line_end(p);
        count++;
    }

    for (int i = count; i < 10; i++) {
        p = str_buf_move_cursor(p, UI_SENSOR_LINE_POS + count + 1, 1);
        p = str_buf_clear_to_line_end(p);
    }

    *p = '\0';
    ui_puts(temp_buf);
}

void ui_init(int terminal_tid) {
    term_tid = terminal_tid;

    char *temp_buf = str_buf_get_temp();
    char *p = temp_buf;

    ui_puts("\033[2J\033[H");

    // Draw fixed header area (rows 1-5)
    ui_puts("===== Train Control System CS652 K4 =====\r\n");
    ui_puts("Version: " __DATE__ " / " __TIME__ "\r\n");

    p = str_buf_append(p, "Time: 00:00.0\r\n");
    p = str_buf_append(p, "Idle: 0%\r\n\r\n");

    p = str_buf_append(p, "-------- Commands --------\r\n");
    p = str_buf_append(p, "- tr <t> <spd>\r\n");
    p = str_buf_append(p, "- sw <s> <S|C>\r\n");
    p = str_buf_append(p, "- rv <t>\r\n");
    p = str_buf_append(p, "- li <t> <0|1>\r\n");
    p = str_buf_append(p, "- q\r\n\r\n");
    *p = '\0';
    ui_puts(temp_buf);

    // Switches area (rows 7-9)
    ui_switches();

    // Sensors area (rows 11-21)
    ui_draw_sensors(0);

    p = temp_buf;
    p = str_buf_append(p, "\r\n");
    p = str_buf_move_cursor(p, UI_CMD_SEPERATOR_LINE_POS, 1);
    p = str_buf_append(p, "=========================================");
    *p = '\0';
    ui_puts(temp_buf);

    // Command area
    ui_prepare_cmd();

    last_idle_percent = 0;
}

static char *ui_print_cmd_prefix(char *p) {
    return str_buf_append(p, "cmd> ");
}

void ui_prepare_cmd(void) {
    if (term_tid < 0) {
        return;
    }

    char *temp_buf = str_buf_get_temp();
    char *p = temp_buf;

    // Set scroll region for command area
    p = str_buf_append(p, "\033[");
    p = str_buf_append_int(p, UI_CMD_SCROLL_TOP);
    p = str_buf_append(p, ";");
    p = str_buf_append_int(p, UI_CMD_SCROLL_BOTTOM);
    p = str_buf_append(p, "r");

    // Move cursor to start of command area and print prompt
    p = str_buf_move_cursor(p, UI_CMD_SCROLL_TOP, 1);
    p = ui_print_cmd_prefix(p);

    #ifdef MEASURE
    UI_CMD_LINE_POS++; 
    #endif

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
    if (term_tid < 0) return;

    char *temp_buf = str_buf_get_temp();
    char *p = temp_buf;

    #ifdef MEASURE
    p = str_buf_move_cursor(p, UI_CMD_LINE_POS, 1);
    UI_CMD_LINE_POS++;
    #endif

    p = str_buf_clear_line(p);
    p = ui_print_cmd_prefix(p);
    *p = '\0';
    ui_puts(temp_buf);
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

    if (!has_changes) return;

    char *temp_buf = str_buf_get_temp();
    char *p = temp_buf;

    for (int i = 0; i < 7; i++) {
        if (clock_buf[i] == last_clock_buf[i]) continue;

        p = str_buf_save_cursor(p);
        p = str_buf_move_cursor(p, UI_TIME_LINE_POS, UI_TIME_CHAR_START_POS + i);
        p = str_buf_append_char(p, clock_buf[i]);
        p = str_buf_previous_cursor(p);
        last_clock_buf[i] = clock_buf[i];
    }
    *p = '\0';
    ui_puts(temp_buf);
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

    char *temp_buf = str_buf_get_temp();
    char *p = temp_buf;

    p = str_buf_save_cursor(p);
    p = str_buf_move_cursor(p, UI_IDLE_LINE_POS, 1);

    p = str_buf_append(p, "Idle: ");
    p = str_buf_append_int(p, percent);

    p = str_buf_clear_to_line_end(p);
    p = str_buf_previous_cursor(p);

    *p = '\0';
    ui_puts(temp_buf);
    last_idle_percent = percent;
}

#ifdef MEASURE
void ui_print_sensor_time(uint64_t start, uint64_t end) {
    char *temp_buf = str_buf_get_temp();
    char *p = temp_buf;

    p = str_buf_move_cursor(p, UI_MEASURE_LINE_POS, 1);
    p = str_buf_append_uint(p, start);
    p = str_buf_append_char(p, ',');
    p = str_buf_append_uint(p, end);

    *p = '\0';
    ui_puts(temp_buf);
    UI_MEASURE_LINE_POS++;
}
#endif

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
