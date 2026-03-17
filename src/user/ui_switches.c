#include "ui.h"
#include "ui_priv.h"
#include "util.h"
#include "track.h"

/* ---- Switch change log (Row 10) ---- */
#define SW_LOG_SIZE 8
typedef struct { int sw_num; char dir; } sw_log_entry_t;
static sw_log_entry_t sw_log[SW_LOG_SIZE];
static int sw_log_head = 0;
static int sw_log_count = 0;

void ui_draw_switch_log(void) {
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

    p = buf_append(p, "\033[7;1HSW: ");
    for (int i = 0; i < 8; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = ui_append_switch_cell(p, sw_num, state);
    }
    p = buf_append(p, "\033[K");

    p = buf_append(p, "\033[8;1H    ");
    for (int i = 8; i < 16; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = ui_append_switch_cell(p, sw_num, state);
    }
    p = buf_append(p, "\033[K");

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
