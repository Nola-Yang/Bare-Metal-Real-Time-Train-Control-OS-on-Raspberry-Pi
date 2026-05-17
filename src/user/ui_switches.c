#include "ui.h"
#include "ui_priv.h"
#include "util.h"
#include "track.h"

static char *ui_append_switch_cell(char *p, char *end, int sw_num, char state) {
    if (sw_num < 10) {
        p = buf_append_cap(p, end, "  ");
    } else if (sw_num < 100) {
        p = buf_append_cap(p, end, " ");
    }
    p = buf_append_int_cap(p, end, sw_num);
    p = buf_append_char_cap(p, end, ':');

    if (state == 'S') {
        p = buf_append_cap(p, end, "\033[32m");
    } else if (state == 'C') {
        p = buf_append_cap(p, end, "\033[33m");
    } else {
        p = buf_append_cap(p, end, "\033[31m");
    }
    p = buf_append_char_cap(p, end, state);
    p = buf_append_cap(p, end, "\033[0m ");
    return p;
}

void ui_switches(void) {
    char temp_buf[512];
    char *p = temp_buf;
    char *end = temp_buf + sizeof(temp_buf) - 1;
    const switch_entry_t *switches = track_get_switch_state();

    p = buf_append_cap(p, end, "\033[7;1HSW: ");
    for (int i = 0; i < 8; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = ui_append_switch_cell(p, end, sw_num, state);
    }
    p = buf_append_cap(p, end, "\033[K");

    p = buf_append_cap(p, end, "\033[8;1H    ");
    for (int i = 8; i < 16; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = ui_append_switch_cell(p, end, sw_num, state);
    }
    p = buf_append_cap(p, end, "\033[K");

    p = buf_append_cap(p, end, "\033[9;1H    ");
    for (int i = 16; i < 22; i++) {
        int sw_num = track_index_to_switch(i);
        char state = switches[i].state;
        p = ui_append_switch_cell(p, end, sw_num, state);
    }
    p = buf_append_cap(p, end, "\033[K");

    p = buf_append_cap(p, end, "\033[10;1H\033[K");

    *p = '\0';
    ui_puts(temp_buf);
}
