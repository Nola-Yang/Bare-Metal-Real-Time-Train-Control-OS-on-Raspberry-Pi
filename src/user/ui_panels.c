#include "ui.h"
#include "ui_priv.h"
#include "util.h"
#include "track.h"
#include "demo_manager.h"
#include "timer.h"
#include "train_tracking/position.h"
#include "train_tracking/traffic_manager.h"

static const char *ui_state_long(train_route_state_t st) {
    switch (st) {
    case TRAIN_STATE_UNKNOWN:           return "UNKNOWN";
    case TRAIN_STATE_KNOWN:             return "KNOWN";
    case TRAIN_STATE_STOPPING_TR:       return "STOPPING_TR";
    case TRAIN_STATE_FIND_POS:          return "FIND_POS";
    case TRAIN_STATE_ON_ROUTE:          return "ON_ROUTE";
    case TRAIN_STATE_STOPPING:          return "STOPPING";
    case TRAIN_STATE_STOPPED:           return "STOPPED";
    case TRAIN_STATE_RECOVERY_STOPPING: return "RECOVERY_STOPPING";
    case TRAIN_STATE_STOPPING_GOTO:     return "STOPPING_GOTO";
    case TRAIN_STATE_DEAD_TRACK:        return "DEAD_TRACK";
    case TRAIN_STATE_WAIT_RESOURCE:     return "WAIT_RESOURCE";
    default:                            return "UNKNOWN_STATE";
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

static int ui_limited_append_char(char *dst, int pos, int cap, char c) {
    if (cap > 0 && pos + 1 < cap) dst[pos++] = c;
    return pos;
}

static int ui_limited_append_str(char *dst, int pos, int cap, const char *src) {
    if (!src || !src[0]) src = "-";
    while (*src && pos + 1 < cap) {
        dst[pos++] = *src++;
    }
    return pos;
}

static int ui_limited_append_uint(char *dst, int pos, int cap, unsigned int value) {
    char num[16];
    ui2a(value, 10, num);
    return ui_limited_append_str(dst, pos, cap, num);
}

static int ui_limited_append_int(char *dst, int pos, int cap, int value) {
    char num[16];
    i2a(value, num);
    return ui_limited_append_str(dst, pos, cap, num);
}

static void ui_limited_finish(char *dst, int pos, int cap) {
    if (cap <= 0) return;
    if (pos >= cap) pos = cap - 1;
    dst[pos] = '\0';
}

static char *ui_append_field(char *p, const char *text, int width) {
    int len = 0;
    if (!text) text = "-";
    while (text[len] && len < width) {
        *p++ = text[len++];
    }
    while (len < width) {
        *p++ = ' ';
        len++;
    }
    return p;
}

static void ui_build_dest_text(const train_pos_t *pos, char *out, int cap) {
    int n = 0;
    if (!pos || !pos->target_sensor || !pos->target_sensor->name) {
        n = ui_limited_append_char(out, n, cap, '-');
    } else {
        n = ui_limited_append_str(out, n, cap, pos->target_sensor->name);
        if (pos->midrev.active && pos->midrev.final_target &&
            pos->midrev.final_target->name) {
            n = ui_limited_append_char(out, n, cap, '>');
            n = ui_limited_append_str(out, n, cap, pos->midrev.final_target->name);
        }
    }
    ui_limited_finish(out, n, cap);
}

static void ui_build_rem_text(const train_pos_t *pos, char *out, int cap) {
    int n = 0;
    if (pos &&
        (pos->route_state == TRAIN_STATE_ON_ROUTE ||
         pos->route_state == TRAIN_STATE_STOPPING)) {
        n = ui_limited_append_int(out, n, cap, (int)pos->dist_to_target_mm);
        n = ui_limited_append_str(out, n, cap, "mm");
    } else {
        n = ui_limited_append_char(out, n, cap, '-');
    }
    ui_limited_finish(out, n, cap);
}

static void ui_build_sensor_id_text(uint16_t sensor_id, char *out, int cap) {
    int n = 0;
    if (sensor_id == 0) {
        n = ui_limited_append_char(out, n, cap, '-');
    } else {
        int bank = (sensor_id - 1) / 16;
        int number = (sensor_id - 1) % 16 + 1;
        n = ui_limited_append_char(out, n, cap, (char)('A' + bank));
        n = ui_limited_append_uint(out, n, cap, (unsigned int)number);
    }
    ui_limited_finish(out, n, cap);
}

static char *ui_move_to_row(char *p, int row) {
    p = buf_append(p, "\033[");
    p = buf_append_int(p, row);
    p = buf_append(p, ";1H");
    return p;
}

static char *ui_append_section_bar(char *p, int row, const char *title) {
    p = ui_move_to_row(p, row);
    p = buf_append(p, "================ ");
    p = buf_append(p, title);
    p = buf_append(p, " ===============================================================\033[K");
    return p;
}

static char *ui_append_blank_row(char *p, int row) {
    p = ui_move_to_row(p, row);
    p = buf_append(p, "\033[K");
    return p;
}

static char *ui_append_position_row(char *p, int row, int train, const train_pos_t *pos) {
    char dest_buf[64];
    char rem_buf[24];
    const char *cur_name = (pos && pos->cur_sensor && pos->cur_sensor->name)
                           ? pos->cur_sensor->name : "-";
    const char *next_name = (pos && pos->pred.next_sensor && pos->pred.next_sensor->name)
                            ? pos->pred.next_sensor->name : "-";
    const char *queued_name = (pos && pos->queued_valid && pos->queued_target &&
                               pos->queued_target->name)
                              ? pos->queued_target->name : "-";

    ui_build_dest_text(pos, dest_buf, sizeof(dest_buf));
    ui_build_rem_text(pos, rem_buf, sizeof(rem_buf));

    p = ui_move_to_row(p, row);
    if (train > 0) p = buf_append_int(p, train);
    else p = buf_append(p, "-");
    p = ui_append_field(p, "", (train > 0) ? 1 : 2);
    p = ui_append_field(p, cur_name, 8);
    p = ui_append_field(p, next_name, 8);
    p = ui_append_field(p, dest_buf, 24);
    p = ui_append_field(p, pos ? ui_state_long(pos->route_state) : "-", 20);
    p = ui_append_field(p, rem_buf, 12);
    p = ui_append_field(p, queued_name, 24);
    p = buf_append(p, "\033[K");
    return p;
}

static void ui_build_demo_sensor_line(uint64_t now_us, char *out, int cap) {
    demo_ui_summary_t summary;
    char amb_buf[16];
    char spur_buf[16];
    int n = 0;

    demo_get_ui_summary(&summary, now_us);
    ui_build_sensor_id_text(summary.sensor_stats.last_ambiguous_sensor_id, amb_buf, sizeof(amb_buf));
    ui_build_sensor_id_text(summary.sensor_stats.last_spurious_sensor_id, spur_buf, sizeof(spur_buf));

    n = ui_limited_append_str(out, n, cap, "Sensor Stats: ambiguous=");
    n = ui_limited_append_int(out, n, cap, summary.sensor_stats.ambiguous_count);
    n = ui_limited_append_str(out, n, cap, " last=");
    n = ui_limited_append_str(out, n, cap, amb_buf);
    n = ui_limited_append_str(out, n, cap, "  spurious=");
    n = ui_limited_append_int(out, n, cap, summary.sensor_stats.spurious_count);
    n = ui_limited_append_str(out, n, cap, " last=");
    n = ui_limited_append_str(out, n, cap, spur_buf);
    ui_limited_finish(out, n, cap);
}

static void ui_build_demo_tuning_line(uint64_t now_us, char *out, int cap) {
    demo_ui_summary_t summary;
    int n = 0;

    demo_get_ui_summary(&summary, now_us);
    n = ui_limited_append_str(out, n, cap, "Tuning: gold_min_trip_mm=");
    n = ui_limited_append_int(out, n, cap, summary.gold_min_trip_mm);
    ui_limited_finish(out, n, cap);
}

static int ui_text_len(const char *s) {
    int n = 0;
    while (s && s[n]) n++;
    return n;
}

static int ui_contains_reservation_node(const uint16_t *nodes, int count, uint16_t idx) {
    if (!nodes) return 0;
    for (int i = 0; i < count; i++) {
        if (nodes[i] == idx) return 1;
    }
    return 0;
}

static int ui_append_reservation_node(uint16_t *out, int count, int max_nodes, uint16_t idx) {
    if (ui_contains_reservation_node(out, count, idx)) return count;
    if (out && count < max_nodes) out[count] = idx;
    return count + 1;
}

static int ui_node_index(const track_node *node) {
    if (!node) return -1;
    {
        int idx = (int)(node - g_track);
        return (0 <= idx && idx < TRACK_MAX) ? idx : -1;
    }
}

static int ui_is_sensor_node_idx(int idx) {
    return 0 <= idx && idx < TRACK_MAX && g_track[idx].type == NODE_SENSOR;
}

static int ui_append_reserved_path_nodes(
    int train_num,
    const uint16_t *path,
    int path_count,
    uint16_t *out,
    int count,
    int max_nodes
) {
    if (!path || path_count <= 0) return count;

    for (int i = 0; i < path_count; i++) {
        int idx = (int)path[i];
        if (idx < 0 || idx >= TRACK_MAX) continue;
        if (!ui_is_sensor_node_idx(idx)) continue;
        if (!traffic_is_reserved_by(&g_track[idx], train_num)) continue;
        count = ui_append_reservation_node(out, count, max_nodes, (uint16_t)idx);
    }
    return count;
}

static int ui_collect_reservation_display_nodes(int train_num, uint16_t *out, int max_nodes) {
    int count = 0;
    train_pos_t *pos = pos_get(train_num);

    if (pos) {
        count = ui_append_reserved_path_nodes(train_num, pos->route_path,
                                              pos->route_path_count,
                                              out, count, max_nodes);
        count = ui_append_reserved_path_nodes(train_num, pos->midrev.path2,
                                              pos->midrev.path2_count,
                                              out, count, max_nodes);
    }
    if (count > 0) return count;

    {
        uint16_t reserved[TRACK_MAX];
        int total_reserved = traffic_get_reserved_nodes(train_num, reserved, TRACK_MAX);

        for (int i = 0; i < total_reserved; i++) {
            int idx = (int)reserved[i];
            int rev_idx;

            if (idx < 0 || idx >= TRACK_MAX) continue;
            if (!ui_is_sensor_node_idx(idx)) continue;

            rev_idx = ui_node_index(g_track[idx].reverse);
            if (rev_idx >= 0 && ui_contains_reservation_node(out, count, (uint16_t)rev_idx)) {
                continue;
            }

            count = ui_append_reservation_node(out, count, max_nodes, (uint16_t)idx);
        }
    }
    return count;
}

static void ui_build_reservation_token(int idx, char *out, int cap) {
    int n = 0;
    const char *name = (idx >= 0 && idx < TRACK_MAX && g_track[idx].name)
                       ? g_track[idx].name : "?";
    n = ui_limited_append_str(out, n, cap, name);
    ui_limited_finish(out, n, cap);
}

static void ui_init_text_line(char *line, int cap) {
    if (cap <= 0) return;
    line[0] = '\0';
}

static void ui_build_reservation_block_lines(
    int train_num,
    char lines[UI_RESERVATION_BLOCK_ROWS][UI_RESERVATION_COL_WIDTH + 1]
) {
    uint16_t nodes[TRACK_MAX];
    int total = ui_collect_reservation_display_nodes(train_num, nodes, TRACK_MAX);
    int fill = total;
    int row = 1;
    int line_len = 2;
    int line_has_tokens = 0;
    int overflow = 0;

    for (int i = 0; i < UI_RESERVATION_BLOCK_ROWS; i++) {
        ui_init_text_line(lines[i], UI_RESERVATION_COL_WIDTH + 1);
    }

    {
        int n = 0;
        n = ui_limited_append_str(lines[0], n, UI_RESERVATION_COL_WIDTH + 1, "tr");
        n = ui_limited_append_int(lines[0], n, UI_RESERVATION_COL_WIDTH + 1, train_num);
        n = ui_limited_append_str(lines[0], n, UI_RESERVATION_COL_WIDTH + 1, " count=");
        n = ui_limited_append_int(lines[0], n, UI_RESERVATION_COL_WIDTH + 1, total);
        ui_limited_finish(lines[0], n, UI_RESERVATION_COL_WIDTH + 1);
    }

    if (fill <= 0) {
        lines[1][0] = ' ';
        lines[1][1] = ' ';
        lines[1][2] = '-';
        lines[1][3] = '\0';
        return;
    }

    lines[1][0] = ' ';
    lines[1][1] = ' ';
    lines[1][2] = '\0';

    for (int i = 0; i < fill; i++) {
        char token[24];
        int token_len;
        int needed;
        ui_build_reservation_token((int)nodes[i], token, sizeof(token));
        token_len = ui_text_len(token);
        needed = token_len + (line_has_tokens ? 2 : 0);

        if (line_has_tokens && line_len + needed > UI_RESERVATION_LINE_CHARS) {
            row++;
            if (row >= UI_RESERVATION_BLOCK_ROWS) {
                overflow = 1;
                break;
            }
            lines[row][0] = ' ';
            lines[row][1] = ' ';
            lines[row][2] = '\0';
            line_len = 2;
            line_has_tokens = 0;
        }

        if (line_has_tokens) {
            lines[row][line_len++] = ',';
            lines[row][line_len++] = ' ';
        }
        for (int j = 0; token[j] && line_len + 1 < UI_RESERVATION_COL_WIDTH; j++) {
            lines[row][line_len++] = token[j];
        }
        lines[row][line_len] = '\0';
        line_has_tokens = 1;
    }

    if (overflow) {
        int ellipsis_row = UI_RESERVATION_BLOCK_ROWS - 1;
        int n = ui_text_len(lines[ellipsis_row]);
        if (n == 0) {
            lines[ellipsis_row][0] = ' ';
            lines[ellipsis_row][1] = ' ';
            lines[ellipsis_row][2] = '.';
            lines[ellipsis_row][3] = '.';
            lines[ellipsis_row][4] = '.';
            lines[ellipsis_row][5] = '\0';
        } else if (n + 4 < UI_RESERVATION_COL_WIDTH) {
            lines[ellipsis_row][n++] = ' ';
            lines[ellipsis_row][n++] = '.';
            lines[ellipsis_row][n++] = '.';
            lines[ellipsis_row][n++] = '.';
            lines[ellipsis_row][n] = '\0';
        }
    }
}

void ui_draw_position(void) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;
    int trains[5];
    uint64_t now_us = read_timer();
    char line_buf[160];
    static const int RESERVATION_TRAINS[UI_RESERVATION_TRAIN_COUNT] = {13, 14, 15, 17, 18, 55};
    char reservation_blocks[UI_RESERVATION_TRAIN_COUNT]
                           [UI_RESERVATION_BLOCK_ROWS]
                           [UI_RESERVATION_COL_WIDTH + 1];

    ui_build_train_rows(trains);

    p = ui_append_section_bar(p, 22, "Position");
    p = ui_move_to_row(p, 23);
    p = ui_append_field(p, "TR", 3);
    p = ui_append_field(p, "CUR", 8);
    p = ui_append_field(p, "NEXT", 8);
    p = ui_append_field(p, "TARGET", 24);
    p = ui_append_field(p, "STATE", 20);
    p = ui_append_field(p, "REMAIN", 12);
    p = buf_append(p, "QUEUED\033[K");

    for (int i = 0; i < 5; i++) {
        int train = trains[i];
        train_pos_t *pos = (train > 0) ? pos_get(train) : NULL;
        p = ui_append_position_row(p, 24 + i, train, pos);
    }

    p = ui_move_to_row(p, 29);
    p = buf_append(p, "Warn: ");
    {
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
            int spurious = 0;
            int ambiguous = 0;
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
    }
    p = buf_append(p, "\033[K");

    p = ui_append_section_bar(p, 30, "Demo Status");
    ui_build_demo_sensor_line(now_us, line_buf, sizeof(line_buf));
    p = ui_move_to_row(p, 31);
    p = buf_append(p, line_buf);
    p = buf_append(p, "\033[K");

    ui_build_demo_tuning_line(now_us, line_buf, sizeof(line_buf));
    p = ui_move_to_row(p, 32);
    p = buf_append(p, line_buf);
    p = buf_append(p, "\033[K");

    p = ui_append_blank_row(p, 33);

    p = ui_append_section_bar(p, 34, "Reservations");
    for (int i = 0; i < UI_RESERVATION_TRAIN_COUNT; i++) {
        ui_build_reservation_block_lines(RESERVATION_TRAINS[i], reservation_blocks[i]);
    }
    for (int group = 0; group < UI_RESERVATION_GROUP_ROWS; group++) {
        for (int line = 0; line < UI_RESERVATION_BLOCK_ROWS; line++) {
            int row = UI_RESERVATION_START_ROW + group * UI_RESERVATION_BLOCK_ROWS + line;
            int left_idx = group * UI_RESERVATION_BLOCK_COLS;
            int right_idx = left_idx + 1;
            p = ui_move_to_row(p, row);
            p = ui_append_field(p, reservation_blocks[left_idx][line], UI_RESERVATION_COL_WIDTH);
            p = ui_append_field(p, "", UI_RESERVATION_COL_GAP);
            p = ui_append_field(p, reservation_blocks[right_idx][line], UI_RESERVATION_COL_WIDTH);
            p = buf_append(p, "\033[K");
        }
    }

    p = ui_move_to_row(p, UI_CMD_SCROLL_TOP - 1);
    p = buf_append(p, "===================================================================================================\033[K");
    *p = '\0';
    ui_puts(temp_buf);
}

void ui_draw_prediction_error(void) {
    return;
}

void ui_draw_offroute(void) {
    return;
}

void ui_draw_sensors(uint64_t start_us) {
    char *temp_buf = buf_get_temp();
    char *p = temp_buf;
    int head;
    const sensor_entry_t *sensors = track_get_sensor_log(&head);

    p = buf_append(p, "\033[11;1HRecent Sensors:\033[K");

    {
        int count = 0;
        for (int i = 0; i < SENSOR_LOG_SIZE && count < 10; i++) {
            int idx = (head - 1 - i + SENSOR_LOG_SIZE) % SENSOR_LOG_SIZE;
            if (sensors[idx].sensor_id != 0) {
                uint64_t elapsed_us = sensors[idx].time_us - start_us;
                uint64_t elapsed_tenths = elapsed_us / 100000;
                uint32_t minutes = (elapsed_tenths / 600);
                uint32_t seconds = (elapsed_tenths / 10) % 60;
                uint32_t tenths = elapsed_tenths % 10;
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
    }

    *p = '\0';
    ui_puts(temp_buf);
}
