#include "command.h"
#include "util.h"
#include "track.h"
#include "train_tracking/position.h"
#include "ui.h"
#include "kassert.h"

/* Parse node name token -> track_node*. Returns NULL on failure. */
static track_node *parse_node(const char *tok) {
    if (!tok || !tok[0]) return NULL;
    return pos_find_node(tok);
}

static int parse_int_token(const char *tok, int *out) {
    if (!tok || !tok[0] || !out) return 0;

    const char *p = tok;
    if (*p == '+' || *p == '-') p++;
    if (!*p) return 0;

    while (*p) {
        if (*p < '0' || *p > '9') return 0;
        p++;
    }

    *out = str2int(tok);
    return 1;
}

// Returns number of tokens
static int tokenize(char *cmd, char *argv[], int max_args) {
    KASSERT(cmd != NULL);
    KASSERT(argv != NULL);
    KASSERT(max_args > 0);

    int argc = 0;
    char *p = cmd;

    while (*p && argc < max_args) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }

    return argc;
}

// Returns: 0 = exit, 1 = continue (no output), 2 = continue (has output)
int execute_it(char *cmd, int *rv_train, int rv_in_progress) {
    KASSERT(cmd != NULL);
    KASSERT(rv_train != NULL);

    *rv_train = -1;
    char *argv[6];
    int argc = tokenize(cmd, argv, 6);

    if (argc == 0) return 1;

    // q
    if ((argv[0][0] == 'q' || argv[0][0] == 'Q') && argv[0][1] == '\0') {
        ui_puts("Rebooting...\r\n");
        return 0;
    }

    // tr
    if (argv[0][0] == 't' && argv[0][1] == 'r' && argv[0][2] == '\0') {
        if (argc != 3) {
            ui_puts("Usage: tr <train> <speed 0-14>\r\n");
            return 2;
        }
        int train = 0;
        int speed = 0;
        if (!parse_int_token(argv[1], &train)) {
            ui_puts("Train must be a number\r\n");
            return 2;
        }
        if (!parse_int_token(argv[2], &speed)) {
            ui_puts("Speed must be a number\r\n");
            return 2;
        }
        if (speed < 0 || speed > 14) {
            ui_puts("Speed must be 0-14\r\n");
            return 2;
        }
        int can_speed = (speed == 0) ? 0 : 1 + (speed - 1) * 77;
        track_set_speed(train, can_speed);
        pos_on_speed_change(train, speed);
        return 1;
    }

    // sw
    if (argv[0][0] == 's' && argv[0][1] == 'w' && argv[0][2] == '\0') {
        if (argc != 3) {
            ui_puts("Usage: sw <switch> <S|C>\r\n");
            return 2;
        }
        int sw = 0;
        if (!parse_int_token(argv[1], &sw)) {
            ui_puts("Switch must be a number\r\n");
            return 2;
        }
        if (!track_is_valid_switch(sw)) {
            ui_puts("Invalid switch. Valid: 1-18, 153-156\r\n");
            return 2;
        }
        char dir = argv[2][0];
        if (dir != 'S' && dir != 'C' && dir != 's' && dir != 'c') {
            ui_puts("Direction must be S or C\r\n");
            return 2;
        }
        if (dir == 's') dir = 'S';
        if (dir == 'c') dir = 'C';
        track_set_switch(sw, dir);
        return 1;
    }

    // rv
    if (argv[0][0] == 'r' && argv[0][1] == 'v' && argv[0][2] == '\0') {
        if (argc != 2) {
            ui_puts("Usage: rv <train>\r\n");
            return 2;
        }

        int train = 0;
        if (!parse_int_token(argv[1], &train)) {
            ui_puts("Train must be a number\r\n");
            return 2;
        }
        if (pos_is_train_goto_active(train)) {
            ui_puts("Error: goto in progress for this train\r\n");
            return 2;
        }
        int rv_result = track_start_reverse(train);
        if (rv_result > 0) {
            if (rv_result == 1) {
                *rv_train = train;  
            } else {
                /*train was stopped */
                pos_on_reverse(train);
            }
            return 1;
        } else {
            return 2;
        }
    }

    // li (light)
    if (argv[0][0] == 'l' && argv[0][1] == 'i' && argv[0][2] == '\0') {
        if (argc != 3) {
            ui_puts("Usage: li <train> <0|1>\r\n");
            return 2;
        }
        int train = 0;
        int on = 0;
        if (!parse_int_token(argv[1], &train)) {
            ui_puts("Train must be a number\r\n");
            return 2;
        }
        if (!parse_int_token(argv[2], &on)) {
            ui_puts("Light must be a number\r\n");
            return 2;
        }
        if (on != 0 && on != 1) {
            ui_puts("Light must be 0 or 1\r\n");
            return 2;
        }
        track_set_light(train, on);
        return 1;
    }


    // goto <train> <node> [offset_mm]
    if (argv[0][0] == 'g' && argv[0][1] == 'o' && argv[0][2] == 't' &&
        argv[0][3] == 'o' && argv[0][4] == '\0') {
        if (argc < 3 || argc > 4) {
            ui_puts("Usage: goto <train> <node> [offset_mm]\r\n");
            return 2;
        }
        int train = 0;
        if (!parse_int_token(argv[1], &train)) {
            ui_puts("Train must be a number\r\n");
            return 2;
        }
        if (pos_is_train_goto_active(train)) {
            ui_puts("Error: goto in progress for this train\r\n");
            return 2;
        }
        if (rv_in_progress) {
            ui_puts("goto: cannot execute while rv is in progress\r\n");
            return 2;
        }
        track_node *target = parse_node(argv[2]);
        if (!target) {
            ui_puts("Unknown node name\r\n");
            return 2;
        }
        int32_t offset = 0;
        if (argc >= 4) {
            int offset_i = 0;
            if (!parse_int_token(argv[3], &offset_i)) {
                ui_puts("Offset must be a number\r\n");
                return 2;
            }
            offset = (int32_t)offset_i;
        }
        int gr = pos_goto(train, target, offset);
        if (!gr) {
            ui_puts("goto: no slot available\r\n");
            return 2;
        }
        return 1;
    }

    ui_puts("Unknown command: ");
    ui_puts(argv[0]);
    ui_puts("\r\n");
    return 2;
}
