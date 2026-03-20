#include "command.h"
#include "demo_manager.h"
#include "util.h"
#include "track.h"
#include "train_tracking/position.h"
#include "train_tracking/traffic_manager.h"
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

static int parse_train_token(const char *tok, int *out) {
    if (!parse_int_token(tok, out)) {
        ui_cmd_puts("Train must be a number\r\n");
        return 0;
    }
    if (!track_is_valid_train(*out)) {
        ui_cmd_puts("Train must be one of: 13, 14, 15, 17, 18, 55\r\n");
        return 0;
    }
    return 1;
}

static int tok_eq(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b && *a == *b) {
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
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
    char *argv[10];
    int argc = tokenize(cmd, argv, 10);

    if (argc == 0) return 1;

    // q
    if (tok_eq(argv[0], "q") || tok_eq(argv[0], "Q")) {
        ui_cmd_puts("Rebooting...\r\n");
        return 0;
    }

    // demo
    if (tok_eq(argv[0], "demo")) {
        return demo_handle_command(argc, argv);
    }

    if (tok_eq(argv[0], "findpos")) {
        if (argc < 2 || argc > 5) {
            ui_cmd_puts("Usage: findpos <t1> [t2] [t3] [t4]\r\n");
            return 2;
        }

        char *demo_argv[10];
        demo_argv[0] = "demo";
        demo_argv[1] = "locate";
        for (int i = 1; i < argc; i++) {
            demo_argv[i + 1] = argv[i];
        }
        return demo_handle_command(argc + 1, demo_argv);
    }

    // tr
    if (tok_eq(argv[0], "tr")) {
        if (argc != 3) {
            ui_cmd_puts("Usage: tr <train> <speed 0-14>\r\n");
            return 2;
        }
        int train = 0;
        int speed = 0;
        if (!parse_train_token(argv[1], &train)) return 2;
        if (!parse_int_token(argv[2], &speed)) {
            ui_cmd_puts("Speed must be a number\r\n");
            return 2;
        }
        if (speed < 0 || speed > 14) {
            ui_cmd_puts("Speed must be 0-14\r\n");
            return 2;
        }
        int can_speed = (speed == 0) ? 0 : 1 + (speed - 1) * 77;
        pos_on_speed_change(train, speed);
        track_set_speed(train, can_speed);
        return 1;
    }

    // sw
    if (tok_eq(argv[0], "sw")) {
        if (argc != 3) {
            ui_cmd_puts("Usage: sw <switch> <S|C>\r\n");
            return 2;
        }
        int sw = 0;
        if (!parse_int_token(argv[1], &sw)) {
            ui_cmd_puts("Switch must be a number\r\n");
            return 2;
        }
        if (!track_is_valid_switch(sw)) {
            ui_cmd_puts("Invalid switch. Valid: 1-18, 153-156\r\n");
            return 2;
        }
        char dir = argv[2][0];
        if (dir != 'S' && dir != 'C' && dir != 's' && dir != 'c') {
            ui_cmd_puts("Direction must be S or C\r\n");
            return 2;
        }
        if (dir == 's') dir = 'S';
        if (dir == 'c') dir = 'C';

        int owner = traffic_can_set_switch(sw, -1);
        if (owner >= 0) {
            char owner_buf[12];
            i2a(owner, owner_buf);
            ui_cmd_puts("Switch locked by tr ");
            ui_cmd_puts(owner_buf);
            ui_cmd_puts("\r\n");
            return 2;
        }

        track_set_switch(sw, dir);
        return 1;
    }

    // rv
    if (tok_eq(argv[0], "rv")) {
        if (argc != 2) {
            ui_cmd_puts("Usage: rv <train>\r\n");
            return 2;
        }

        int train = 0;
        if (!parse_train_token(argv[1], &train)) return 2;
        if (pos_is_train_goto_active(train)) {
            ui_cmd_puts("Error: goto in progress for this train\r\n");
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
    if (tok_eq(argv[0], "li")) {
        if (argc != 3) {
            ui_cmd_puts("Usage: li <train> <0|1>\r\n");
            return 2;
        }
        int train = 0;
        int on = 0;
        if (!parse_train_token(argv[1], &train)) return 2;
        if (!parse_int_token(argv[2], &on)) {
            ui_cmd_puts("Light must be a number\r\n");
            return 2;
        }
        if (on != 0 && on != 1) {
            ui_cmd_puts("Light must be 0 or 1\r\n");
            return 2;
        }
        track_set_light(train, on);
        return 1;
    }


    // goto <train> <node> [offset_mm]
    if (tok_eq(argv[0], "goto")) {
        if (argc < 3 || argc > 4) {
            ui_cmd_puts("Usage: goto <train> <node> [offset_mm]\r\n");
            return 2;
        }
        int train = 0;
        if (!parse_train_token(argv[1], &train)) return 2;
        if (rv_in_progress) {
            ui_cmd_puts("goto: cannot execute while rv is in progress\r\n");
            return 2;
        }
        track_node *target = parse_node(argv[2]);
        if (!target) {
            ui_cmd_puts("Unknown node name\r\n");
            return 2;
        }
        int32_t offset = 0;
        if (argc >= 4) {
            int offset_i = 0;
            if (!parse_int_token(argv[3], &offset_i)) {
                ui_cmd_puts("Offset must be a number\r\n");
                return 2;
            }
            offset = (int32_t)offset_i;
        }
        int gr = pos_goto(train, target, offset);
        if (!gr) {
            ui_cmd_puts("goto: no slot available\r\n");
            return 2;
        }
        return 1;
    }

    ui_cmd_puts("Unknown command: ");
    ui_cmd_puts(argv[0]);
    ui_cmd_puts("\r\n");
    return 2;
}
