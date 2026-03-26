#include "command.h"
#include "track.h"
#include "util.h"
#include "kassert.h"

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

bool is_valid_goto_speed(int speed_level) {
    switch (speed_level) {
        case 8:
        case 10:
            return true;
        
        default:
            return false;
    }
}

bool is_valid_speed_level(int speed_level) {
    return (0 <= speed_level && speed_level <= 14);
}

static int tok_eq(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b && *a == *b) {
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static int tokenize(char *cmd, char *argv[], int max_args) {
    int argc = 0;
    char *p = cmd;

    KASSERT(cmd != NULL);
    KASSERT(argv != NULL);
    KASSERT(max_args > 0);

    while (*p && argc < max_args) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }

    return argc;
}

static void command_init(train_command_t *out) {
    KASSERT(out != NULL);

    out->type = TRAIN_CMD_NONE;
    out->error = TRAIN_CMD_ERR_NONE;
    out->argc = 0;
    out->raw_cmdline[0] = '\0';
    out->train = 0;
    out->value = 0;
    out->aux = 0;
    out->target_idx = -1;
    out->offset_mm = 0;
    out->dir = '\0';

    for (int i = 0; i < TRAIN_CMD_MAX_ARGS; i++) {
        out->argv[i][0] = '\0';
    }
}

static void command_copy_token(char dst[TRAIN_CMD_TOKEN_MAX], const char *src) {
    int i = 0;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    while (src[i] && i + 1 < TRAIN_CMD_TOKEN_MAX) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void command_set_parse_error(train_command_t *out,
                                    train_command_error_t error) {
    out->type = TRAIN_CMD_PARSE_ERROR;
    out->error = error;
}

static int parse_train_token(const char *tok, train_command_t *out, int *train) {
    if (!parse_int_token(tok, train)) {
        command_set_parse_error(out, TRAIN_CMD_ERR_TRAIN_NOT_NUMBER);
        return 0;
    }
    if (!track_is_valid_train(*train)) {
        command_set_parse_error(out, TRAIN_CMD_ERR_TRAIN_INVALID);
        return 0;
    }
    return 1;
}

static void copy_raw_cmdline(train_command_t *out, const char *cmdline) {
    int i = 0;
    if (!cmdline) {
        out->raw_cmdline[0] = '\0';
        return;
    }
    while (cmdline[i] && i + 1 < TRAIN_CMD_MAX_LEN) {
        out->raw_cmdline[i] = cmdline[i];
        i++;
    }
    out->raw_cmdline[i] = '\0';
}

int parse_train_command(const char *cmdline, train_command_t *out) {
    char buf[TRAIN_CMD_MAX_LEN];
    char *argv[TRAIN_CMD_MAX_ARGS];
    int argc;

    KASSERT(out != NULL);
    command_init(out);
    copy_raw_cmdline(out, cmdline);

    if (!cmdline || !cmdline[0]) return 0;

    for (int i = 0; i < TRAIN_CMD_MAX_LEN; i++) {
        buf[i] = cmdline[i];
        if (cmdline[i] == '\0') break;
    }
    buf[TRAIN_CMD_MAX_LEN - 1] = '\0';

    argc = tokenize(buf, argv, TRAIN_CMD_MAX_ARGS);
    if (argc == 0) return 0;

    out->argc = argc;
    for (int i = 0; i < argc; i++) {
        command_copy_token(out->argv[i], argv[i]);
    }

    if (tok_eq(argv[0], "q") || tok_eq(argv[0], "Q")) {
        out->type = TRAIN_CMD_QUIT;
        return 1;
    }

    if (tok_eq(argv[0], "demo")) {
        out->type = TRAIN_CMD_DEMO;
        if (argc < 2) {
            command_set_parse_error(out, TRAIN_CMD_ERR_USAGE_DEMO);
        }
        return 1;
    }

    if (tok_eq(argv[0], "findpos")) {
        out->type = TRAIN_CMD_FINDPOS;
        if (argc < 2 || argc > 5) {
            command_set_parse_error(out, TRAIN_CMD_ERR_USAGE_FINDPOS);
        }
        return 1;
    }

    if (tok_eq(argv[0], "tr")) {
        int train = 0;
        int speed = 0;

        out->type = TRAIN_CMD_TR;
        if (argc != 3) {
            command_set_parse_error(out, TRAIN_CMD_ERR_USAGE_TR);
            return 1;
        }
        if (!parse_train_token(argv[1], out, &train)) return 1;
        if (!parse_int_token(argv[2], &speed)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_SPEED_NOT_NUMBER);
            return 1;
        }
        if (!is_valid_speed_level(speed)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_SPEED_RANGE);
            return 1;
        }
        out->train = train;
        out->value = speed;
        return 1;
    }

    if (tok_eq(argv[0], "sw")) {
        int sw = 0;
        char dir;

        out->type = TRAIN_CMD_SW;
        if (argc != 3) {
            command_set_parse_error(out, TRAIN_CMD_ERR_USAGE_SW);
            return 1;
        }
        if (!parse_int_token(argv[1], &sw)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_SWITCH_NOT_NUMBER);
            return 1;
        }
        if (!track_is_valid_switch(sw)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_SWITCH_INVALID);
            return 1;
        }

        dir = argv[2][0];
        if (dir == 's') dir = 'S';
        if (dir == 'c') dir = 'C';
        if (dir != 'S' && dir != 'C') {
            command_set_parse_error(out, TRAIN_CMD_ERR_SWITCH_DIR);
            return 1;
        }

        out->value = sw;
        out->dir = dir;
        return 1;
    }

    if (tok_eq(argv[0], "rv")) {
        int train = 0;

        out->type = TRAIN_CMD_RV;
        if (argc != 2) {
            command_set_parse_error(out, TRAIN_CMD_ERR_USAGE_RV);
            return 1;
        }
        if (!parse_train_token(argv[1], out, &train)) return 1;
        out->train = train;
        return 1;
    }

    if (tok_eq(argv[0], "li")) {
        int train = 0;
        int on = 0;

        out->type = TRAIN_CMD_LIGHT;
        if (argc != 3) {
            command_set_parse_error(out, TRAIN_CMD_ERR_USAGE_LIGHT);
            return 1;
        }
        if (!parse_train_token(argv[1], out, &train)) return 1;
        if (!parse_int_token(argv[2], &on)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_LIGHT_NOT_NUMBER);
            return 1;
        }
        if (on != 0 && on != 1) {
            command_set_parse_error(out, TRAIN_CMD_ERR_LIGHT_RANGE);
            return 1;
        }
        out->train = train;
        out->value = on;
        return 1;
    }

    if (tok_eq(argv[0], "goto")) {
        int train = 0;
        int offset = 0;
        int speed = 0;

        track_node *target;

        out->type = TRAIN_CMD_GOTO;
        if (argc < 4 || argc > 5) {
            command_set_parse_error(out, TRAIN_CMD_ERR_USAGE_GOTO);
            return 1;
        }
        if (!parse_train_token(argv[1], out, &train)) return 1;

        target = track_find_node(argv[2]);
        if (!target) {
            command_set_parse_error(out, TRAIN_CMD_ERR_NODE_UNKNOWN);
            return 1;
        }

        if (!parse_int_token(argv[3], &speed)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_SPEED_NOT_NUMBER);
            return 1;
        }

        if (!is_valid_speed_level(speed)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_SPEED_RANGE);
            return 1;
        }

        if (!is_valid_goto_speed(speed)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_INVALID_GOTO_SPEED);
            return 1;
        }

        if (argc >= 5 && !parse_int_token(argv[4], &offset)) {
            command_set_parse_error(out, TRAIN_CMD_ERR_OFFSET_NOT_NUMBER);
            return 1;
        }

        out->train = train;
        out->target_idx = (int)(target - g_track);
        out->offset_mm = (int32_t)offset;
        out->value = speed;
        return 1;
    }

    out->type = TRAIN_CMD_UNKNOWN;
    out->error = TRAIN_CMD_ERR_UNKNOWN;
    return 1;
}
