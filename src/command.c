#include "command.h"
#include "util.h"
#include "track.h"
#include "ui.h"

// Returns number of tokens
static int tokenize(char *cmd, char *argv[], int max_args) {
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
int execute_it(char *cmd, uint64_t now) {
    char *argv[4];
    int argc = tokenize(cmd, argv, 4);

    if (argc == 0) return 1;

    // q
    if ((argv[0][0] == 'q' || argv[0][0] == 'Q') && argv[0][1] == '\0') {
        ui_puts("Rebooting...\r\n");
        return 0;
    }

    // tr
    if (argv[0][0] == 't' && argv[0][1] == 'r' && argv[0][2] == '\0') {
        if (argc != 3) {
            ui_puts("Usage: tr <train> <speed>\r\n");
            return 2;
        }
        int train = str2int(argv[1]);
        int speed = str2int(argv[2]);
        track_set_speed(train, speed);
        return 1;
    }

    // sw
    if (argv[0][0] == 's' && argv[0][1] == 'w' && argv[0][2] == '\0') {
        if (argc != 3) {
            ui_puts("Usage: sw <switch> <S|C>\r\n");
            return 2;
        }
        int sw = str2int(argv[1]);
        char dir = argv[2][0];
        if (dir != 'S' && dir != 'C' && dir != 's' && dir != 'c') {
            ui_puts("Direction must be S or C\r\n");
            return 2;
        }
        if (dir == 's') dir = 'S';
        if (dir == 'c') dir = 'C';
        track_set_switch(sw, dir);
        track_update_switch(sw, dir);
        ui_mark_switches_dirty();
        return 1;
    }

    // rv
    if (argv[0][0] == 'r' && argv[0][1] == 'v' && argv[0][2] == '\0') {
        if (argc != 2) {
            ui_puts("Usage: rv <train>\r\n");
            return 2;
        }

        int train = str2int(argv[1]);
        if (track_start_reverse(train, now)) {
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
        int train = str2int(argv[1]);
        int on = str2int(argv[2]);
        track_set_light(train, on);
        return 1;
    }

    // init
    if (argv[0][0] == 'i' && argv[0][1] == 'n' && argv[0][2] == 'i' && argv[0][3] == 't' && argv[0][4] == '\0') {
        ui_puts("Initializing track...\r\n");

        for (int i = 1; i <= 18; i++) {
            track_set_switch(i, 'S');
            track_update_switch(i, 'S');
        }

        for (int i = 153; i <= 156; i++) {
            track_set_switch(i, 'S');
            track_update_switch(i, 'S');
        }
        ui_mark_switches_dirty();

        ui_puts("Track initialized: all switches set to straight\r\n");
        return 2;
    }

    ui_puts("Unknown command: ");
    ui_puts(argv[0]);
    ui_puts("\r\n");
    return 2;
}
