#include "command.h"
#include "util.h"
#include "track.h"
#include "ui.h"
#include "kassert.h"

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
int execute_it(char *cmd, int *rv_train) {
    KASSERT(cmd != NULL);
    KASSERT(rv_train != NULL);

    *rv_train = -1;
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
            ui_puts("Usage: tr <train> <speed 0-14>\r\n");
            return 2;
        }
        int train = str2int(argv[1]);
        int speed = str2int(argv[2]);
        if (speed < 0 || speed > 14) {
            ui_puts("Speed must be 0-14\r\n");
            return 2;
        }
        int can_speed = (speed == 0) ? 0 : 1 + (speed - 1) * 77;
        track_set_speed(train, can_speed);
        return 1;
    }

    // sw
    if (argv[0][0] == 's' && argv[0][1] == 'w' && argv[0][2] == '\0') {
        if (argc != 3) {
            ui_puts("Usage: sw <switch> <S|C>\r\n");
            return 2;
        }
        int sw = str2int(argv[1]);
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
        track_update_switch(sw, dir);

        // 153/154 and 155/156 cannot both be C
        if (dir == 'C') {
            int partner = -1;
            if (sw == 153) partner = 154;
            else if (sw == 154) partner = 153;
            else if (sw == 155) partner = 156;
            else if (sw == 156) partner = 155;

            if (partner >= 0) {
                int idx = track_switch_to_index(partner);
                if (idx >= 0 && track_get_switch_state()[idx].state == 'C') {
                    track_set_switch(partner, 'S');
                    track_update_switch(partner, 'S');
                }
            }
        }

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
        int rv_result = track_start_reverse(train);
        if (rv_result > 0) {
            if (rv_result == 1) {
                *rv_train = train;  
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
        int train = str2int(argv[1]);
        int on = str2int(argv[2]);
        if (on != 0 && on != 1) {
            ui_puts("Light must be 0 or 1\r\n");
            return 2;
        }
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
