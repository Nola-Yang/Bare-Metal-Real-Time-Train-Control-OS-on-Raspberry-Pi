#include "command.h"
#include "util.h"
#include "track.h"
#include "ui.h"
#include "can_data.h"
#include "kassert.h"
#include "text_util.h"
#include <stddef.h>

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
int execute_cmd(char *cmd, int *rv_train, CmdQueue_t *cmd_queue) {
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

        int can_speed = speed_step_to_speed(speed);

        bool is_reversing = is_train_reversing(train);
        if (is_reversing) {
            CommandData_t cmdData = {CMD_SPEED, train, can_speed};
            ring_buffer_put(cmd_queue, cmdData);
            return 1;
        }

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
        if (!is_valid_switch_no(sw)) {
            ui_puts("Invalid switch. Valid: 1-18, 153-156\r\n");
            return 2;
        }
        char dir = argv[2][0];
        
        if (dir == 's') dir = SWITCH_STRAIGHT;
        if (dir == 'c') dir = SWITCH_CURVED;
        
        if (dir != SWITCH_STRAIGHT && dir != SWITCH_CURVED) {
            ui_puts("Direction must be S or C\r\n");
            return 2;
        }
        
        track_set_switch(sw, dir);
        track_update_switch(sw, dir);

        // 153/154 and 155/156 cannot both be C
        if (dir == SWITCH_CURVED) {
            int partner = -1;
            if (sw == 153) partner = 154;
            else if (sw == 154) partner = 153;
            else if (sw == 155) partner = 156;
            else if (sw == 156) partner = 155;

            if (partner >= 0) {
                int idx = get_switch_ind(partner);
                if (idx >= 0 && track_get_switch_state()[idx].state == SWITCH_CURVED) {
                    track_set_switch(partner, SWITCH_STRAIGHT);
                    track_update_switch(partner, SWITCH_STRAIGHT);
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
        if (rv_result <= 0) return 2;

        if (rv_result == 1) {
            *rv_train = train;  
        } else if (rv_result == 3) {
            CommandData_t cmdData = {CMD_REVERSE, train, 0};
            ring_buffer_put(cmd_queue, cmdData);
        }

        return 1;
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

    ui_puts("Unknown command: ");
    ui_puts(argv[0]);
    ui_puts("\r\n");
    return 2;
}


int execute_cmd_data(CommandData_t *cmd_data, int *rv_train, CmdQueue_t *cmd_queue) {
    int train;

    switch (cmd_data->command) {
        case CMD_SPEED: {
            train = cmd_data->arg1;
            track_set_speed(train, cmd_data->arg2);
            return 1;
        }

        case CMD_REVERSE: {
            train = cmd_data->arg1;
            int rv_result = track_start_reverse(train);
            if (rv_result <= 0) return 2;

            if (rv_result == 1) {
                *rv_train = train;
            } else if (rv_result == 3) {
                CommandData_t new_cmd_data = {CMD_REVERSE, train, 0};
                ring_buffer_put(cmd_queue, new_cmd_data);
            }

            return 1;
        }
    }

    return 2;
}
