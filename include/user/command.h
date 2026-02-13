#ifndef _command_h_
#define _command_h_ 1

#include <stdint.h>
#include "ring_buffer.h"


#define CMD_REVERSE 1
#define CMD_SPEED 2

// CommandData: Data to hold info about a command
typedef struct {
    int32_t command;
    int arg1;
    int arg2;
} CommandData_t;

// CmdQueue: Queue to buffer commands that cannot be immediately ran
#define CMD_QUEUE_MAX 4
RING_BUFFER_DECLARE(CmdQueue_t, CommandData_t, CMD_QUEUE_MAX);


// Execute a command
// Returns: 0 = exit, 1 = continue (no output), 2 = continue (has output)
// On rv success, sets *rv_train = train_num; otherwise *rv_train = -1
int execute_cmd(char *cmdline, int *rv_train, CmdQueue_t *cmd_queue);

// execute_cmd_data: Executes a buffered command
// Returns: 0 = exit, 1 = continue (no output), 2 = continue (has output)
int execute_cmd_data(CommandData_t *cmd_data, int *rv_train, CmdQueue_t *cmd_queue);

#endif /* _command_h_ */
