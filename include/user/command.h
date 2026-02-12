#ifndef _command_h_
#define _command_h_ 1

#include <stdint.h>

// Execute a command
// Returns: 0 = exit, 1 = continue (no output), 2 = continue (has output)
// On rv success, sets *rv_train = train_num; otherwise *rv_train = -1
int execute_it(char *cmdline, int *rv_train);

#endif /* _command_h_ */
