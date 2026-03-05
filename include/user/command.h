#ifndef _command_h_
#define _command_h_ 1

#include <stdint.h>

// Execute a command
// Returns: 0 = exit, 1 = continue (no output), 2 = continue (has output)
// On rv success, sets *rv_train = train_num; otherwise *rv_train = -1
// rv_in_progress: non-zero if a reverse is still pending (goto is blocked)
int execute_it(char *cmdline, int *rv_train, int rv_in_progress);

#endif /* _command_h_ */
