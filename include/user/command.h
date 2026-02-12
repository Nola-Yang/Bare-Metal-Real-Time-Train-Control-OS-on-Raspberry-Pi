#ifndef _command_h_
#define _command_h_ 1

#include <stdint.h>

// Execute a command
// Returns: 0 = exit, 1 = continue (no output), 2 = continue (has output)
int execute_it(char *cmdline, uint64_t now);

#endif /* _command_h_ */
