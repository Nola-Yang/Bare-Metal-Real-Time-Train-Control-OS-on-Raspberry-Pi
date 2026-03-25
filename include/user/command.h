#ifndef _command_h_
#define _command_h_ 1

#include "runtime_protocol.h"

// Parse a command line into a structured command.
// Returns 0 for an empty line, 1 when out is filled (including parse errors).
int parse_train_command(const char *cmdline, train_command_t *out);

#endif /* _command_h_ */
