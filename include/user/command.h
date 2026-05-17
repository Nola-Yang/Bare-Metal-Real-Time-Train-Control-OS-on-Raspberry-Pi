#ifndef _command_h_
#define _command_h_ 1

#include "runtime_protocol.h"
#include <stdbool.h>

// Parse a command line into a structured command.
// Returns 0 for an empty line, 1 when out is filled (including parse errors).
int parse_train_command(const char *cmdline, train_command_t *out);

bool is_valid_goto_speed(int speed_level);
bool is_valid_speed_level(int speed_level);

#endif /* _command_h_ */
