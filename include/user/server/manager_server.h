#ifndef _manager_server_h_
#define _manager_server_h_ 1

#include <stdint.h>
#include "runtime_protocol.h"

typedef enum {
    MANAGER_REQ_INIT = 0,
    MANAGER_REQ_COMMAND = 1,
    MANAGER_REQ_TICK = 2,
} manager_request_type_t;

typedef struct {
    int type;
    uint64_t now_us;
    train_command_t command;
} manager_request_t;

typedef struct {
    int status;
} manager_reply_t;

int manager_server_send_request(int tid,
                                manager_request_type_t type,
                                const train_command_t *cmd,
                                uint64_t now_us);

void manager_server_task_loop(const char *server_name,
                              void (*init_fn)(void),
                              int (*command_fn)(const train_command_t *cmd),
                              void (*tick_fn)(uint64_t now_us));

#endif /* _manager_server_h_ */
