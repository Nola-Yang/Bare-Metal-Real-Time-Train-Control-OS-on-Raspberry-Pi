#include "server/manager_server.h"
#include "server/nameserver.h"
#include "syscall.h"

int manager_server_send_request(int tid,
                                manager_request_type_t type,
                                const train_command_t *cmd,
                                uint64_t now_us) {
    manager_request_t req = {0};
    manager_reply_t reply = {0};

    req.type = (int)type;
    req.now_us = now_us;
    if (cmd) req.command = *cmd;

    if (Send(tid, (const char *)&req, sizeof(req),
             (char *)&reply, sizeof(reply)) < 0) {
        return -1;
    }
    return reply.status;
}

void manager_server_task_loop(const char *server_name,
                              void (*init_fn)(void),
                              int (*command_fn)(const train_command_t *cmd),
                              void (*tick_fn)(uint64_t now_us)) {
    int tid;
    manager_request_t req;
    manager_reply_t reply;

    RegisterAs(server_name);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        reply.status = 0;

        switch ((manager_request_type_t)req.type) {
        case MANAGER_REQ_INIT:
            if (init_fn) init_fn();
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;

        case MANAGER_REQ_COMMAND:
            reply.status = command_fn ? command_fn(&req.command) : -1;
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;

        case MANAGER_REQ_TICK:
            if (tick_fn) tick_fn(req.now_us);
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;

        default:
            reply.status = -1;
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;
        }
    }
}
