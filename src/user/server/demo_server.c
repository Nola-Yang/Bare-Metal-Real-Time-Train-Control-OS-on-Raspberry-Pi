#include "server/demo_server.h"
#include "demo_manager.h"
#include "server/nameserver.h"
#include "syscall.h"

typedef enum {
    DEMO_REQ_INIT = 0,
    DEMO_REQ_COMMAND = 1,
    DEMO_REQ_TICK = 2,
} demo_request_type_t;

typedef struct {
    int type;
    uint64_t now_us;
    train_command_t command;
} DemoRequest_t;

typedef struct {
    int status;
} DemoReply_t;

static int demo_send_request(int tid, const DemoRequest_t *req, DemoReply_t *reply) {
    if (!req || !reply) return -1;
    return Send(tid, (const char *)req, sizeof(*req),
                (char *)reply, sizeof(*reply));
}

static int demo_handle_runtime_command(const train_command_t *cmd) {
    char *argv[TRAIN_CMD_MAX_ARGS + 1];
    char local_argv[TRAIN_CMD_MAX_ARGS + 1][TRAIN_CMD_TOKEN_MAX];
    int argc = 0;

    if (!cmd) return 2;

    if (cmd->type == TRAIN_CMD_DEMO) {
        argc = cmd->argc;
        for (int i = 0; i < argc; i++) {
            int j = 0;
            while (cmd->argv[i][j] && j + 1 < TRAIN_CMD_TOKEN_MAX) {
                local_argv[i][j] = cmd->argv[i][j];
                j++;
            }
            local_argv[i][j] = '\0';
            argv[i] = local_argv[i];
        }
        return demo_handle_command(argc, argv);
    }

    if (cmd->type == TRAIN_CMD_FINDPOS) {
        argc = cmd->argc + 1;
        local_argv[0][0] = 'd';
        local_argv[0][1] = 'e';
        local_argv[0][2] = 'm';
        local_argv[0][3] = 'o';
        local_argv[0][4] = '\0';
        local_argv[1][0] = 'l';
        local_argv[1][1] = 'o';
        local_argv[1][2] = 'c';
        local_argv[1][3] = 'a';
        local_argv[1][4] = 't';
        local_argv[1][5] = 'e';
        local_argv[1][6] = '\0';
        argv[0] = local_argv[0];
        argv[1] = local_argv[1];

        for (int i = 1; i < cmd->argc; i++) {
            int dst = i + 1;
            int j = 0;
            while (cmd->argv[i][j] && j + 1 < TRAIN_CMD_TOKEN_MAX) {
                local_argv[dst][j] = cmd->argv[i][j];
                j++;
            }
            local_argv[dst][j] = '\0';
            argv[dst] = local_argv[dst];
        }
        return demo_handle_command(argc, argv);
    }

    return 2;
}

void demo_server_task(void) {
    int tid;
    DemoRequest_t req;
    DemoReply_t reply;

    RegisterAs(DEMO_SERVER_NAME);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        reply.status = 0;

        switch ((demo_request_type_t)req.type) {
            case DEMO_REQ_INIT:
                demo_init();
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case DEMO_REQ_COMMAND:
                reply.status = demo_handle_runtime_command(&req.command);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case DEMO_REQ_TICK:
                demo_on_tick(req.now_us);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            default:
                reply.status = -1;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}

int DemoServerInit(int tid) {
    DemoRequest_t req;
    DemoReply_t reply;

    req.type = DEMO_REQ_INIT;
    return (demo_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int DemoServerHandleCommand(int tid, const train_command_t *cmd) {
    DemoRequest_t req;
    DemoReply_t reply;

    if (!cmd) return -1;

    req.type = DEMO_REQ_COMMAND;
    req.command = *cmd;
    return (demo_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int DemoServerOnTick(int tid, uint64_t now_us) {
    DemoRequest_t req;
    DemoReply_t reply;

    req.type = DEMO_REQ_TICK;
    req.now_us = now_us;
    return (demo_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}
