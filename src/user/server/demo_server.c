#include "server/demo_server.h"
#include "server/manager_server.h"
#include "demo_manager.h"

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
        int trains[TRAIN_CMD_MAX_ARGS];
        int train_count = 0;
        for (int i = 1; i < cmd->argc && train_count < TRAIN_CMD_MAX_ARGS; i++) {
            int v = 0;
            const char *p = cmd->argv[i];
            while (*p >= '0' && *p <= '9') { v = v * 10 + (*p - '0'); p++; }
            trains[train_count++] = v;
        }
        (void)argc;
        (void)local_argv;
        return demo_start_locate(train_count, trains);
    }

    return 2;
}

void demo_server_task(void) {
    manager_server_task_loop(DEMO_SERVER_NAME,
                             demo_init,
                             demo_handle_runtime_command,
                             demo_on_tick);
}

int DemoServerInit(int tid) {
    return manager_server_send_request(tid, MANAGER_REQ_INIT, 0, 0);
}

int DemoServerHandleCommand(int tid, const train_command_t *cmd) {
    if (!cmd) return -1;
    return manager_server_send_request(tid, MANAGER_REQ_COMMAND, cmd, 0);
}

int DemoServerOnTick(int tid, uint64_t now_us) {
    return manager_server_send_request(tid, MANAGER_REQ_TICK, 0, now_us);
}
