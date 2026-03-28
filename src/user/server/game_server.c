#include "server/game_server.h"
#include "server/manager_server.h"
#include "game_manager.h"

void game_server_task(void) {
    manager_server_task_loop(GAME_SERVER_NAME,
                             game_init,
                             game_handle_command,
                             game_on_tick);
}

int GameServerInit(int tid) {
    return manager_server_send_request(tid, MANAGER_REQ_INIT, 0, 0);
}

int GameServerHandleCommand(int tid, const train_command_t *cmd) {
    if (!cmd) return -1;
    return manager_server_send_request(tid, MANAGER_REQ_COMMAND, cmd, 0);
}

int GameServerOnTick(int tid, uint64_t now_us) {
    return manager_server_send_request(tid, MANAGER_REQ_TICK, 0, now_us);
}
