#ifndef _game_server_h_
#define _game_server_h_ 1

#include <stdint.h>
#include "runtime_protocol.h"

#define GAME_SERVER_NAME "GameServer"

void game_server_task(void);
int GameServerInit(int tid);
int GameServerHandleCommand(int tid, const train_command_t *cmd);
int GameServerOnTick(int tid, uint64_t now_us);

#endif /* _game_server_h_ */
