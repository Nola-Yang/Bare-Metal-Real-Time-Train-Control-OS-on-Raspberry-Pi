#include "server/game_server.h"
#include "game_manager.h"
#include "server/nameserver.h"
#include "syscall.h"

typedef enum {
    GAME_REQ_INIT = 0,
    GAME_REQ_COMMAND = 1,
    GAME_REQ_TICK = 2,
} game_request_type_t;

typedef struct {
    int type;
    uint64_t now_us;
    train_command_t command;
} GameRequest_t;

typedef struct {
    int status;
} GameReply_t;

static int game_send_request(int tid, const GameRequest_t *req, GameReply_t *reply) {
    if (!req || !reply) return -1;
    return Send(tid, (const char *)req, sizeof(*req),
                (char *)reply, sizeof(*reply));
}

void game_server_task(void) {
    int tid;
    GameRequest_t req;
    GameReply_t reply;

    RegisterAs(GAME_SERVER_NAME);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        reply.status = 0;

        switch ((game_request_type_t)req.type) {
        case GAME_REQ_INIT:
            game_init();
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;
        case GAME_REQ_COMMAND:
            reply.status = game_handle_command(&req.command);
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;
        case GAME_REQ_TICK:
            game_on_tick(req.now_us);
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;
        default:
            reply.status = -1;
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;
        }
    }
}

int GameServerInit(int tid) {
    GameRequest_t req;
    GameReply_t reply;

    req.type = GAME_REQ_INIT;
    return (game_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int GameServerHandleCommand(int tid, const train_command_t *cmd) {
    GameRequest_t req;
    GameReply_t reply;

    if (!cmd) return -1;
    req.type = GAME_REQ_COMMAND;
    req.command = *cmd;
    return (game_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int GameServerOnTick(int tid, uint64_t now_us) {
    GameRequest_t req;
    GameReply_t reply;

    req.type = GAME_REQ_TICK;
    req.now_us = now_us;
    return (game_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}
