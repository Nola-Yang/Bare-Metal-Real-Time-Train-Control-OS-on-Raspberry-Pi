#include "server/deadlock_server.h"
#include "server/position_server.h"
#include "server/nameserver.h"
#include "train_tracking/deadlock.h"
#include "syscall.h"

typedef enum {
    DEADLOCK_REQ_INIT = 0,
    DEADLOCK_REQ_TICK = 1,
} deadlock_request_type_t;

typedef struct {
    int type;
    uint64_t now_us;
} deadlock_request_t;

typedef struct {
    int status;
} deadlock_reply_t;

static int deadlock_server_send_request(int tid,
                                        deadlock_request_type_t type,
                                        uint64_t now_us) {
    deadlock_request_t req = {0};
    deadlock_reply_t reply = {0};

    req.type = (int)type;
    req.now_us = now_us;

    if (Send(tid, (const char *)&req, sizeof(req),
             (char *)&reply, sizeof(reply)) < 0) {
        return -1;
    }
    return reply.status;
}

static void deadlock_server_on_tick(uint64_t now_us) {
    int position_tid;
    deadlock_snapshot_t snapshot;
    deadlock_result_t result;

    position_tid = WhoIs(POSITION_SERVER_NAME);
    if (position_tid < 0) return;
    if (PositionServerGetDeadlockSnapshot(position_tid, now_us, &snapshot) < 0) {
        return;
    }
    if (!deadlock_plan_from_snapshot(&snapshot, &result)) return;
    (void)PositionServerApplyDeadlockResult(position_tid, &result);
}

void deadlock_server_task(void) {
    int tid;
    deadlock_request_t req;
    deadlock_reply_t reply;

    RegisterAs(DEADLOCK_SERVER_NAME);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        reply.status = 0;
        switch ((deadlock_request_type_t)req.type) {
        case DEADLOCK_REQ_INIT:
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;

        case DEADLOCK_REQ_TICK:
            deadlock_server_on_tick(req.now_us);
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;

        default:
            reply.status = -1;
            Reply(tid, (const char *)&reply, sizeof(reply));
            break;
        }
    }
}

int DeadlockServerInit(int tid) {
    return deadlock_server_send_request(tid, DEADLOCK_REQ_INIT, 0);
}

int DeadlockServerOnTick(int tid, uint64_t now_us) {
    return deadlock_server_send_request(tid, DEADLOCK_REQ_TICK, now_us);
}
