#include "server/position_server.h"
#include "train_tracking/position.h"
#include "train_tracking/traffic_manager.h"
#include "server/nameserver.h"
#include "syscall.h"
#include "kassert.h"
#include "train_tracking/speed_table.h"

typedef enum {
    POS_REQ_INIT = 0,
    POS_REQ_SENSOR = 1,
    POS_REQ_FAST_TICK = 2,
    POS_REQ_REPLAN_TICK = 3,
    POS_REQ_SWITCH_SETTLE_TICK = 4,
    POS_REQ_SPEED_CHANGE = 5,
    POS_REQ_REVERSE = 6,
    POS_REQ_GOTO = 7,
    POS_REQ_START_FIND_POS = 8,
    POS_REQ_MARK_ROUTES_DIRTY = 9,
    POS_REQ_RELEASE_TRAIN = 10,
    POS_REQ_RESET_ALL = 11,
    POS_REQ_RELEASE_NODE = 12,
} position_request_type_t;

typedef struct {
    int type;
    int train;
    int value;
    int target_idx;
    int32_t offset_mm;
    uint16_t sensor_id;
    uint64_t now_us;
} PositionRequest_t;

typedef struct {
    int status;
} PositionReply_t;

static int position_send_request(int tid,
                                 const PositionRequest_t *req,
                                 PositionReply_t *reply) {
    if (!req || !reply) return -1;
    return Send(tid, (const char *)req, sizeof(*req),
                (char *)reply, sizeof(*reply));
}

void position_server_task(void) {
    int tid;
    PositionRequest_t req;
    PositionReply_t reply;

    RegisterAs(POSITION_SERVER_NAME);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        reply.status = 0;

        switch ((position_request_type_t)req.type) {
            case POS_REQ_INIT:
                pos_init();
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_SENSOR:
                pos_on_sensor_trigger(req.sensor_id, req.now_us);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_FAST_TICK:
                pos_on_tick(req.now_us);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_REPLAN_TICK:
                pos_replan_on_tick(req.now_us);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_SWITCH_SETTLE_TICK:
                pos_on_switch_settle_tick(req.now_us);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_SPEED_CHANGE:
                pos_on_speed_change(req.train, req.value);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_REVERSE:
                pos_on_reverse(req.train);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_GOTO:
                if (req.target_idx < 0 || req.target_idx >= TRACK_MAX) {
                    reply.status = 0;
                } else {
                    reply.status = pos_goto(req.train, &g_track[req.target_idx],
                                            req.value, req.offset_mm);
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_START_FIND_POS:
                reply.status = pos_start_find_pos(req.train, req.value);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_MARK_ROUTES_DIRTY:
                pos_mark_routes_dirty();
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_RELEASE_TRAIN:
                traffic_release_train(req.train);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_RELEASE_NODE:
                if (req.target_idx < 0 || req.target_idx >= TRACK_MAX) {
                    reply.status = 0;
                } else {
                    reply.status = traffic_release_physical_node(&g_track[req.target_idx]);
                }
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case POS_REQ_RESET_ALL:
                pos_init();
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            default:
                reply.status = -1;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}

int PositionServerInit(int tid) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_INIT;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerOnSensor(int tid, uint16_t sensor_id, uint64_t time_us) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_SENSOR;
    req.sensor_id = sensor_id;
    req.now_us = time_us;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerOnFastTick(int tid, uint64_t now_us) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_FAST_TICK;
    req.now_us = now_us;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerOnReplanTick(int tid, uint64_t now_us) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_REPLAN_TICK;
    req.now_us = now_us;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerOnSwitchSettleTick(int tid, uint64_t now_us) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_SWITCH_SETTLE_TICK;
    req.now_us = now_us;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerSpeedChange(int tid, int train_num, int user_speed) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_SPEED_CHANGE;
    req.train = train_num;
    req.value = user_speed;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerReverse(int tid, int train_num) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_REVERSE;
    req.train = train_num;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerGoto(int tid, int train_num, int target_idx, int speed_level, int32_t offset_mm) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_GOTO;
    req.train = train_num;
    req.target_idx = target_idx;
    req.offset_mm = offset_mm;
    req.value = speed_level;

    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerStartFindPos(int tid, int train_num) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_START_FIND_POS;
    req.train = train_num;
    req.value = DEFAULT_SPEED_LEVEL;
    
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerMarkRoutesDirty(int tid) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_MARK_ROUTES_DIRTY;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerReleaseTrain(int tid, int train_num) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_RELEASE_TRAIN;
    req.train = train_num;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerReleaseNode(int tid, int target_idx) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_RELEASE_NODE;
    req.target_idx = target_idx;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}

int PositionServerResetAll(int tid) {
    PositionRequest_t req;
    PositionReply_t reply;

    req.type = POS_REQ_RESET_ALL;
    return (position_send_request(tid, &req, &reply) < 0) ? -1 : reply.status;
}
