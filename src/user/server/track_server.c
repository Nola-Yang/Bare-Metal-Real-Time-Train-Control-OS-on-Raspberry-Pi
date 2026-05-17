#include "server/track_server.h"
#include "track_server_internal.h"
#include "server/can_server.h"
#include "server/nameserver.h"
#include "runtime_protocol.h"
#include "task_scheduler.h"
#include "syscall.h"
#include "timer.h"
#include "ui.h"
#include "kassert.h"

typedef struct {
    int runtime_tid;
    runtime_event_t event;
} TrackRuntimeNotifyMsg_t;

static void track_runtime_notify_task(void) {
    int tid;
    TrackRuntimeNotifyMsg_t msg;
    TrackReply_t reply;
    runtime_reply_t runtime_reply;

    int msglen = Receive(&tid, (char *)&msg, sizeof(msg));
    (void)msglen;

    reply.status = 0;
    Reply(tid, (const char *)&reply, sizeof(reply));

    if (msg.runtime_tid >= 0) {
        Send(msg.runtime_tid, (const char *)&msg.event, sizeof(msg.event),
             (char *)&runtime_reply, sizeof(runtime_reply));
    }

    Exit();
}

static void runtime_can_ingress_task(void) {
    int parent = MyParentTid();
    int rx_can_tid = WhoIs(CAN_SERVER_NAME);
    TrackRequest_t req;
    TrackReply_t reply;

    KASSERT(rx_can_tid >= 0);

    req.type = TRACK_REQ_CAN_FRAME;
    for (;;) {
        if (CANReceive(rx_can_tid, &req.frame, &req.now_us) == 0) {
            Send(parent, (const char *)&req, sizeof(req),
                 (char *)&reply, sizeof(reply));
        }
    }
}

static void track_notify_runtime(int runtime_tid, const runtime_event_t *event) {
    int courier_tid;
    TrackRuntimeNotifyMsg_t msg;
    TrackReply_t reply;

    if (runtime_tid < 0 || !event) return;

    courier_tid = Create(TRAIN_COURIER_PRIORITY, track_runtime_notify_task);
    KASSERT(courier_tid >= 0);

    msg.runtime_tid = runtime_tid;
    msg.event = *event;

    Send(courier_tid, (const char *)&msg, sizeof(msg),
         (char *)&reply, sizeof(reply));
}

static void track_local_process_can_frame(int runtime_tid,
                                          const can_frame_t *frame,
                                          uint64_t now_us) {
    uint8_t command = (uint8_t)((frame->id >> 17) & 0xFF);
    int is_response = (int)((frame->id >> 16) & 1);

    if (command == 0x11 && frame->dlc >= 5) {
        uint16_t sensor_id = ((uint16_t)frame->data[2] << 8) | frame->data[3];
        uint8_t state = frame->data[4];

        if (sensor_id > 0) {
            runtime_event_t event;

            track_local_log_sensor(sensor_id, now_us, state);
            ui_mark_sensors_dirty();

            if (state != 1) return;

            event.type = RUNTIME_EVENT_SENSOR_HIT;
            event.now_us = now_us;
            event.sensor_id = sensor_id;
            event.sensor_state = state;
            track_notify_runtime(runtime_tid, &event);
        }
        return;
    }

    if (command == 0x0B && is_response && frame->dlc >= 5) {
        uint32_t sw_id_raw = ((uint32_t)frame->data[0] << 24) |
                             ((uint32_t)frame->data[1] << 16) |
                             ((uint32_t)frame->data[2] << 8) |
                             (uint32_t)frame->data[3];
        int sw_num = (int)(sw_id_raw - 0x3000 + 1);
        char dir = (frame->data[4] == 0x01) ? 'S' : 'C';

        if (track_is_valid_switch(sw_num)) {
            runtime_event_t event;

            track_local_update_switch(sw_num, dir);
            ui_mark_switches_dirty();

            event.type = RUNTIME_EVENT_SWITCH_ACK;
            event.now_us = now_us;
            event.sw_num = sw_num;
            event.sw_dir = dir;
            track_notify_runtime(runtime_tid, &event);
        }
    }
}

void track_server_task(void) {
    int runtime_tid = MyParentTid();
    int tid;
    int ingress_tid = -1;
    TrackRequest_t req;
    TrackReply_t reply;

    RegisterAs(TRACK_SERVER_NAME);

    for (;;) {
        int msglen = Receive(&tid, (char *)&req, sizeof(req));
        (void)msglen;

        reply.status = 0;

        switch ((track_request_type_t)req.type) {
            case TRACK_REQ_INIT:
                track_bind_can_server(req.can_server_tid);
                track_init_graph();
                track_local_reset_state();
                KASSERT(req.can_server_tid >= 0);
                CANEnableInterrupts(req.can_server_tid);
                if (ingress_tid < 0) {
                    ingress_tid = Create(RUNTIME_CAN_INGRESS_PRIORITY,
                                         runtime_can_ingress_task);
                    KASSERT(ingress_tid >= 0);
                }
                track_local_bootstrap_defaults();
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_SET_SPEED:
                reply.status = track_local_set_speed(req.train, req.value);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_SEND_DIRECTION:
                reply.status = track_local_send_direction(req.train,
                                                          (uint8_t)req.value);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_REVERSE:
                reply.status = track_local_reverse(req.train);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_SET_SWITCH:
                reply.status = track_local_set_switch(req.value, req.dir);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_SET_LIGHT:
                reply.status = track_local_set_light(req.train, req.value);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_START_REVERSE:
                reply.status = track_local_start_reverse(req.train);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_COMPLETE_REVERSE:
                reply.status = track_local_complete_reverse(req.train);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_LOG_SENSOR:
                track_local_log_sensor(req.sensor_id, req.now_us, req.sensor_state);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_UPDATE_SWITCH:
                track_local_update_switch(req.value, req.dir);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_CAN_FRAME:
                track_local_process_can_frame(runtime_tid, &req.frame, req.now_us);
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            case TRACK_REQ_RESET_STARTUP:
                track_local_reset_state();
                track_local_bootstrap_defaults();
                ui_mark_sensors_dirty();
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;

            default:
                reply.status = -1;
                Reply(tid, (const char *)&reply, sizeof(reply));
                break;
        }
    }
}
