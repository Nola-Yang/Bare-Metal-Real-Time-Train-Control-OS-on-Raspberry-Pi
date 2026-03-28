#include "track.h"
#include "server/track_server.h"
#include "server/track_server_internal.h"
#include "server/can_server.h"
#include "train_tracking/track_data.h"
#include "server/nameserver.h"
#include "syscall.h"
#include "ui.h"
#include "util.h"
#include "kassert.h"
#include <stddef.h>

/* Global track graph */
track_node g_track[TRACK_MAX];

static int can_tid = -1;
static int g_track_server_tid = -1;

/* Track state storage */
static switch_entry_t switch_state[MAX_SWITCHES];
static sensor_entry_t sensor_log[SENSOR_LOG_SIZE];
static train_state_t trains[MAX_ACTIVE_TRAINS];
static int sensor_log_head = 0;
static uint32_t g_switch_generation = 0;

static int train_num_to_active_index(int train_num) {
    if (13 <= train_num && train_num <= 15) return train_num - 13;
    if (17 <= train_num && train_num <= 18) return train_num - 14;
    if (train_num == 55) return 0; /* 55 reuses train-13 calibration/profile */
    return -1;
}

static int track_ensure_server_tid(void) {
    if (g_track_server_tid < 0) {
        g_track_server_tid = WhoIs(TRACK_SERVER_NAME);
    }
    return g_track_server_tid;
}

static int track_send_request(int tid, const TrackRequest_t *req, TrackReply_t *reply) {
    if (!req || !reply) return -1;
    return Send(tid, (const char *)req, sizeof(*req), (char *)reply, sizeof(*reply));
}

static train_state_t *find_train(int train_num) {
    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        if (trains[i].train_num == train_num) return &trains[i];
    }
    return NULL;
}

static train_state_t *find_or_create_train(int train_num) {
    if (!track_is_valid_train(train_num)) return NULL;

    train_state_t *t = find_train(train_num);
    if (t) return t;

    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        if (trains[i].train_num < 0) {
            trains[i].train_num = train_num;
            trains[i].speed = 0;
            trains[i].direction = 0;
            trains[i].rv_state = 0;
            trains[i].rv_prev_speed = 0;
            return &trains[i];
        }
    }

    return NULL;
}

int track_is_valid_train(int train_num) {
    return train_num_to_active_index(train_num) >= 0;
}

int track_switch_to_index(int sw_num) {
    if (sw_num >= 1 && sw_num <= 18) {
        return sw_num - 1;
    } else if (sw_num >= 153 && sw_num <= 156) {
        return 18 + (sw_num - 153);
    }
    return -1;
}

int track_index_to_switch(int index) {
    if (index >= 0 && index <= 17) {
        return index + 1;
    } else if (index >= 18 && index <= 21) {
        return 153 + (index - 18);
    }
    return -1;
}

int track_is_valid_switch(int sw_num) {
    return (sw_num >= 1 && sw_num <= 18) || (sw_num >= 153 && sw_num <= 156);
}

void track_bind_can_server(int can_server_tid) {
    can_tid = can_server_tid;
}

void track_init_graph(void) {
#ifdef TRACK_D
    init_trackb(g_track);
#else
    init_tracka(g_track);
#endif
}

void track_local_log_sensor(uint16_t sensor_id, uint64_t time_us, uint8_t state) {
    sensor_log[sensor_log_head].sensor_id = sensor_id;
    sensor_log[sensor_log_head].time_us = time_us;
    sensor_log[sensor_log_head].state = state;
    sensor_log_head = (sensor_log_head + 1) % SENSOR_LOG_SIZE;
}

void track_local_update_switch(int sw_num, char state) {
    int index;
    char prev;

    KASSERT(track_is_valid_switch(sw_num));

    index = track_switch_to_index(sw_num);
    if (index >= 0) {
        prev = switch_state[index].state;
        switch_state[index].state = state;
        if (prev != state) g_switch_generation++;
    }
}

static int track_local_send_frame(const can_frame_t *frame) {
    KASSERT(can_tid >= 0);
    return CANSend(can_tid, frame);
}

int track_local_send_direction(int train_num, uint8_t dir) {
    can_frame_t frame;

    frame.id = ((uint32_t)0 << 25) |
               ((uint32_t)0x05 << 17) |
               ((uint32_t)0 << 16) |
               (uint32_t)0xC300;
    frame.ext = 1;
    frame.dlc = 5;
    frame.data[0] = (train_num >> 24) & 0xFF;
    frame.data[1] = (train_num >> 16) & 0xFF;
    frame.data[2] = (train_num >> 8) & 0xFF;
    frame.data[3] = train_num & 0xFF;
    frame.data[4] = dir;
    return track_local_send_frame(&frame);
}

int track_local_set_speed(int train, int speed) {
    can_frame_t frame;
    train_state_t *t;

    if (!track_is_valid_train(train)) return -1;

    if (speed < 0) speed = 0;
    if (speed > 1000) speed = 1000;

    frame.id =
        ((uint32_t)0 << 25) |
        ((uint32_t)0x04 << 17) |
        ((uint32_t)0 << 16) |
        (uint32_t)0xC300;
    frame.ext = 1;
    frame.dlc = 6;
    frame.data[0] = (train >> 24) & 0xFF;
    frame.data[1] = (train >> 16) & 0xFF;
    frame.data[2] = (train >> 8) & 0xFF;
    frame.data[3] = train & 0xFF;
    frame.data[4] = (speed >> 8) & 0xFF;
    frame.data[5] = speed & 0xFF;

    if (track_local_send_frame(&frame) != 0) {
        KASSERT(0 && "CANsend fail in track_set_speed");
        return -1;
    }

    t = find_or_create_train(train);
    if (t) {
        if (t->rv_state == 1) {
            t->rv_prev_speed = speed;
        }
        t->speed = speed;
    }

    return 0;
}

int track_local_reverse(int train_num) {
    train_state_t *t;

    if (!track_is_valid_train(train_num)) return -1;
    if (track_local_send_direction(train_num, 0x03) != 0) return -1;

    t = find_or_create_train(train_num);
    if (t) t->direction = 1 - t->direction;
    return 0;
}

int track_local_set_switch(int sw_num, char dir) {
    can_frame_t frame;
    uint32_t switch_id;

    KASSERT(track_is_valid_switch(sw_num));
    KASSERT(dir == 'S' || dir == 'C');

    if (sw_num == 153 || sw_num == 154 || sw_num == 155 || sw_num == 156) {
        if (dir == 'S') return 0;
        switch (sw_num) {
            case 153: track_local_update_switch(154, 'S'); break;
            case 154: track_local_update_switch(153, 'S'); break;
            case 155: track_local_update_switch(156, 'S'); break;
            case 156: track_local_update_switch(155, 'S'); break;
        }
    }

    frame.id =
        ((uint32_t)0 << 25) |
        ((uint32_t)0x0B << 17) |
        ((uint32_t)0 << 16) |
        (uint32_t)0xC300;
    frame.ext = 1;
    frame.dlc = 6;

    switch_id = 0x3000 + (uint32_t)sw_num - 1;
    frame.data[0] = (switch_id >> 24) & 0xFF;
    frame.data[1] = (switch_id >> 16) & 0xFF;
    frame.data[2] = (switch_id >> 8) & 0xFF;
    frame.data[3] = switch_id & 0xFF;
    frame.data[4] = (dir == 'S') ? 0x01 : 0x00;
    frame.data[5] = 0x01;

    if (track_local_send_frame(&frame) != 0) {
        panic("CANsend fail in track_set_switch!\r\n");
        return -1;
    }

    return 0;
}

int track_local_set_light(int train, int on) {
    can_frame_t frame;

    if (!track_is_valid_train(train)) return -1;

    frame.id =
        ((uint32_t)0 << 25) |
        ((uint32_t)0x06 << 17) |
        ((uint32_t)0 << 16) |
        (uint32_t)0xC300;
    frame.ext = 1;
    frame.dlc = 6;
    frame.data[0] = (train >> 24) & 0xFF;
    frame.data[1] = (train >> 16) & 0xFF;
    frame.data[2] = (train >> 8) & 0xFF;
    frame.data[3] = train & 0xFF;
    frame.data[4] = 0x00;
    frame.data[5] = on ? 0x01 : 0x00;

    if (track_local_send_frame(&frame) != 0) {
        KASSERT(0 && "CANsend fail in track_set_light");
        return -1;
    }

    return 0;
}

int track_local_complete_reverse(int train_num) {
    train_state_t *t = find_train(train_num);
    if (t && t->rv_state == 1) {
        int prev_speed = t->rv_prev_speed;
        t->rv_state = 0;
        if (track_local_reverse(train_num) < 0) return -1;
        if (track_local_set_speed(train_num, prev_speed) < 0) return -1;
    }
    return 0;
}

int track_local_start_reverse(int train_num) {
    train_state_t *t;

    if (!track_is_valid_train(train_num)) return 0;

    t = find_or_create_train(train_num);
    if (!t) {
        KASSERT(0 && "No free train slots");
        return 0;
    }

    if (t->rv_state == 1) {
        return 0;
    }

    if (t->speed == 0) {
        if (track_local_reverse(train_num) < 0) return 0;
        return 2;
    }

    t->rv_prev_speed = t->speed;
    if (track_local_set_speed(train_num, 0) < 0) return 0;
    t->rv_state = 1;
    return 1;
}

void track_local_reset_state(void) {
    for (int i = 0; i < MAX_SWITCHES; i++) {
        switch_state[i].state = '?';
    }
    g_switch_generation = 0;

    for (int i = 0; i < SENSOR_LOG_SIZE; i++) {
        sensor_log[i].sensor_id = 0;
        sensor_log[i].time_us = 0;
        sensor_log[i].state = 0;
    }
    sensor_log_head = 0;

    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        trains[i].train_num = -1;
        trains[i].speed = 0;
        trains[i].direction = 0;
        trains[i].rv_state = 0;
        trains[i].rv_prev_speed = 0;
    }
}

void track_local_bootstrap_defaults(void) {
    static const int startup_trains[MAX_ACTIVE_TRAINS] = {13, 14, 15, 17, 18};

    for (int sw = 1; sw <= 18; sw++) {
        track_local_set_switch(sw, 'S');
    }
    for (int sw = 153; sw <= 156; sw++) {
        char state = (sw == 153 || sw == 155) ? 'C' : 'S';
        track_local_set_switch(sw, state);
    }
    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        track_local_set_speed(startup_trains[i], 0);
        track_local_set_light(startup_trains[i], 1);
    }

    ui_mark_switches_dirty();
}

int TrackServerInit(int tid, int can_server_tid) {
    TrackRequest_t req;
    TrackReply_t reply;

    req.type = TRACK_REQ_INIT;
    req.can_server_tid = can_server_tid;

    if (track_send_request(tid, &req, &reply) < 0) return -1;
    return reply.status;
}

void track_log_sensor(uint16_t sensor_id, uint64_t time_us, uint8_t state) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_LOG_SENSOR;
    req.sensor_id = sensor_id;
    req.now_us = time_us;
    req.sensor_state = state;

    KASSERT(track_send_request(tid, &req, &reply) >= 0);
}

void track_update_switch(int sw_num, char state) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_UPDATE_SWITCH;
    req.value = sw_num;
    req.dir = state;

    KASSERT(track_send_request(tid, &req, &reply) >= 0);
}

const sensor_entry_t *track_get_sensor_log(int *head) {
    if (head) *head = sensor_log_head;
    return sensor_log;
}

const switch_entry_t *track_get_switch_state(void) {
    return switch_state;
}

uint32_t track_get_switch_generation(void) {
    return g_switch_generation;
}

track_node *track_find_node(const char *name) {
    if (!name) return NULL;

    for (int i = 0; i < TRACK_MAX; i++) {
        const char *a;
        const char *b;

        if (g_track[i].type == NODE_NONE || g_track[i].name == NULL) continue;

        a = g_track[i].name;
        b = name;
        while (*a && *b && *a == *b) {
            a++;
            b++;
        }
        if (*a == '\0' && *b == '\0') return &g_track[i];
    }
    return NULL;
}

void track_set_speed(int train, int speed) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_SET_SPEED;
    req.train = train;
    req.value = speed;

    KASSERT(track_send_request(tid, &req, &reply) >= 0);
}

void track_reverse(int train_num) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_REVERSE;
    req.train = train_num;

    KASSERT(track_send_request(tid, &req, &reply) >= 0);
}

void track_send_direction(int train_num, uint8_t dir) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_SEND_DIRECTION;
    req.train = train_num;
    req.value = (int)dir;

    KASSERT(track_send_request(tid, &req, &reply) >= 0);
}

void track_set_switch(int sw_num, char dir) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_SET_SWITCH;
    req.value = sw_num;
    req.dir = dir;

    KASSERT(track_send_request(tid, &req, &reply) >= 0);
}

void track_set_light(int train, int on) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_SET_LIGHT;
    req.train = train;
    req.value = on;

    KASSERT(track_send_request(tid, &req, &reply) >= 0);
}

void track_complete_reverse(int train_num) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_COMPLETE_REVERSE;
    req.train = train_num;

    KASSERT(track_send_request(tid, &req, &reply) >= 0);
}

int track_start_reverse(int train_num) {
    TrackRequest_t req;
    TrackReply_t reply;
    int tid = track_ensure_server_tid();

    KASSERT(tid >= 0);

    req.type = TRACK_REQ_START_REVERSE;
    req.train = train_num;

    if (track_send_request(tid, &req, &reply) < 0) return 0;
    return reply.status;
}
