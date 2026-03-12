#include "track.h"
#include "train_tracking/track_data.h"
#include "mcp2515.h"
#include "server/can_server.h"
#include "timer.h"
#include "util.h"
#include "kassert.h"
#include <stddef.h>

/* Global track graph */
track_node g_track[TRACK_MAX];

// Server TIDs for communication
static int can_tid = -1;
static int term_tid = -1;

// Track state storage
static switch_entry_t switch_state[MAX_SWITCHES];
static sensor_entry_t sensor_log[SENSOR_LOG_SIZE];
static train_state_t trains[MAX_ACTIVE_TRAINS];
static int sensor_log_head = 0;

static int train_num_to_active_index(int train_num) {
    if (13 <= train_num && train_num <= 15) return train_num - 13;
    if (17 <= train_num && train_num <= 18) return train_num - 14;
    if (train_num == 55) return 0; /* 55 reuses train-13 calibration/profile */
    return -1;
}

int track_is_valid_train(int train_num) {
    return train_num_to_active_index(train_num) >= 0;
}


// Find train by number, returns NULL if not found
static train_state_t* find_train(int train_num) {
    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        if (trains[i].train_num == train_num) return &trains[i];
    }
    return NULL;
}

// Find or create train slot, returns NULL if full
static train_state_t* find_or_create_train(int train_num) {
    if (!track_is_valid_train(train_num)) return NULL;

    train_state_t* t = find_train(train_num);
    if (t) return t;
    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        if (trains[i].train_num < 0) {
            trains[i].train_num = train_num;
            trains[i].speed = 0;
            trains[i].direction = 0;
            trains[i].rv_state = 0;
            return &trains[i];
        }
    }
    return NULL;
}

// Switch number mapping functions
// User switch numbers: 1-18, 153-156
// Array indices: 0-21
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

void track_init_graph(void) {
#ifdef TRACK_D
    init_trackb(g_track);
#else
    init_tracka(g_track);
#endif
}

void track_init(int can_server_tid, int term_server_tid) {
    KASSERT(can_server_tid >= 0);
    KASSERT(term_server_tid >= 0);

    can_tid = can_server_tid;
    term_tid = term_server_tid;

    track_init_graph();

    for (int i = 0; i < MAX_SWITCHES; i++) {
        switch_state[i].state = '?';
        switch_state[i].last_update_us = 0;
    }

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
    }
}

void track_log_sensor(uint16_t sensor_id, uint64_t time_us, uint8_t state) {
    sensor_log[sensor_log_head].sensor_id = sensor_id;
    sensor_log[sensor_log_head].time_us = time_us;
    sensor_log[sensor_log_head].state = state;
    sensor_log_head = (sensor_log_head + 1) % SENSOR_LOG_SIZE;
}

void track_update_switch(int sw_num, char state) {
    KASSERT(track_is_valid_switch(sw_num));

    int index = track_switch_to_index(sw_num);
    if (index >= 0) {
        switch_state[index].state = state;
        switch_state[index].last_update_us = read_timer();
    }
}

void track_update_speed(int train_num, int speed) {
    if (!track_is_valid_train(train_num)) return;
    train_state_t* t = find_or_create_train(train_num);
    if (t) t->speed = speed;
}

void track_update_direction(int train_num, int direction) {
    if (!track_is_valid_train(train_num)) return;
    train_state_t* t = find_or_create_train(train_num);
    if (t) t->direction = direction;
}


const sensor_entry_t* track_get_sensor_log(int *head) {
    if (head) *head = sensor_log_head;
    return sensor_log;
}

const switch_entry_t* track_get_switch_state(void) {
    return switch_state;
}

const train_state_t* track_get_trains(void) {
    return trains;
}

void track_set_speed(int train, int speed) {
    KASSERT(can_tid >= 0);
    if (!track_is_valid_train(train)) return;

    can_frame_t frame;

    if (speed < 0) speed = 0;
    if (speed > 1000) speed = 1000;

    // 29-bit CAN Identifier
    uint8_t  priority = 0;
    uint8_t  command  = 0x04;   // Speed
    uint8_t  response = 0;
    uint16_t hash     = 0xC300;

    frame.id =
        ((uint32_t)priority << 25) |
        ((uint32_t)command  << 17) |
        ((uint32_t)response << 16) |
        (uint32_t)hash;

    frame.ext = 1;
    frame.dlc = 6;

    frame.data[0] = (train >> 24) & 0xFF;
    frame.data[1] = (train >> 16) & 0xFF;
    frame.data[2] = (train >>  8) & 0xFF;
    frame.data[3] =  train        & 0xFF;
    frame.data[4] = (speed >> 8) & 0xFF;
    frame.data[5] = speed & 0xFF;

    if (CANSend(can_tid, &frame) == 0) {
        train_state_t* t = find_or_create_train(train);
        if (t) {
            if (t->rv_state == 1) {
                t->rv_prev_speed = speed;
            }
            t->speed = speed;
        }
    } else {
        KASSERT(0 && "CANsend fail in track_set_speed");
    }
}

void track_reverse(int train) {
    KASSERT(can_tid >= 0);
    if (!track_is_valid_train(train)) return;

    can_frame_t frame;

    uint8_t  priority = 0;
    uint8_t  command  = 0x05;   // Direction
    uint8_t  response = 0;
    uint16_t hash     = 0xC300;

    frame.id =
        ((uint32_t)priority << 25) |
        ((uint32_t)command  << 17) |
        ((uint32_t)response << 16) |
        (uint32_t)hash;

    frame.ext = 1;
    frame.dlc = 5;

    frame.data[0] = (train >> 24) & 0xFF;
    frame.data[1] = (train >> 16) & 0xFF;
    frame.data[2] = (train >>  8) & 0xFF;
    frame.data[3] =  train        & 0xFF;
    frame.data[4] = 0x03;  

    if (CANSend(can_tid, &frame) == 0) {
        train_state_t* t = find_or_create_train(train);
        if (t) t->direction = 1 - t->direction;
    } else {
        KASSERT(0 && "CANsend fail in track_reverse");
    }
}

void track_set_switch(int sw_num, char dir) {
    KASSERT(can_tid >= 0);
    KASSERT(track_is_valid_switch(sw_num));
    KASSERT(dir == 'S' || dir == 'C');

    /* 153/154 and 155/156 cannot both be C. */
    if (dir == 'C') {
        int partner = (sw_num == 153) ? 154 :
                      (sw_num == 154) ? 153 :
                      (sw_num == 155) ? 156 :
                      (sw_num == 156) ? 155 : -1;
        if (partner > 0) {
            int pidx = track_switch_to_index(partner);
            if (pidx >= 0 && switch_state[pidx].state == 'C') {
                track_set_switch(partner, 'S');
                track_update_switch(partner, 'S');
            }
        }
    }

    can_frame_t frame;

    uint8_t  priority = 0;
    uint8_t  command  = 0x0B;   // Switch
    uint8_t  response = 0;
    uint16_t hash     = 0xC300;

    frame.id =
        ((uint32_t)priority << 25) |
        ((uint32_t)command  << 17) |
        ((uint32_t)response << 16) |
        (uint32_t)hash;

    frame.ext = 1;
    frame.dlc = 6;

    // Switch ID (Byte 0-3): 0x3000 + switch_number - 1
    uint32_t switch_id = 0x3000 + sw_num - 1;
    frame.data[0] = (switch_id >> 24) & 0xFF;
    frame.data[1] = (switch_id >> 16) & 0xFF;
    frame.data[2] = (switch_id >>  8) & 0xFF;
    frame.data[3] =  switch_id        & 0xFF;

    // Position (Byte 4): 0=curved, 1=straight
    frame.data[4] = (dir == 'S') ? 0x01 : 0x00;

    // Engage (Byte 5): 1 = engage solenoid
    frame.data[5] = 0x01;

    if (CANSend(can_tid, &frame) != 0) {
        panic("CANsend fail in track_set_switch!\r\n");
    }
}

void track_set_light(int train, int on) {
    KASSERT(can_tid >= 0);
    KASSERT(on == 0 || on == 1);
    if (!track_is_valid_train(train)) return;

    can_frame_t frame;

    uint8_t  priority = 0;
    uint8_t  command  = 0x06;   // Light
    uint8_t  response = 0;
    uint16_t hash     = 0xC300;

    frame.id =
        ((uint32_t)priority << 25) |
        ((uint32_t)command  << 17) |
        ((uint32_t)response << 16) |
        (uint32_t)hash;

    frame.ext = 1;
    frame.dlc = 6;

    frame.data[0] = (train >> 24) & 0xFF;
    frame.data[1] = (train >> 16) & 0xFF;
    frame.data[2] = (train >>  8) & 0xFF;
    frame.data[3] =  train        & 0xFF;

    // Function (Byte 4): 0 = light
    frame.data[4] = 0x00;

    // State (Byte 5): 0=off, 1=on
    frame.data[5] = on ? 0x01 : 0x00;

    if (CANSend(can_tid, &frame) != 0) {
        KASSERT(0 && "CANsend fail in track_set_light");
    }
}


void track_complete_reverse(int train_num) {
    train_state_t *t = find_train(train_num);
    if (t && t->rv_state == 1) {
        int prev_speed = t->rv_prev_speed;
        t->rv_state = 0;
        track_reverse(train_num);
        track_set_speed(train_num, prev_speed);
    }
}

int track_start_reverse(int train_num) {
    if (!track_is_valid_train(train_num)) return 0;

    train_state_t* t = find_or_create_train(train_num);
    if (!t) {
        KASSERT(0 && "No free train slots");
        return 0;
    }

    if (t->rv_state == 1) {
        // Already waiting for reverse to complete
        return 0;
    }

    if (t->speed == 0) {
        track_reverse(train_num);
        return 2;  // immediate reverse, no delay needed
    }

    t->rv_prev_speed = t->speed;
    track_set_speed(train_num, 0);
    t->rv_state = 1;

    return 1;  // delay needed
}
