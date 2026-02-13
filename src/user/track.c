#include "track.h"
#include "mcp2515.h"
#include "can_server.h"
#include "timer.h"
#include "util.h"
#include "uart.h"
#include "kassert.h"
#include <stddef.h>

// Server TIDs for communication
static int can_tid = -1;
static int term_tid = -1;

// Track state storage
static switch_entry_t Switch_State[MAX_SWITCHES];
static sensor_entry_t Sensor_Log[SENSOR_LOG_SIZE];
static train_state_t Trains[MAX_ACTIVE_TRAINS];
static int Sensor_Log_Head = 0;


// Find train by number, returns NULL if not found
static train_state_t* find_train(int train_num) {
    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        if (Trains[i].train_num == train_num) return &Trains[i];
    }
    return NULL;
}

// Find or create train slot, returns NULL if full
static train_state_t* find_or_create_train(int train_num) {
    train_state_t* t = find_train(train_num);
    if (t) return t;
    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        if (Trains[i].train_num < 0) {
            Trains[i].train_num = train_num;
            Trains[i].speed = 0;
            Trains[i].direction = 0;
            Trains[i].rv_state = 0;
            return &Trains[i];
        }
    }
    return NULL;
}

void track_init(int can_server_tid, int term_server_tid) {
    can_tid = can_server_tid;
    term_tid = term_server_tid;

    for (int i = 0; i < MAX_SWITCHES; i++) {
        Switch_State[i].state = '?';
        Switch_State[i].last_update_us = 0;
    }

    for (int i = 0; i < SENSOR_LOG_SIZE; i++) {
        default_init_sensor_data(&(Sensor_Log[i].sensor_data));
    }
    Sensor_Log_Head = 0;

    for (int i = 0; i < MAX_ACTIVE_TRAINS; i++) {
        Trains[i].train_num = -1;
        Trains[i].speed = 0;
        Trains[i].direction = 0;
        Trains[i].rv_state = 0;
    }
}

void track_log_sensor(SensorData_t *sensor_data, uint64_t time_us) {
    memcpy(&(Sensor_Log[Sensor_Log_Head].sensor_data), sensor_data, sizeof(SensorData_t));
    Sensor_Log[Sensor_Log_Head].time_us = time_us;
    Sensor_Log_Head = (Sensor_Log_Head + 1) % SENSOR_LOG_SIZE;
}

void track_update_switch(int sw_num, char state) {
    int index = get_switch_ind(sw_num);
    if (index < 0) return;

    Switch_State[index].state = state;
    Switch_State[index].last_update_us = read_timer();
}

void track_update_speed(int train_num, int speed) {
    train_state_t* t = find_or_create_train(train_num);
    if (t) t->speed = speed;
}

void track_update_direction(int train_num, int direction) {
    train_state_t* t = find_or_create_train(train_num);
    if (t) t->direction = direction;
}

sensor_entry_t* track_get_sensor_log(int *head) {
    if (head) *head = Sensor_Log_Head;
    return Sensor_Log;
}

const switch_entry_t* track_get_switch_state(void) {
    return Switch_State;
}

const train_state_t* track_get_trains(void) {
    return Trains;
}

void track_set_speed(int train, int speed) {
    CanData_t frame;
    uint8_t frame_data[CAN_DATA_MAX_BYTE_LEN];

    init_marklin_speed_data_by_speed(&frame, train, speed, frame_data);

    int send_error = CANSend(can_tid, &frame);
    if (send_error) {
        panic("Error: CAN TX queue full\r\n");
        return;
    }

    train_state_t* t = find_or_create_train(train);
    if (!t) return;

    t->speed = speed;
    if (t->rv_state == 1) {
        t->rv_prev_speed = speed;
    }
}

void track_reverse(int train) {
    CanData_t frame;
    uint8_t frame_data[CAN_DATA_MAX_BYTE_LEN];

    init_marklin_reverse_data(&frame, train, frame_data);

    int send_error = CANSend(can_tid, &frame);
    if (send_error) {
        panic("Error: CAN TX queue full\r\n");
        return;
    }

    train_state_t* t = find_or_create_train(train);
    if (t) {
        t->direction = 1 - t->direction;
    }
}

void track_set_switch(int sw_num, char dir) {
    if (!is_valid_switch_no(sw_num)) {
        panic("Invalid switch number (1-18, 153-156)\r\n");
        return;
    }

    CanData_t frame;
    uint8_t frame_data[CAN_DATA_MAX_BYTE_LEN];
    bool is_curved = (dir == SWITCH_CURVED);

    init_marklin_switch_data(&frame, sw_num, is_curved, frame_data);

    if (CANSend(can_tid, &frame) != 0) {
        panic("Error: CAN TX queue full\r\n");
    }
}

void track_set_light(int train, int on) {
    KASSERT(on == 0 || on == 1);

    CanData_t frame;
    uint8_t frame_data[CAN_DATA_MAX_BYTE_LEN];

    init_marklin_light_data(&frame, train, frame_data, (bool)on);

    if (CANSend(can_tid, &frame) != 0) {
        panic("Error: CAN TX queue full\r\n");
    }
}

bool is_train_reversing(int train_num) {
    train_state_t *t = find_train(train_num);
    if (!t) return false;

    return t->rv_state != 0;
}

int get_train_rv_prev_speed(int train_num) {
    train_state_t *t = find_train(train_num);
    if (!t || t->rv_state == 0) return -1;

    return t->rv_prev_speed;
}

void track_reset_reverse(int train_num) {
    train_state_t *t = find_train(train_num);
    if (!t || t->rv_state == 0) return;

    t->rv_state = 0;
}

void track_complete_reverse(int train_num) {
    train_state_t *t = find_train(train_num);
    if (!t || t->rv_state == 0) return;

    int prev_speed = t->rv_prev_speed;
    track_reverse(train_num);
    track_set_speed(train_num, prev_speed);
}

int track_start_reverse(int train_num) {
    train_state_t* t = find_or_create_train(train_num);
    if (!t) {
        panic("No free train slots\r\n");
        return 0;
    }

    if (t->rv_state != 0) {
        // Already waiting for reverse to complete
        return 3;
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
