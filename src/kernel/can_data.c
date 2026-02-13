#include <stdbool.h>
#include <string.h>
#include "uart.h"
#include "util.h"
#include "can_data.h"


static const uint8_t EXIDE_MASK = 0x08;

// Masks for putting the correct bits into the SIDL register
static const uint8_t MARKLIN_CMD_TO_SIDL_TOP_MASK = 0x0E;
static const uint8_t MARKLIN_CMD_TO_SIDL_BTM_MASK = 0x01;
static const uint8_t SIDL_TO_MARKLIN_CMD_TOP_MASK = 0xE0;
static const uint8_t SIDL_TO_MARKLIN_CMD_BTM_MASK = 0x02;

// the magic marklin hash
static const uint8_t MARKLIN_HASH_TOP = 0xC3;
static const uint8_t MARKLIN_HASH_BTM = 0x00;

// a different hash for sending out a response
static const uint8_t MARKLIN_RESP_HASH_TOP = 0xC3;
static const uint8_t MARKLIN_RESP_HASH_BTM = 0x01;

static const uint8_t SENSORS_PER_BANK = 16;

static uint32_t MIN_MIDDLE_SWITCH_NO = 153;
static uint32_t MAX_MIDDLE_SWITCH_NO = 156;
static uint32_t MIN_REGULAR_SWITCH_NO = 1;
static uint32_t MAX_REGULAR_SWITCH_NO = 18;

static const uint32_t SWITCH_NO_OFFSET = 0x3000;


void init_empty_can_data(CanData_t *can_data, uint8_t *data) {
    can_data->length = 0;
    memcpy(can_data->data, data, CAN_DATA_MAX_BYTE_LEN);
}

void init_marklin_can_data(CanData_t *can_data, uint8_t command, uint8_t length, uint8_t *data, bool response) {
    // SIDH format:
    //  0000, CMD[7:4]
    uint8_t sidh = 0 | (command >> 4);

    // SIDL format:
    //  CMD[3:1], 0, EXIDE, 0, CMD[0], response indicator
    uint8_t response_bit = (response) ? 0x01 : 0x00;
    uint8_t sidl = ((command & MARKLIN_CMD_TO_SIDL_TOP_MASK) << 4) | EXIDE_MASK | ((command & MARKLIN_CMD_TO_SIDL_BTM_MASK) << 1) | response_bit;

    uint8_t eid8;
    uint8_t eid0;

    if (response) {
        eid8 = MARKLIN_RESP_HASH_TOP;
        eid0 = MARKLIN_RESP_HASH_BTM;
    } else {
        eid8 = MARKLIN_HASH_TOP;
        eid0 = MARKLIN_HASH_BTM;
    }

    uint8_t *id =  can_data->id;
    id[0] = sidh;
    id[1] = sidl;
    id[2] = eid8;
    id[3] = eid0;

    can_data->length = length;
    memcpy(can_data->data, data, CAN_DATA_MAX_BYTE_LEN);
}

// set_train_no(data, train_no): Sets the train number for the data portion of a CAN data
static void set_train_no(uint8_t *data, uint32_t train_no) {
    uint32_t mask = 0xFF000000;

    for (uint8_t i = 0; i < CAN_ID_BYTE_LEN; ++i) {
        data[i] = (train_no & mask) >> ((CAN_ID_BYTE_LEN - 1 - i) * 8);
        mask = mask >> 8;
    }
}

// set_switch_no(data, switch_no): Sets the switch number for the data portion of a CAN data
static void set_switch_no(uint8_t *data, uint32_t switch_no) {
    switch_no = max_uint(0, switch_no - 1);
    switch_no += SWITCH_NO_OFFSET;
    set_train_no(data, switch_no);
}


// get_train_no(data): Retrieves the train number from the data portion of a CAN data
static uint32_t get_train_no(uint8_t *data) {
    uint32_t result = ((uint32_t)data[0] << 24) |
                      ((uint32_t)data[1] << 16) |
                      ((uint32_t)data[2] << 8)  |
                      ((uint32_t)data[3]);
    return result;
}

// get_speed(data): Retrieves the speed level from the data portion of a CAN data
static uint16_t get_speed(uint8_t *data) {
    uint16_t result = ((uint32_t)data[4] << 8)  |
                      ((uint32_t)data[5]);
    return result;
}

// data_get_switch_no(data): Retrieves the switch number from the data portion of a CAN data
static uint32_t data_get_switch_no(uint8_t *data) {
    uint32_t result = get_train_no(data);
    return result + 1 - SWITCH_NO_OFFSET;
}

void init_marklin_light_data(CanData_t *can_data, uint32_t train_no, uint8_t *data, bool is_on) {
    set_train_no(data, train_no);
    data[4] = 0;
    data[5] = is_on;
    init_marklin_can_data(can_data, MARKLIN_CMD_TURN_ON_LIGHTS, 6, data, false);
}

uint16_t speed_step_to_speed(uint32_t speed_step) {
    if (speed_step == 0) return 0;

    uint16_t result = 1 + (speed_step - 1) * 77;
    result = min_uint(result, 1000);
    return result;
}

void init_marklin_speed_data_by_step(CanData_t *can_data, uint32_t train_no, uint32_t speed_step, uint8_t *data) {
    uint16_t speed = speed_step_to_speed(speed_step);
    init_marklin_speed_data_by_speed(can_data, train_no, speed, data);
}

void init_marklin_speed_data_by_speed(CanData_t *can_data, uint32_t train_no, uint16_t speed, uint8_t *data) {
    set_train_no(data, train_no);

    data[4] = (speed & 0xFF00) >> 8;
    data[5] = speed & 0x00FF;

    init_marklin_can_data(can_data, MARKLIN_CMD_ACCEL_TRAIN, 6, data, false);
}

void init_marklin_reverse_data(CanData_t *can_data, uint32_t train_no, uint8_t *data) {
    set_train_no(data, train_no);
    data[4] = 3;
    init_marklin_can_data(can_data, MARKLIN_CMD_REVERSE_TRAIN, 5, data, false);
}

void init_marklin_switch_data(CanData_t *can_data, uint32_t switch_no, bool curved, uint8_t *data) {
    set_switch_no(data, switch_no);
    data[4] = (curved) ? 0 : 1;
    data[5] = 1;
    init_marklin_can_data(can_data, MARKLIN_CMD_SWITCH, 6, data, false);
}

uint8_t can_data_get_marklin_command(CanData_t *can_data) {
    uint8_t command_top = (can_data->id[0] << 4);

    uint8_t sidl = can_data->id[1];
    uint8_t command_btm = ((sidl & SIDL_TO_MARKLIN_CMD_TOP_MASK) >> 4) | ((sidl & SIDL_TO_MARKLIN_CMD_BTM_MASK) >> 1);

    return command_top | command_btm;
}

bool is_marklin_sensor_data(CanData_t *can_data) {
    uint8_t command = can_data_get_marklin_command(can_data); 
    return (command == MARKLIN_CMD_SENSOR && can_data->length >= 5);
}

void can_data_get_sensor(CanData_t *can_data, SensorData_t *sensor_data) {
    uint8_t *data = can_data-> data;
    int16_t sensor_id = ((int16_t)data[2] << 8) | data[3];

    if (sensor_id <= 0) {
        sensor_data->bank = 0;
        sensor_data->sensor_no = 0;
    } else {
        sensor_id -= 1;
        sensor_data->bank = (sensor_id / SENSORS_PER_BANK) + 'A';
        sensor_data->sensor_no = (sensor_id % SENSORS_PER_BANK) + 1;
    }

    sensor_data->old_state = data[4];
    sensor_data->new_state = data[5];
}

bool can_switch_data_resp_confirm(CanData_t *can_data) {
    return can_data->data[5] == 0x01;
}

uint32_t can_switch_data_get_switch_no(CanData_t *can_data) {
    return data_get_switch_no(can_data->data);
}

char can_switch_data_get_direction(CanData_t *can_data) {
    if (can_data->data[4]) {
        return SWITCH_STRAIGHT;
    }

    return SWITCH_CURVED;
}

uint32_t can_data_get_train_no(CanData_t *can_data){
    return get_train_no(can_data->data);
}

uint16_t can_data_get_speed(CanData_t *can_data) {
    return get_speed(can_data->data);
}

bool is_valid_switch_no(uint32_t switch_no) {
    return (MIN_REGULAR_SWITCH_NO <= switch_no && switch_no <= MAX_REGULAR_SWITCH_NO) || 
           (MIN_MIDDLE_SWITCH_NO <= switch_no && switch_no <= MAX_MIDDLE_SWITCH_NO);
}

uint32_t get_switch_no(uint32_t switch_ind) {
    if (switch_ind < MAX_REGULAR_SWITCH_NO) {
        return switch_ind + 1;
    } else if (switch_ind < SWITCH_COUNT) {
        return switch_ind + 135;
    }

    return -1;
}

uint32_t get_switch_ind(uint32_t switch_no) {
    if (switch_no <= MAX_REGULAR_SWITCH_NO) {
        return switch_no - 1;
    } else if (MIN_MIDDLE_SWITCH_NO <= switch_no && switch_no <= MAX_MIDDLE_SWITCH_NO) {
        return switch_no - 135;
    }

    return -1;
}