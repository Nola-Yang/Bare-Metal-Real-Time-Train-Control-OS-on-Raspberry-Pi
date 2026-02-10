#ifndef _can_data_h_
#define _can_data_h_

#include <stdint.h>
#include <stdbool.h>
#include "sensor_data.h"

// Sizes for a CAN data
#define CAN_ID_BYTE_LEN 4
#define CAN_DATA_MAX_BYTE_LEN 8

// Marklin command numbers
#define MARKLIN_CMD_TRAIN_DISCOVER 0x01
#define MARKLIN_CMD_ACCEL_TRAIN 0x04
#define MARKLIN_CMD_REVERSE_TRAIN 0x05
#define MARKLIN_CMD_TURN_ON_LIGHTS 0x06
#define MARKLIN_CMD_SWITCH 0x0B
#define MARKLIN_CMD_SENSOR 0x11
#define MARKLIN_CMD_PING 0x18
#define MARKLIN_CMD_BOOTLOAD 0x1B

#define SWITCH_CURVED 'C'
#define SWITCH_STRAIGHT 'S'


// CanData: Structure for use for data sent/received from the CAN protocol
typedef struct {
    uint8_t id[CAN_ID_BYTE_LEN];
    uint8_t length;
    uint8_t data[CAN_DATA_MAX_BYTE_LEN];
} CanData;


// init_empty_can_data(can_data, data): Constructs an empty CAN data
void init_empty_can_data(CanData *can_data, uint8_t *data);

// init_marklin_can_data(can_data, command, length, data, reponse): Constructs a CAN data used for the Marklin system
void init_marklin_can_data(CanData *can_data, uint8_t command, uint8_t length, uint8_t *data, bool response);

// init_marklin_reverse_data(can_data, train_no, data): Constructs CAN data for reversing a train
void init_marklin_reverse_data(CanData *can_data, uint32_t train_no, uint8_t *data);

// init_marklin_speed_data_by_step(can_data, train_no, speed_step, data): Constructs CAN data for speeding up a train
//  a train's speed step
void init_marklin_speed_data_by_step(CanData *can_data, uint32_t train_no, uint32_t speed_step, uint8_t *data);

// init_marklin_speed_data_by_step(can_data, train_no, speed_step, data): Constructs CAN data for speeding up a train
//   using the train's speed
void init_marklin_speed_data_by_speed(CanData *can_data, uint32_t train_no, uint16_t speed, uint8_t *data);

// init_marklin_light_data(can_data, train_no, data, is_on): Constructs CAN data for turning on the lights of the train
void init_marklin_light_data(CanData *can_data, uint32_t train_no, uint8_t *data, bool is_on);

// init_marklin_switch_data(can_data, switch_no, curved, data): Constructs CAN data for changing the direction of switches
void init_marklin_switch_data(CanData *can_data, uint32_t switch_no, bool curved, uint8_t *data);

// can_data_get_marklin_command(can_data): Retrieves the marklin command number from a CAN data
uint8_t can_data_get_markling_command(CanData *can_data);

// can_data_gen_sensor(can_data): Retrieves the data related to a sensor from a CAN data
void can_data_get_sensor(CanData *can_data, SensorData *sensor_data);

// can_data_log(can_data, line_no): Pretty prints CAN data for debugging purposes
void can_data_log(CanData *can_data, uint32_t line_no);

// can_switch_data_resp_confirm(can_data): Whether the CAN data for changing a switch direction has been confirmed
bool can_switch_data_resp_confirm(CanData *can_data);

// can_switch_data_get_switch_no(can_data): Retrieves the switch number from a CAN data that changes the switch direction
uint32_t can_switch_data_get_switch_no(CanData *can_data);

// can_switch_data_get_direction(can_data): Retrieves the direction the switch has changed to from a CAN data
char can_switch_data_get_direction(CanData *can_data);

// can_data_get_train_no(can_data): Retrieves the train number from a CAN data
uint32_t can_data_get_train_no(CanData *can_data);

// speed_step_to_speed(speed_step): Retrieves the train speed based on the train speed step
uint16_t speed_step_to_speed(uint32_t speed_step);

// can_data_get_speed(can_data): Retrieves the speed from a CAN data that accelerates the train
uint16_t can_data_get_speed(CanData *can_data);

#endif