#ifndef _sensor_data_h_
#define _sensor_data_h_ 1


#include <stdint.h>
#include <stdbool.h>


// SensorData: Stucture to store data related to the sensors
typedef struct {
    char bank;
    uint32_t sensor_no;
    bool old_state;
    bool new_state;
} SensorData_t;


// init_sensor_data(sensor_data, bank, sensor_no, old_state, new_state): Constructs the data for a sensor
void init_sensor_data(SensorData_t * sensor_data, char bank, uint32_t sensor_no, bool old_state, bool new_state);

// default_init_sensor_data(sensor_data): Default constructs an empty data for a sensor
void default_init_sensor_data(SensorData_t* sensor_data);

// sensor_data_is_valid(sensor_data): Determines whether a sensor data is valid
bool sensor_data_is_valid(SensorData_t *sensor_data);

#endif
