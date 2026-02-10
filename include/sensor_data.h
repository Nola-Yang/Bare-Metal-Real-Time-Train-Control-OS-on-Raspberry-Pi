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
} SensorData;


// init_sensor_data(sensor_data, bank, sensor_no, old_state, new_state): Constructs the data for a sensor
void init_sensor_data(SensorData * sensor_data, char bank, uint32_t sensor_no, bool old_state, bool new_state);

#endif
