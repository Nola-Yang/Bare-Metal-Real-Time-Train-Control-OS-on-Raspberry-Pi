#include "sensor_data.h"


void init_sensor_data(SensorData * sensor_data, char bank, uint32_t sensor_no, bool old_state, bool new_state) {
    sensor_data->bank = bank;
    sensor_data->sensor_no = sensor_no;
    sensor_data->old_state = old_state;
    sensor_data->new_state = new_state;
}