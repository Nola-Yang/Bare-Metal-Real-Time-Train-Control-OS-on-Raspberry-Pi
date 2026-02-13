#include "sensor_data.h"


void init_sensor_data(SensorData_t * sensor_data, char bank, uint32_t sensor_no, bool old_state, bool new_state) {
    sensor_data->bank = bank;
    sensor_data->sensor_no = sensor_no;
    sensor_data->old_state = old_state;
    sensor_data->new_state = new_state;
}

void default_init_sensor_data(SensorData_t* sensor_data) {
    init_sensor_data(sensor_data, 0, 0, false, false);
}

bool sensor_data_is_valid(SensorData_t *sensor_data) {
    return !(sensor_data->bank <= 0 && sensor_data->sensor_no <= 0);
}