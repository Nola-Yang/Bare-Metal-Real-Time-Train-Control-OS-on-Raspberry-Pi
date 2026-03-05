#ifndef _speed_table_h_
#define _speed_table_h_ 1

#include <stdint.h>

#define MAX_PHYSICAL_TRAINS 5

/* Lookup calibrated speed (mm/s). */
int32_t speed_table_get_v(int32_t train_ind, int user_speed);

/* Lookup calibrated deceleration (mm/s^2).*/
int32_t speed_table_get_decel(int32_t train_ind, int user_speed);


#endif
