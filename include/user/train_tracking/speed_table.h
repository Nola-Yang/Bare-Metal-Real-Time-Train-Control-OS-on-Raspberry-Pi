#ifndef _speed_table_h_
#define _speed_table_h_ 1

#include <stdint.h>
#include "track_node.h"

#define MAX_PHYSICAL_TRAINS 5
#define DEFAULT_SPEED_LEVEL 8

/* Lookup calibrated speed (mm/s). */
int32_t speed_table_get_v(int32_t train_ind, int user_speed);

/* Lookup the nominal calibrated deceleration (mm/s^2) without overrides. */
int32_t speed_table_get_nominal_decel(int32_t train_ind, int user_speed);

/* Lookup calibrated deceleration (mm/s^2).*/
int32_t speed_table_get_decel(int32_t train_ind, int user_speed, track_node *target);

/* Lookup the caliberated acceleration */
int32_t speed_table_get_accel(int32_t train_ind, int user_speed);

/* Lookup the caliberated early stopping time */
uint64_t speed_table_get_early_stop(int32_t train_ind, int user_speed);


#endif
