#include "train_tracking/position_priv.h"
#include "train_tracking/speed_table.h"
#include <stdint.h>

#define MAX_SENSORS 80
#define NUM_OF_SPEED_LEVELS 2

#ifdef TRACK_D
    static const int32_t GOTO_SPEED_MM_S[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{227, 232, 242, 229, 230},
         {227, 232, 242, 229, 230}};

    static const int32_t GOTO_DECEL_MM_S2[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{144, 144, 144, 144, 144},
         {144, 144, 144, 144, 144}};

    static const int32_t GOTO_ACCEL_MM_S2[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{37, 37, 38, 38, 38},
         {37, 37, 38, 38, 38}};
#else
    static const int32_t GOTO_SPEED_MM_S[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{226, 224, 226, 222, 236},
         {226, 224, 226, 222, 236}};

    static const int32_t GOTO_DECEL_MM_S2[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{167, 167, 167, 167, 167},
         {167, 167, 167, 167, 167}};

    static const int32_t GOTO_ACCEL_MM_S2[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{37, 37, 38, 38, 38},
         {37, 37, 38, 38, 38}};
#endif


static int get_speed_ind(int speed_level) {
    switch (speed_level) {
        case 8:
            return 0;
        case 10:
            return 1;
        default:
            return -1;
    }
}

int32_t speed_table_get_v(int32_t train_ind, int user_speed) {
    int speed_ind = get_speed_ind(user_speed);
    if (speed_ind == -1) return 0;

    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;
    return GOTO_SPEED_MM_S[speed_ind][train_ind];
}

int32_t speed_table_get_nominal_decel(int32_t train_ind, int user_speed) {
    int speed_ind = get_speed_ind(user_speed);
    if (speed_ind == -1) return 0;

    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;
    return GOTO_DECEL_MM_S2[speed_ind][train_ind];
}

int32_t speed_table_get_decel(int32_t train_ind, int user_speed) {
    int32_t nominal = speed_table_get_nominal_decel(train_ind, user_speed);
    if (nominal <= 0) return 0;
    return nominal;
}

int32_t speed_table_get_accel(int32_t train_ind, int user_speed) {
    int speed_ind = get_speed_ind(user_speed);
    if (speed_ind == -1) return 0;

    return GOTO_ACCEL_MM_S2[speed_ind][train_ind];
}
