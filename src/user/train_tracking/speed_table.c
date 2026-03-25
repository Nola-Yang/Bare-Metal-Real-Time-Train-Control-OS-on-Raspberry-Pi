#include "train_tracking/position_priv.h"
#include "train_tracking/speed_table.h"
#include <stdint.h>

#define MAX_SENSORS 80

#ifdef TRACK_D
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {227, 232, 242, 229, 230};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {144, 144, 144, 144, 144};

    static const int32_t GOTO_DECEL_OVERRIDE[MAX_SENSORS] =
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#else
    static const int32_t GOTO_SPEED_MM_S[MAX_PHYSICAL_TRAINS] =
        {226, 224, 226, 222, 236};
    static const int32_t GOTO_DECEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {167, 167, 167, 167, 167};

    static const int32_t GOTO_DECEL_OVERRIDE[MAX_SENSORS] =
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#endif

int32_t speed_table_get_v(int32_t train_ind, int user_speed) {
    if (user_speed != GOTO_USER_SPEED) return 0;
    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;
    return GOTO_SPEED_MM_S[train_ind];
}

int32_t speed_table_get_nominal_decel(int32_t train_ind, int user_speed) {
    if (user_speed != GOTO_USER_SPEED) return 0;
    if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return 0;
    return GOTO_DECEL_MM_S2[train_ind];
}

int32_t speed_table_get_decel(int32_t train_ind, int user_speed, track_node *target) {
    int32_t nominal = speed_table_get_nominal_decel(train_ind, user_speed);
    if (nominal <= 0) return 0;
    if (!target) return nominal;

    if (target->type == NODE_SENSOR) {
        int32_t override = GOTO_DECEL_OVERRIDE[target->num];
        return (override > -1) ? override : nominal;
    }
    return nominal;
}
