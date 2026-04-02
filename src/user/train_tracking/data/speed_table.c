#include "train_tracking/position_priv.h"
#include "train_tracking/speed_table.h"
#include <stdint.h>

#ifdef TRACK_D
    static const int32_t GOTO_SPEED_MM_S[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{227, 232, 242, 229, 230},
         {365, 365, 365, 365, 365}};

    static const int32_t GOTO_DECEL_MM_S2[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{150, 150, 150, 150, 150},
         {174, 174, 174, 174, 174 }};

    static const int32_t GOTO_DECEL_OVERRIDE[NUM_OF_SPEED_LEVELS][MAX_SENSORS] = 
    {{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
     {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

    static const int32_t GOTO_ACCEL_MM_S2[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{46, 46, 46, 46, 46},
         {90, 90, 90, 90, 90}};

    uint64_t STOP_EARLY_US[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{930000ULL, 930000ULL, 930000ULL, 930000ULL, 930000ULL},
         {800000ULL, 800000ULL, 800000ULL, 800000ULL, 800000ULL}};

    static const int32_t NODE_OFFSET_OVERRIDE[MAX_SENSORS] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -200, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
#else
    static const int32_t GOTO_SPEED_MM_S[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{227, 232, 242, 229, 230},
         {365, 365, 365, 365, 365}};

    static const int32_t GOTO_DECEL_MM_S2[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{144, 144, 144, 144, 144},
         {174, 174, 174, 174, 174 }};

    static const int32_t GOTO_DECEL_OVERRIDE[NUM_OF_SPEED_LEVELS][MAX_SENSORS] = 
    {{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
     {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

    static const int32_t GOTO_ACCEL_MM_S2[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{590, 590, 590 , 590, 590},
         {90, 90, 90, 90, 90}};

    uint64_t STOP_EARLY_US[NUM_OF_SPEED_LEVELS][MAX_PHYSICAL_TRAINS] =
        {{800000ULL, 800000ULL, 800000ULL, 800000ULL, 800000ULL},
         {800000ULL, 800000ULL, 800000ULL, 800000ULL, 800000ULL}};

    static const int32_t NODE_OFFSET_OVERRIDE[MAX_SENSORS] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
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

int32_t speed_table_get_decel(int32_t train_ind, int user_speed, track_node *target) {
    int32_t nominal = speed_table_get_nominal_decel(train_ind, user_speed);
    if (nominal <= 0) return 0;

    if (target->type == NODE_SENSOR) {
        int speed_ind = get_speed_ind(user_speed);
        if (speed_ind == -1) return 0;

        int32_t override = GOTO_DECEL_OVERRIDE[speed_ind][target->num];
        return (override > -1) ? override : nominal;
    }

    return nominal;
}

int32_t speed_table_get_accel(int32_t train_ind, int user_speed) {
    int speed_ind = get_speed_ind(user_speed);
    if (speed_ind == -1) return 0;

    return GOTO_ACCEL_MM_S2[speed_ind][train_ind];
}

uint64_t speed_table_get_early_stop(int32_t train_ind, int user_speed) {
    int speed_ind = get_speed_ind(user_speed);
    if (speed_ind == -1) return 0;

    return STOP_EARLY_US[speed_ind][train_ind];
}

int32_t node_get_override_offset(track_node *target) {
    if (target->type != NODE_SENSOR) return 0;
    return NODE_OFFSET_OVERRIDE[target->num];
}