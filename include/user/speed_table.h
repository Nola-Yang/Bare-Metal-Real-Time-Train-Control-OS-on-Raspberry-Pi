#ifndef _speed_table_h_
#define _speed_table_h_ 1

#include <stdint.h>

/*
 * Speed table extracted from Train Measurements.xlsx
 * Unit: mm/s  (= 1_000_000 / observed_us_per_mm)
 * Index 0 = speed step 0 (stopped), indices 1-14 = user speed steps 1-14.
 *
 * CAN speed <-> user speed:  can = (user==0)?0 : 1+(user-1)*77
 */

/* Straight track */
static const int32_t SPEED_V_STRAIGHT_MM_S[15] = {
    0,
    20, 31, 49, 76, 117, 162, 222, 288, 359, 444, 548, 622, 754, 869
};

/* Curved track */
static const int32_t SPEED_V_CURVE_MM_S[15] = {
    0,
    39, 48, 64, 78, 119, 164, 220, 287, 361, 438, 545, 624, 727, 833
};

/*
 * Braking distances in mm (placeholder — replace with measured values).
 * SPEED_STOP_DIST_MM[n] = estimated distance to stop from user speed n.
 */
static const int32_t SPEED_STOP_DIST_MM[15] = {
    0,
    100, 150, 220, 310, 420,
    550, 700, 200 , 1060, 1260,
    1480, 1720, 1980, 2260
};

#endif /* _speed_table_h_ */
