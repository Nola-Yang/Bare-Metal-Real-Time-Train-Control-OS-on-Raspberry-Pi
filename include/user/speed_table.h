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

/*
 * speed table (mm/s).  Filled at runtime by init_speed_table() in position.c.
 * Polynomial gives us/mm: f(x) = -0.3358x^5 + 17.71x^4 - 375.3x^3 + 4053x^2 - 22980x + 58520
 * Speed (mm/s) = 1,000,000 / f(x),  x = user speed step (1–14); index 0 = 0.
 */
static int32_t SPEED_V_MM_S[15];

/*
 * Braking distances in mm.
 * SPEED_STOP_DIST_MM[n] = estimated distance to stop from user speed n.
 * Filled at runtime by speed_table_init_braking() using the cubic formula:
 *   dist(x) = 1.463*x^3 - 21.19*x^2 + 148.8*x - 216  (mm, x = speed step)
 */
static int32_t SPEED_STOP_DIST_MM[15];

#endif /* _speed_table_h_ */
