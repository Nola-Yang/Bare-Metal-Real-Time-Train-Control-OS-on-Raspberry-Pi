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
 * Polynomial gives us/mm: f(x) = 1.335e-08x^5 - 5.583e-07x^4 + 8.883e-06x^3 - 6.228e-05x^2 + 0.0002327x - 0.000273
 * Speed (mm/s) = 1,000,000 / f(x),  x = user speed step (1–14); index 0 = 0.
 */
extern int32_t SPEED_V_MM_S[15];

/*
 * Braking distances in mm.
 * SPEED_STOP_DIST_MM[n] = estimated distance to stop from user speed n.
 * Filled at runtime by init_braking_table() using the 5th-degree polynomial:
 *   dist(x) = -0.02836x^5 + 1.244x^4 - 19.68x^3 + 149.5x^2 - 487.8x + 636  (mm, x = speed step)
 */
extern int32_t SPEED_STOP_DIST_MM[15];

#endif /* _speed_table_h_ */
