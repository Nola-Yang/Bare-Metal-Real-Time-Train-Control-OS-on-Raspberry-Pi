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
 * Polynomial gives mm/us: f(x) = 1.335e-08x^5 - 5.583e-07x^4 + 8.883e-06x^3 - 6.228e-05x^2 + 0.0002327x - 0.000273
 * Speed (mm/s) = f(x) * 1,000,000,  x = user speed step (1–14); index 0 = 0.
 */

#define MAX_PHYSICAL_TRAINS 5
extern int32_t SPEED_V_MM_S[MAX_PHYSICAL_TRAINS][15];

/*
 * Model:  d_brake = effective_v² / (2 * SPEED_DECEL_MM_S2[speed])
 *         t_stop  = effective_v  / SPEED_DECEL_MM_S2[speed]
 */
extern int32_t SPEED_DECEL_MM_S2[MAX_PHYSICAL_TRAINS][15];


#endif
