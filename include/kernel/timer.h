#ifndef _timer_h_
#define _timer_h_ 1

#include <stdint.h>

// System Timer CS (Control/Status) register bits
#define TIMER_CS_M0     (1 << 0)
#define TIMER_CS_M1     (1 << 1)
#define TIMER_CS_M2     (1 << 2)
#define TIMER_CS_M3     (1 << 3)

// read_timer(): Reads the 64-bit system timer 
uint64_t read_timer(void);

// Set system timer compare register C1
void timer_set_c1(uint32_t value);

// Set system timer compare register C3
void timer_set_c3(uint32_t value);

// Clear timer C1 interrupt 
void timer_clear_c1(void);

// Clear timer C3 interrupt 
void timer_clear_c3(void);

// Get current C1 compare value
uint32_t timer_get_c1(void);

// Get current C3 compare value
uint32_t timer_get_c3(void);

#endif
