#ifndef _clock_server_h_
#define _clock_server_h_ 1

#include <stdint.h>

#define CLOCK_SERVER_NAME "ClockServer"

// Message types
#define CLOCK_MSG_NOTIFY    0   // From notifier: tick occurred
#define CLOCK_MSG_TIME      1   // Request: get current time
#define CLOCK_MSG_DELAY     2   // Request: delay for ticks
#define CLOCK_MSG_DELAY_UNTIL 3 // Request: delay until tick

typedef struct {
    int type;
    int ticks;  // For DELAY: number of ticks; For DELAY_UNTIL: target tick
} ClockRequest_t;

typedef struct {
    int ticks;  // Current tick count
} ClockReply_t;


void clock_server_task(void);

// User API

// Returns the number of ticks since the clock server was created
// Returns: >= 0 tick count, -1 if clock server tid is invalid
int Time(int tid);

// Delays the calling task by the given number of ticks
// Returns: >= 0 on success, -1 if tid invalid, -2 if ticks < 0
int Delay(int tid, int ticks);

// Delays the calling task until the given tick
// Returns: >= 0 on success, -1 if tid invalid, -2 if ticks < 0
int DelayUntil(int tid, int ticks);

#endif