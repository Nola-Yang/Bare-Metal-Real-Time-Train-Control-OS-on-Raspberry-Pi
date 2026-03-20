#ifndef _demo_manager_h_
#define _demo_manager_h_ 1

#include <stdint.h>
#include "train_tracking/traffic_manager.h"

typedef struct {
    const char *mode_name;
    const char *state_name;
    uint32_t seed;
    uint32_t uptime_sec;
    int gold_min_trip_mm;
    traffic_sensor_stats_t sensor_stats;
} demo_ui_summary_t;

/* Initialize demo runtime state. */
void demo_init(void);

/* Handle `demo ...` command tokens.
 * Returns 1 on success/no extra output, 2 on usage or command output. */
int demo_handle_command(int argc, char *argv[]);

/* Periodic scheduler hook (10 ms). */
void demo_on_tick(uint64_t now_us);

/* Export summary fields for the on-screen UI. */
void demo_get_ui_summary(demo_ui_summary_t *out, uint64_t now_us);

/* Return 1 while demo gold is still auto-dispatching new targets. */
int demo_is_auto_dispatching_targets(void);

// get_demo_train_ind: Retrieves the index based on the train number
int get_demo_train_ind(int train_num);

int gold_dispatch_next_by_ind(int demo_train_ind);
int demo_retry_train_by_ind(int demo_train_ind);

#endif /* _demo_manager_h_ */
