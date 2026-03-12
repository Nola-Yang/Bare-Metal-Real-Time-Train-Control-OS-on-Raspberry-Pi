#ifndef _demo_manager_h_
#define _demo_manager_h_ 1

#include <stdint.h>

/* Initialize demo runtime state. */
void demo_init(void);

/* Handle `demo ...` command tokens.
 * Returns 1 on success/no extra output, 2 on usage or command output. */
int demo_handle_command(int argc, char *argv[]);

/* Periodic scheduler hook (10 ms). */
void demo_on_tick(uint64_t now_us);

/* Called when a train reaches STOPPED after a goto completion. */
void demo_on_train_stopped(int train_num, uint64_t now_us);

/* Optional hook for future sensor-anomaly notification throttling. */
void demo_on_sensor_stats_changed(void);

#endif /* _demo_manager_h_ */
