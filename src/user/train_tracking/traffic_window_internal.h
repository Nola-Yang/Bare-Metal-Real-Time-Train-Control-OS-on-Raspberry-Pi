#ifndef _traffic_window_internal_h_
#define _traffic_window_internal_h_

#include "train_tracking/position.h"

int traffic_window_build_prefix_plan(const uint16_t *path, int path_count,
                                     int start_cursor, int end_cursor,
                                     route_plan_t *out_plan);

#endif /* _traffic_window_internal_h_ */
