#ifndef _traffic_window_internal_h_
#define _traffic_window_internal_h_

#include "train_tracking/position.h"

int traffic_window_build_prefix_plan(const uint16_t *path, int path_count,
                                     int start_cursor, int end_cursor,
                                     route_plan_t *out_plan);

int traffic_window_get_trailing_branch(const uint16_t *path, int path_count,
                                       int end_cursor,
                                       int *out_branch_idx,
                                       int *out_sw_num,
                                       char *out_sw_dir);

#endif /* _traffic_window_internal_h_ */
