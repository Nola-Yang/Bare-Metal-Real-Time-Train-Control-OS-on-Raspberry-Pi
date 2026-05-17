#ifndef _traffic_attr_internal_h_
#define _traffic_attr_internal_h_

#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "traffic_manager_internal.h"
#include <stddef.h>
#include <stdint.h>

#define ATTR_TIME_GATE_US      2500000ULL
#define ATTR_ALT_TIME_GATE_US  6000000ULL
#define ATTR_MAX_SKIP          2
#define ATTR_MARGIN            120
#define ATTR_SKIP_TIME_SLACK_US 500000ULL
#define ATTR_RESCUE_AMBIG_US   250000

static inline int abs64(int64_t x) {
    return (x < 0) ? (int)(-x) : (int)x;
}

static inline int abs_time_us(uint64_t a, uint64_t b) {
    return (a >= b) ? (int)(a - b) : (int)(b - a);
}

int sensor_hops_between(track_node *from_sensor, track_node *hit_sensor, int max_hops);
int current_leg_alt_branch_skip_to_hit(const train_pos_t *pos,
                                       track_node *hit,
                                       int32_t *out_dist_mm);
int route_path_alt_branch_skip_to_hit(const train_pos_t *pos,
                                      track_node *hit,
                                      int32_t *out_dist_mm);
int route_path_skip_to_hit(const train_pos_t *pos,
                           track_node *hit,
                           int32_t *out_dist_mm);

int sensors_form_adjacent_pair(track_node *first,
                               track_node *second,
                               int32_t *out_dist_mm);
train_pos_t *attr_pick_stale_train_for_pair(track_node *pending_sensor,
                                            track_node *current_hit,
                                            uint64_t now_us,
                                            int *out_ambiguous,
                                            uint8_t *out_revive_dead_track);

#endif /* _traffic_attr_internal_h_ */
