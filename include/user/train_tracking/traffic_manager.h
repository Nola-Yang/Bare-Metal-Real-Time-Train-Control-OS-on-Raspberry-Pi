#ifndef _traffic_manager_h_
#define _traffic_manager_h_

#include <stdint.h>
#include "train_tracking/position.h"

typedef struct {
    int spurious_count;
    int ambiguous_count;
    uint16_t last_spurious_sensor_id;
    uint16_t last_ambiguous_sensor_id;
    uint64_t last_spurious_time_us;
    uint64_t last_ambiguous_time_us;
} traffic_sensor_stats_t;

/* Initialize reservation/attribution state. */
void traffic_init(void);

/* Build blocked-node constraints for requester_train route planning. */
void traffic_build_constraints(int requester_train, uint8_t blocked[TRACK_MAX]);

/* Reserve a plan's node path for a train. Returns 1 on success, 0 on conflict. */
int traffic_reserve_plan(int train_num, track_node *start, const route_plan_t *plan);

/* Return 1 if the plan can be reserved by train_num without conflict. */
int traffic_can_reserve_plan(int train_num, const route_plan_t *plan);

/* Release all reservations owned by train_num. */
void traffic_release_train(int train_num);

/* Release all reservations except the current travel window:
 * keep `body_mm` behind `last_hit`, the track from `last_hit` to `next_hit`
 * (if reachable), and `body_mm` ahead of `next_hit`, all relative to the
 * train's travel direction. */
void traffic_release_train_keep_body(int train_num, track_node *last_hit,
                                     int32_t body_mm, track_node *next_hit);

/* Refresh an active route reservation from the remaining planned path. */
void traffic_refresh_route_reservation(int train_num, track_node *cur_sensor,
                                       track_node *next_hit,
                                       const uint16_t *path, int path_cursor,
                                       int path_count);

/* Release all nodes before prev_sensor owned by train_num.
 * Walks backward from prev_sensor; nodes at prev_sensor and ahead are untouched. */
void traffic_release_before_sensor(int train_num, track_node *prev_sensor);

/* Check if it is safe to set a switch.
 * Any reservation on the switch envelope blocks the change, including self-owned
 * reservations. Returns -1 when safe; otherwise returns the blocking train number. */
int traffic_can_set_switch(int sw_num, int requester_train);

/* Attribute a sensor hit to the best train, or NULL if spurious/ambiguous.
 * Updates per-train attribution diagnostics fields. */
train_pos_t *traffic_attribute_sensor(track_node *hit, uint64_t time_us);

/* Read sensor-attribution counters for UI diagnostics. */
void traffic_get_sensor_stats(int *spurious, int *ambiguous);

/* Read full attribution stats including last observed anomaly metadata. */
void traffic_get_sensor_stats_ex(traffic_sensor_stats_t *out);

/* Export reserved node indices for train_num (ascending by index).
 * Returns total reserved node count; fills up to max_nodes entries in out. */
int traffic_get_reserved_nodes(int train_num, uint16_t *out, int max_nodes);

/* Return 1 if node is currently reserved by train_num, 0 otherwise. */
int traffic_is_reserved_by(track_node *node, int train_num);

/* Monotonic generation counter for reservation ownership changes. */
uint32_t traffic_get_change_generation(void);

#endif /* _traffic_manager_h_ */
