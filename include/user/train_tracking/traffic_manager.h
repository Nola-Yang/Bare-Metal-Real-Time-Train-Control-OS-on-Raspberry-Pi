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

typedef struct {
    train_pos_t *owner;
    uint8_t      rescued_from_pair;
    uint8_t      revive_dead_track;
} traffic_attr_result_t;

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
                                       int path_end_cursor,
                                       int path_count);

/* Refresh a bootstrap/dead-track reservation window anchored at cur_sensor:
 * keep cur_sensor, rear_mm behind it, and every reachable node on the path
 * from cur_sensor to pred_sensor. */
void traffic_refresh_sensor_prediction_reservation(int train_num,
                                                   track_node *cur_sensor,
                                                   track_node *pred_sensor,
                                                   int32_t rear_mm);

/* Check if it is safe to set a switch.
 * Any reservation on the switch envelope blocks the change, including self-owned
 * reservations. Returns -1 when safe; otherwise returns the blocking train number. */
int traffic_can_set_switch(int sw_num, int requester_train);

/* Collect unique train numbers that currently block reserving `plan`.
 * Returns total unique blocker count and fills up to max_trains entries. */
int traffic_collect_plan_blockers(int requester_train, const route_plan_t *plan,
                                  int *out_trains, int max_trains);

/* Collect unique train numbers occupying the safety envelope around `sw_num`.
 * Returns total unique blocker count and fills up to max_trains entries. */
int traffic_collect_switch_envelope_blockers(int sw_num, int *out_trains,
                                             int max_trains);

/* Return the first conflicting reserved node for `plan`, or NULL if none. */
track_node *traffic_find_plan_blocking_node(int requester_train,
                                            const route_plan_t *plan,
                                            int *out_owner);

/* Return the first occupied node in the switch safety envelope, or NULL if clear. */
track_node *traffic_find_switch_blocking_node(int sw_num, int *out_owner);

/* Copy and restore the current reservation owner map for local simulations. */
void traffic_snapshot_reservations(int out_owners[TRACK_MAX]);
void traffic_restore_reservations(const int owners[TRACK_MAX]);

/* Simulate a stopped train holding the same keep-body window that
 * traffic_release_train_keep_body() would preserve after stopping at last_hit. */
void traffic_simulate_parked_train(int train_num, track_node *last_hit,
                                   int32_t body_mm, track_node *next_hit);

/* Attribute a sensor hit to the best train, or return owner=NULL when the
 * event remains spurious after all rescue logic. */
traffic_attr_result_t traffic_attribute_sensor(track_node *hit, uint64_t time_us);

/* Read sensor-attribution counters for UI diagnostics. */
void traffic_get_sensor_stats(int *spurious, int *ambiguous);

/* Read full attribution stats including last observed anomaly metadata. */
void traffic_get_sensor_stats_ex(traffic_sensor_stats_t *out);

/* Export reserved node indices for train_num (ascending by index).
 * Returns total reserved node count; fills up to max_nodes entries in out. */
int traffic_get_reserved_nodes(int train_num, uint16_t *out, int max_nodes);

/* Return 1 if node is currently reserved by train_num, 0 otherwise. */
int traffic_is_reserved_by(track_node *node, int train_num);

/* Return the train number currently owning `node`, or -1 when unreserved. */
int traffic_get_node_owner(track_node *node);

/* Monotonic generation counter for reservation ownership changes. */
uint32_t traffic_get_change_generation(void);

#endif /* _traffic_manager_h_ */
