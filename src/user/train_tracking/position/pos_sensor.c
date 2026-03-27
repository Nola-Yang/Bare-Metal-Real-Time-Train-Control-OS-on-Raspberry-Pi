#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "../traffic/traffic_manager_internal.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "kassert.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

/* Return the remaining route_path cursor for a sensor hit, or -1 when the hit
 * is not on the planned sensor sequence from the current cursor onward. This
 * deliberately ignores mutex-only reservation padding around the route. */
static int route_path_hit_cursor(const train_pos_t *pos, track_node *hit) {
    if (!pos || !hit || pos->route_path_count <= 0) return -1;

    int start = pos->route_path_cursor;
    if (start < 0) start = 0;
    if (start >= pos->route_path_count) return -1;

    int hit_idx = (int)(hit - g_track);
    for (int k = start; k < pos->route_path_count; k++) {
        int idx = (int)pos->route_path[k];
        if (idx < 0 || idx >= TRACK_MAX) continue;
        if (g_track[idx].type != NODE_SENSOR) continue;
        if (idx == hit_idx) return k;
    }
    return -1;
}

static track_node *route_path_first_remaining_sensor(const train_pos_t *pos) {
    if (!pos || pos->route_path_count <= 0) return NULL;

    int start = pos->route_path_cursor;
    if (start < 0) start = 0;
    if (start >= pos->route_path_count) return NULL;

    for (int i = start; i < pos->route_path_count; i++) {
        int idx = (int)pos->route_path[i];
        if (idx < 0 || idx >= TRACK_MAX) continue;
        if (g_track[idx].type == NODE_SENSOR) return &g_track[idx];
    }
    return NULL;
}

static int branch_dir_reaches_sensor(track_node *branch, int dir,
                                     track_node *sensor) {
    if (!branch || branch->type != NODE_BRANCH || !sensor) return 0;
    return follow_dist(branch->edge[dir].dest, sensor, OFF_ROUTE_PATH_MAX_HOPS) >= 0;
}

static int branch_planned_dir(const train_pos_t *pos, track_node *branch) {
    if (!branch || branch->type != NODE_BRANCH) return -1;

    track_node *planned_sensor = route_path_first_remaining_sensor(pos);
    if (!planned_sensor && pos) planned_sensor = pos->pred.next_sensor;

    if (planned_sensor) {
        int straight = branch_dir_reaches_sensor(branch, DIR_STRAIGHT, planned_sensor);
        int curved = branch_dir_reaches_sensor(branch, DIR_CURVED, planned_sensor);
        if (straight != curved) return straight ? DIR_STRAIGHT : DIR_CURVED;
    }

    int sw_idx = track_switch_to_index(branch->num);
    char state = (sw_idx >= 0) ? track_get_switch_state()[sw_idx].state : '?';
    if (state == 'S') return DIR_STRAIGHT;
    if (state == 'C') return DIR_CURVED;
    return -1;
}

static int current_leg_alt_branch_hit(const train_pos_t *pos, track_node *hit) {
    if (!pos || !hit || !pos->cur_sensor) return 0;

    track_node *cur = pos->cur_sensor;
    for (int h = 0; h < 80; h++) {
        track_edge *e = traffic_tm_get_next_edge(cur);
        if (!e || !e->dest) return 0;

        cur = e->dest;
        if (cur == pos->pred.next_sensor || cur->type == NODE_SENSOR) return 0;
        if (cur->type != NODE_BRANCH) continue;

        int planned_dir = branch_planned_dir(pos, cur);
        if (planned_dir != DIR_STRAIGHT && planned_dir != DIR_CURVED) return 0;

        int alt_dir = (planned_dir == DIR_STRAIGHT) ? DIR_CURVED : DIR_STRAIGHT;
        if (!branch_dir_reaches_sensor(cur, alt_dir, hit)) return 0;
        if (branch_dir_reaches_sensor(cur, planned_dir, hit)) return 0;
        return 1;
    }
    return 0;
}

static int route_state_checks_offroute(train_route_state_t route_state) {
    return route_state == TRAIN_STATE_ON_ROUTE ||
           route_state == TRAIN_STATE_STOPPING;
}

/* Update EMA speed, warmup, and prediction-error stats on a sensor hit.
 * Sets *was_predicted for use by the FSM. */
static void update_sensor_stats(train_pos_t *pos, track_node *hit,
                                uint64_t time_us,
                                int *was_predicted) {
    *was_predicted = (pos->pred.next_sensor == hit);

    /* Warmup: suppress EMA calibration for the first 400 mm after a speed change. */
    if (pos->speed_warmup_mm > 0 && pos->cur_sensor != NULL) {
        int32_t seg = follow_dist(pos->cur_sensor, hit, 100);
        if (seg > 0) {
            pos->speed_warmup_mm -= seg;
            if (pos->speed_warmup_mm < 0) pos->speed_warmup_mm = 0;
        }
    }
    int in_warmup = (pos->speed_warmup_mm > 0);

    /* Prediction error */
    if (*was_predicted && pos->pred.trigger_time > 0) {
        pos->pred.last_time_err_us = (int64_t)time_us - (int64_t)pos->pred.trigger_time;
        int64_t derr = (int64_t)pos->effective_v * pos->pred.last_time_err_us / 1000000LL;
        if (derr > 99999) derr = 99999;
        if (derr < -99999) derr = -99999;
        pos->pred.last_dist_err_mm = (int32_t)derr;
        ui_mark_prediction_dirty();
    }

    /* EMA speed update: only while cruising ON_ROUTE, predicted hit, dt > 10 ms. */
    if (!in_warmup && pos->route_state == TRAIN_STATE_ON_ROUTE &&
        *was_predicted && pos->cur_sensor && pos->effective_v > 0) {
        int32_t meas_dist = follow_dist(pos->cur_sensor, hit, 100);
        uint64_t dt = time_us - pos->cur_sensor_time;
        if (dt > 10000 && meas_dist > 0) {
            int32_t meas_v = (int32_t)((int64_t)meas_dist * 1000000LL / (int64_t)dt);
            if (meas_v > 1800) meas_v = 1800;
            pos->effective_v = (int32_t)((7LL * pos->effective_v + meas_v) / 8);
        }
    }
}

/* While waiting out the dead-track timeout, ignore any later attributed sensor
 * and keep the train parked in place. */
static void handle_dead_track_sensor(train_pos_t *pos) {
    if (!pos) return;
    if (pos->cur_sensor) {
        track_node *keep_pred = pos_release_keep_end(pos->cur_sensor, NULL);
        pos->offroute_expected_sensor = keep_pred;
        traffic_refresh_sensor_prediction_reservation(pos->train_num,
                                                      pos->cur_sensor,
                                                      keep_pred,
                                                      TRAIN_BODY_MM);
    }
    track_set_speed(pos->train_num, 0);
    pos->effective_v = 0;
    pos->is_accelerating = 0;
    ui_mark_position_dirty();
}

int pos_hit_matches_alt_branch(const train_pos_t *pos, track_node *hit) {
    if (!pos || !hit) return 0;
    if (pos->pred.branch_node != NULL && pos->pred.alt_sensor != NULL) {
        if (pos->pred.alt_sensor == hit) return 1;

        int32_t alt_dist = follow_dist(pos->pred.alt_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS);
        if (alt_dist >= 0) {
            if (pos->pred.next_sensor == NULL) return 1;
            return follow_dist(pos->pred.next_sensor, hit, OFF_ROUTE_PATH_MAX_HOPS) < 0;
        }
    }

    return current_leg_alt_branch_hit(pos, hit);
}

void pos_revive_dead_track_for_current_hit(train_pos_t *pos) {
    if (!pos || pos->route_state != TRAIN_STATE_DEAD_TRACK ||
        !pos->dead_track_recover.valid || pos->dead_track_recover.orig_target == NULL) {
        return;
    }

    track_node *orig_target = pos->dead_track_recover.orig_target;
    int32_t orig_offset_mm = pos->dead_track_recover.orig_offset_mm;

    pos->dead_track_recover.valid = 0;
    pos->dead_track_recover.orig_target = NULL;
    pos->dead_track_recover.orig_offset_mm = 0;

    pos->route_state = TRAIN_STATE_ON_ROUTE;
    pos->target_sensor = orig_target;
    pos->target_offset_mm = orig_offset_mm;
    pos->pending_target = orig_target;
    pos->pending_offset_mm = orig_offset_mm;
    pos->orig_user_target = orig_target;
    pos->orig_target_offset = orig_offset_mm;
    pos->dist_to_target_mm = 0;
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos->effective_v = 0;
    pos->user_speed = 0;
    pos->is_accelerating = 0;
    pos->awaiting_post_launch_sensor = 0;
    pos_clear_committed_route(pos);
    pos->offroute_valid = 0;
    pos->offroute_expected_sensor = NULL;
    pos->force_offroute_on_next_sensor = 1;
    pos->dead_track_rescue_pending = 1;
    pos->dead_track_bootstrap_due_us = 0;
    pos_clear_prediction(pos);
}

/* Recompute pred.next_sensor and update the dead-track deadline from the
 * next predicted progress sensor only. In FIND_POS, timing is suppressed
 * (trigger_time and deadline stay 0). */
static void update_next_prediction(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    uint64_t dt_pred = 0;
    pos->pred.next_sensor = predict_next_sensor(pos, hit, &dt_pred);
    pos->pred.skipped_sensor_count = 0;

    if (pos->route_state == TRAIN_STATE_FIND_POS ||
        pos->route_state == TRAIN_STATE_RECOVERY_STOPPING) {
        pos->pred.trigger_time = 0;
        pos->dead_track_deadline_us = 0;
        return;
    }

    pos->pred.trigger_time = (dt_pred > 0) ? time_us + dt_pred : 0;
    
    pos->dead_track_deadline_us =
        pos_dead_track_deadline_from_interval(time_us, dt_pred);
}

static void enter_recovery_stop(train_pos_t *pos,
                                track_node *expected_sensor,
                                uint64_t time_us) {
    if (!pos) return;

    pos->force_offroute_on_next_sensor = 0;
    pos->offroute_valid = 1;
    pos->offroute_expected_sensor = expected_sensor;
    pos->pred.trigger_time = 0;
    pos->pred.skipped_sensor_count = 0;
    pos->dead_track_deadline_us = 0;
    track_set_speed(pos->train_num, 0);
    pos->route_state = TRAIN_STATE_RECOVERY_STOPPING;
    pos->stopping_since_us = time_us;
}

void pos_handle_sensor_hit(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    pos_update_accel_velocity(pos, time_us);
    int took_alt_branch = pos_hit_matches_alt_branch(pos, hit);
    int route_hit_cursor = route_path_hit_cursor(pos, hit);

    int was_predicted;
    update_sensor_stats(pos, hit, time_us, &was_predicted);

    pos->cur_sensor = hit;
    pos->cur_sensor_time = time_us;
    pos->awaiting_post_launch_sensor = 0;

    /* Off-route: ON_ROUTE/STOPPING hit outside the remaining planned sensor
     * path, even if the node was still reserved only because of mutex safety
     * padding. */
    if (route_state_checks_offroute(pos->route_state) &&
        pos->target_sensor != NULL) {
        if (pos->force_offroute_on_next_sensor ||
            took_alt_branch ||
            route_hit_cursor < 0) {
            track_node *expected_sensor = pos->pred.next_sensor;
            int conflict_owner = traffic_get_node_owner(hit);

            if (conflict_owner >= 0 && conflict_owner != pos->train_num) {
                train_pos_t *other = pos_get(conflict_owner);
                if (other && route_state_checks_offroute(other->route_state) &&
                    other->cur_sensor != NULL) {
                    enter_recovery_stop(other, other->pred.next_sensor, time_us);
                }
            }

            enter_recovery_stop(pos, expected_sensor, time_us);
            ui_mark_position_dirty();
            return;
        }
    }

    if (pos->route_state == TRAIN_STATE_UNKNOWN) {
        pos->route_state = TRAIN_STATE_KNOWN;
        ui_mark_position_dirty();
    }

    if (pos->route_state == TRAIN_STATE_DEAD_TRACK) {
        handle_dead_track_sensor(pos);
        return;
    }

    /* FIND_POS: stop on the first sensor hit to acquire position. */
    if (pos->route_state == TRAIN_STATE_FIND_POS) {
        track_set_speed(pos->train_num, 0);
        pos->stopping_since_us = time_us;
        pos->route_state = TRAIN_STATE_STOPPING_GOTO;
        ui_mark_position_dirty();
    }

    /* On every on-route sensor hit: advance route_path_cursor to hit and snap
     * dist_to_target_mm to the exact geometric distance from here to the target
     * along the planned path. */
    if ((pos->route_state == TRAIN_STATE_ON_ROUTE ||
         pos->route_state == TRAIN_STATE_STOPPING) &&
        pos->target_sensor) {
        if (route_hit_cursor >= 0) {
            pos->route_path_cursor = route_hit_cursor;
            pos_route_authority_sync_target(pos);
            pos->route_rem_tick_us = time_us;
        }
    }

    if (!pos->offroute_valid) {
        pos->offroute_expected_sensor = NULL;
    }
    update_next_prediction(pos, hit, time_us);
    if (pos->route_state == TRAIN_STATE_STOPPING_GOTO &&
        pos->stop_after_find_pos) {
        traffic_refresh_sensor_prediction_reservation(pos->train_num,
                                                      pos->cur_sensor,
                                                      pos_release_keep_end(pos->cur_sensor,
                                                                           pos->pred.next_sensor),
                                                      TRAIN_BODY_MM);
    }
    if (pos->route_state == TRAIN_STATE_ON_ROUTE) {
        traffic_refresh_route_reservation(pos->train_num, hit,
                                          pos->pred.next_sensor,
                                          pos->route_path,
                                          pos->route_path_cursor,
                                          pos->route_reserved_end_cursor,
                                          pos->route_path_count);
    }

    ui_mark_position_dirty();
}
