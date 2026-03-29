#include "train_tracking/position_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "kassert.h"
#include <stddef.h>
#include <stdint.h>

train_pos_t g_pos[MAX_POS_TRAINS];


static void pos_reset_dead_track_recover(train_pos_t *pos) {
    pos->dead_track_recover.valid = 0;
    pos->dead_track_recover.orig_target = NULL;
    pos->dead_track_recover.orig_offset_mm = 0;
}

static track_node *pos_find_preserved_target(const train_pos_t *pos,
                                             int32_t *out_offset_mm) {
    track_node *target = NULL;
    int32_t offset_mm = 0;

    if (!pos) {
        if (out_offset_mm) *out_offset_mm = 0;
        return NULL;
    }

    if (pos->orig_user_target != NULL) {
        target = pos->orig_user_target;
        offset_mm = pos->orig_target_offset;
    } else if (pos->pending_target != NULL) {
        target = pos->pending_target;
        offset_mm = pos->pending_offset_mm;
    } else if (pos->target_sensor != NULL) {
        target = pos->target_sensor;
        offset_mm = pos->target_offset_mm;
    }

    if (out_offset_mm) *out_offset_mm = target ? offset_mm : 0;
    return target;
}

void pos_clear_deadlock_recover(train_pos_t *pos) {
    if (!pos) return;
    pos->deadlock_recover.valid = 0;
    pos->deadlock_recover.resume_target = NULL;
    pos->deadlock_recover.resume_offset_mm = 0;
    pos->deadlock_recover.yield_target = NULL;
    pos->deadlock_recover.wait_start_mask = 0;
    pos->deadlock_recover.parked_at_yield = 0;
    pos->deadlock_recover.parked_since_us = 0;
}

void pos_clear_prediction(train_pos_t *pos) {
    pos->pred.next_sensor  = NULL;
    pos->pred.alt_sensor   = NULL;
    pos->pred.branch_node  = NULL;
    pos->pred.trigger_time = 0;
    pos->pred.skipped_sensor_count = 0;
    pos->dead_track_deadline_us = 0;
}

static void pos_reset_target_fields(train_pos_t *pos) {
    pos->target_sensor = NULL;
    pos->target_offset_mm = 0;
    pos->dist_to_target_mm = 0;
    pos->pending_target = NULL;
    pos->pending_offset_mm = 0;
    pos->queued_target = NULL;
    pos->queued_offset_mm = 0;
    pos->queued_valid = 0;
    pos->orig_user_target = NULL;
    pos->orig_target_offset = 0;
    pos->offroute_valid = 0;
    pos->offroute_expected_sensor = NULL;
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos_route_authority_reset(pos);
}

static void pos_reset_midrev_fields(train_pos_t *pos) {
    pos->midrev.active = 0;
    pos->midrev.sensor = NULL;
    pos->midrev.final_target = NULL;
    pos->midrev.final_offset = 0;
    pos->midrev.path2_count = 0;
    pos->midrev.sw_count = 0;
    pos->midrev.dist_after = 0;
    for (int k = 0; k < 20; k++) {
        pos->midrev.sw_nums[k] = 0;
        pos->midrev.sw_dirs[k] = '?';
    }
}

static void pos_reset_wait_fields(train_pos_t *pos) {
    if (!pos) return;
    pos->replan.wait_mode = POS_WAIT_NONE;
    pos->replan.need_initial_reverse = 0;
    pos->replan.launch_origin = NULL;
}

void pos_clear_committed_route(train_pos_t *pos) {
    if (!pos) return;

    pos_reset_midrev_fields(pos);
    pos->route_path_count = 0;
    pos->route_path_cursor = 0;
    pos->route_reserved_end_cursor = 0;
    pos->route_rem_tick_us = 0;
    pos_route_authority_reset(pos);
    pos_reset_wait_fields(pos);
}

void pos_commit_route_plan(train_pos_t *pos, const route_plan_t *plan,
                           track_node *launch_origin, int need_initial_reverse,
                           int32_t final_offset_mm) {
    if (!pos || !plan) return;

    pos_clear_committed_route(pos);
    pos->replan.launch_origin = launch_origin;
    pos->replan.need_initial_reverse = need_initial_reverse ? 1 : 0;

    pos->route_path_count = plan->path_count;
    for (int i = 0; i < plan->path_count; i++) {
        pos->route_path[i] = plan->path_nodes[i];
    }

    if (!plan->has_reversal) return;

    pos->midrev.active = 1;
    pos->midrev.sensor = plan->reversal_sensor;
    pos->midrev.final_target = plan->chosen_target;
    pos->midrev.final_offset = final_offset_mm;
    pos->midrev.sw_count = plan->sw_count2;
    for (int i = 0; i < plan->sw_count2; i++) {
        pos->midrev.sw_nums[i] = plan->sw_nums2[i];
        pos->midrev.sw_dirs[i] = plan->sw_dirs2[i];
    }
    pos->midrev.dist_after = plan->dist_after_reversal_mm;
    pos->midrev.path2_count = plan->path_count2;
    for (int i = 0; i < plan->path_count2; i++) {
        pos->midrev.path2[i] = plan->path_nodes2[i];
    }
}

static void pos_init_slot(train_pos_t *slot, int train_num, int train_ind, int speed_level) {
    slot->train_num = train_num;
    slot->train_ind = train_ind;
    slot->goto_speed = speed_level;
    slot->cur_sensor = NULL;
    slot->cur_sensor_time = 0;
    slot->effective_v = 0;
    slot->user_speed = 0;
    slot->pred.next_sensor = NULL;
    slot->pred.alt_sensor = NULL;
    slot->pred.branch_node = NULL;
    slot->pred.trigger_time = 0;
    slot->pred.skipped_sensor_count = 0;
    slot->pred.last_time_err_us = 0;
    slot->pred.last_dist_err_mm = 0;
    slot->route_state = TRAIN_STATE_UNKNOWN;
    pos_reset_target_fields(slot);
    slot->going_forward = 1;
    slot->prev_going_forward = -1;
    slot->position_known = 1;
    track_send_direction(train_num, 0x01);
    slot->stopping_since_us = 0;
    slot->stopped_on_target_hit = 0;
    slot->parked_target_col = POS_TARGET_COL_NONE;
    slot->switch_settle_due_us = 0;
    slot->switch_settle_mode = POS_SWITCH_SETTLE_NONE;
    slot->replan.next_us = 0;
    slot->replan.retry_count = 0;
    slot->replan.rand_state = (uint32_t)(train_num * 1234567u + 1u);
    slot->replan.seen_generation = traffic_get_change_generation();
    slot->replan.blocker_mask = 0;
    slot->replan.wait_mode = POS_WAIT_NONE;
    slot->replan.need_initial_reverse = 0;
    slot->replan.launch_origin = NULL;
    slot->dead_track_deadline_us = 0;
    slot->dead_track_retry_due_us = 0;
    for (int s = 0; s < 15; s++) slot->cached_v[s] = 0;
    slot->speed_warmup_mm = 0;
    slot->accel_a_eff = speed_table_get_accel(train_ind, speed_level);
    slot->is_accelerating = 0;
    slot->accel_start_us = 0;
    slot->awaiting_post_launch_sensor = 0;
    slot->force_offroute_on_next_sensor = 0;
    slot->dead_track_rescue_pending = 0;
    slot->dead_track_warn_active = 0;
    pos_reset_dead_track_recover(slot);
    pos_clear_deadlock_recover(slot);
    pos_reset_midrev_fields(slot);
    slot->route_path_count = 0;
    slot->route_path_cursor = 0;
    slot->route_reserved_end_cursor = 0;
    slot->route_rem_tick_us = 0;
    slot->authority_seen_generation = traffic_get_change_generation();
    slot->authority_next_us = 0;
    slot->stop_after_find_pos = 0;
}

static int32_t train_num_to_ind(int train_num) {
    if (13 <= train_num && train_num <= 15) {
        return train_num - 13;
    } else if (17 <= train_num && train_num <= 18) {
        return train_num - 14;
    } else if (train_num == 55) {
        return 0;
    }

    return -1;
}

train_pos_t *pos_find_slot(int train_num) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        if (g_pos[i].train_num == train_num) return &g_pos[i];
    }
    return NULL;
}

train_pos_t *pos_find_or_create_slot(int train_num, int speed_level) {
    train_pos_t *slot = pos_find_slot(train_num);
    if (slot) return slot;

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        int32_t train_ind;

        if (g_pos[i].train_num >= 0) continue;
        train_ind = train_num_to_ind(train_num);
        if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return NULL;

        slot = &g_pos[i];
        pos_init_slot(slot, train_num, train_ind, speed_level);
        return slot;
    }
    return NULL;
}

void pos_reset_dead_train(int train_num) {
    train_pos_t *pos = pos_find_slot(train_num);
    int was_stopped;
    track_node *keep_hint;
    if (!pos) return;

    was_stopped = pos->route_state == TRAIN_STATE_STOPPED;
    keep_hint = pos->pred.next_sensor ? pos->pred.next_sensor
                                      : pos->offroute_expected_sensor;
    KASSERT(pos->cur_sensor != NULL);

    if (!was_stopped) {
        traffic_release_train_keep_body(train_num, pos->cur_sensor,
                                        TRAIN_BODY_MM,
                                        pos_release_keep_end(pos->cur_sensor,
                                                             keep_hint));
    }

    pos->user_speed = 0;
    pos->effective_v = 0;
    pos->stopping_since_us = 0;
    pos->is_accelerating = 0;
    pos->accel_start_us = 0;
    pos->route_state = TRAIN_STATE_STOPPED;
    pos->stopped_on_target_hit = 0;
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos->switch_settle_due_us = 0;
    pos->switch_settle_mode = POS_SWITCH_SETTLE_NONE;
    pos->awaiting_post_launch_sensor = 0;
    pos->force_offroute_on_next_sensor = 0;
    pos->dead_track_rescue_pending = 0;
    pos->dead_track_warn_active = 0;
    pos->dead_track_recover.valid = 0;
    pos->dead_track_retry_due_us = 0;
    pos->replan.blocker_mask = 0;
    pos->replan.next_us = 0;
    pos->replan.retry_count = 0;
    pos_clear_prediction(pos);
    pos_reset_target_fields(pos);
    pos_clear_committed_route(pos);
    pos_clear_deadlock_recover(pos);
}

void pos_restore_pending_target(train_pos_t *pos) {
    if (pos->pending_target == NULL && pos->orig_user_target != NULL) {
        pos->pending_target = pos->orig_user_target;
        pos->pending_offset_mm = pos->orig_target_offset;
    }
}

void pos_save_ema_and_stop(train_pos_t *pos) {
    if (pos->user_speed > 0 && pos->user_speed <= 14) {
        pos->cached_v[pos->user_speed] = pos->effective_v;
    }
    pos->effective_v = 0;
    pos->awaiting_post_launch_sensor = 0;
}

void pos_prepare_goto_request(train_pos_t *pos, track_node *target, int speed_level, int32_t offset_mm) {
    if (!pos || !target) return;

    pos->goto_speed = speed_level;
    pos->accel_a_eff = speed_table_get_accel(pos->train_ind, speed_level);
    pos->pending_target = target;
    pos->pending_offset_mm = offset_mm;
    pos->orig_user_target = target;
    pos->orig_target_offset = offset_mm;
    pos->replan.blocker_mask = 0;
    pos_reset_wait_fields(pos);
    pos_clear_deadlock_recover(pos);
    pos_reset_dead_track_recover(pos);
    pos->dead_track_warn_active = 0;
    pos->dead_track_retry_due_us = 0;
    pos->offroute_valid = 0;
    pos->offroute_expected_sensor = NULL;
    pos->target_sensor = target;
    pos->target_offset_mm = offset_mm;
    pos->dist_to_target_mm = 0;
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos->replan.next_us = 0;
    pos_clear_committed_route(pos);
    pos->stop_after_find_pos = 0;
}

void pos_prepare_find_pos_request(train_pos_t *pos) {
    track_node *preserved_target;
    int32_t preserved_offset_mm = 0;
    int keep_target;

    if (!pos) return;

    preserved_target = pos_find_preserved_target(pos, &preserved_offset_mm);
    keep_target = pos->route_state == TRAIN_STATE_DEAD_TRACK &&
                  preserved_target != NULL;

    if (keep_target) {
        pos->pending_target = preserved_target;
        pos->pending_offset_mm = preserved_offset_mm;
        pos->orig_user_target = preserved_target;
        pos->orig_target_offset = preserved_offset_mm;
        pos->target_sensor = preserved_target;
        pos->target_offset_mm = preserved_offset_mm;
    } else {
        pos->pending_target = NULL;
        pos->pending_offset_mm = 0;
        pos->orig_user_target = NULL;
        pos->orig_target_offset = 0;
        pos->target_sensor = NULL;
        pos->target_offset_mm = 0;
    }
    pos->replan.blocker_mask = 0;
    pos_reset_wait_fields(pos);
    pos_clear_deadlock_recover(pos);
    pos_reset_dead_track_recover(pos);
    if (pos->route_state != TRAIN_STATE_DEAD_TRACK) {
        pos->dead_track_warn_active = 0;
    }
    pos->dead_track_retry_due_us = 0;
    pos->dist_to_target_mm = 0;
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos->replan.next_us = 0;
    pos->offroute_valid = 0;
    pos->offroute_expected_sensor = NULL;
    pos_clear_committed_route(pos);
    pos->stop_after_find_pos = 1;
}

train_pos_t *pos_get(int train_num) {
    return pos_find_slot(train_num);
}

train_pos_t *pos_get_by_index(int i) {
    if (i < 0 || i >= MAX_POS_TRAINS) return NULL;
    return &g_pos[i];
}
