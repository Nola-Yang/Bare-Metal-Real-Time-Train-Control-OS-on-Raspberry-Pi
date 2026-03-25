#include "train_tracking/position_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include <stddef.h>
#include <stdint.h>

train_pos_t g_pos[MAX_POS_TRAINS];

#ifdef TRACK_D
    static const int32_t GOTO_ACCEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {37, 37, 38, 38, 38};
#else
    static const int32_t GOTO_ACCEL_MM_S2[MAX_PHYSICAL_TRAINS] =
        {37, 37, 38, 38, 38};
#endif

static void pos_reset_dead_track_recover(train_pos_t *pos) {
    pos->dead_track_recover.valid = 0;
    pos->dead_track_recover.orig_target = NULL;
    pos->dead_track_recover.orig_offset_mm = 0;
}

void pos_clear_deadlock_recover(train_pos_t *pos) {
    if (!pos) return;
    pos->deadlock_recover.valid = 0;
    pos->deadlock_recover.resume_target = NULL;
    pos->deadlock_recover.resume_offset_mm = 0;
    pos->deadlock_recover.yield_target = NULL;
    pos->deadlock_recover.wait_start_mask = 0;
    pos->deadlock_recover.parked_at_yield = 0;
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

static void pos_init_slot(train_pos_t *slot, int train_num, int train_ind) {
    slot->train_num = train_num;
    slot->train_ind = train_ind;
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
    slot->position_known = 1;
    track_send_direction(train_num, 0x01);
    slot->stopping_since_us = 0;
    slot->stopped_on_target_hit = 0;
    slot->switch_settle_due_us = 0;
    slot->switch_settle_mode = POS_SWITCH_SETTLE_NONE;
    slot->replan.next_us = 0;
    slot->replan.retry_count = 0;
    slot->replan.rand_state = (uint32_t)(train_num * 1234567u + 1u);
    slot->replan.seen_generation = traffic_get_change_generation();
    slot->replan.blocker_mask = 0;
    slot->dead_track_deadline_us = 0;
    for (int s = 0; s < 15; s++) slot->cached_v[s] = 0;
    slot->speed_warmup_mm = 0;
    slot->accel_a_eff = GOTO_ACCEL_MM_S2[train_ind];
    slot->is_accelerating = 0;
    slot->accel_start_us = 0;
    slot->awaiting_post_launch_sensor = 0;
    slot->force_offroute_on_next_sensor = 0;
    slot->dead_track_rescue_pending = 0;
    pos_reset_dead_track_recover(slot);
    pos_clear_deadlock_recover(slot);
    pos_reset_midrev_fields(slot);
    slot->route_path_count = 0;
    slot->route_path_cursor = 0;
    slot->route_rem_tick_us = 0;
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

train_pos_t *pos_find_or_create_slot(int train_num) {
    train_pos_t *slot = pos_find_slot(train_num);
    if (slot) return slot;

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        int32_t train_ind;

        if (g_pos[i].train_num >= 0) continue;
        train_ind = train_num_to_ind(train_num);
        if (train_ind < 0 || train_ind >= MAX_PHYSICAL_TRAINS) return NULL;

        slot = &g_pos[i];
        pos_init_slot(slot, train_num, train_ind);
        return slot;
    }
    return NULL;
}

void pos_reset_dead_train(int train_num) {
    train_pos_t *pos = pos_find_slot(train_num);
    if (!pos) return;

    pos->route_state = TRAIN_STATE_STOPPED;
    pos->stopped_on_target_hit = 0;
    pos->switch_settle_due_us = 0;
    pos->switch_settle_mode = POS_SWITCH_SETTLE_NONE;
    pos->awaiting_post_launch_sensor = 0;
    pos->force_offroute_on_next_sensor = 0;
    pos->dead_track_rescue_pending = 0;
    pos->dead_track_recover.valid = 0;
    pos->replan.blocker_mask = 0;
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

void pos_prepare_goto_request(train_pos_t *pos, track_node *target, int32_t offset_mm) {
    if (!pos || !target) return;

    pos->pending_target = target;
    pos->pending_offset_mm = offset_mm;
    pos->orig_user_target = target;
    pos->orig_target_offset = offset_mm;
    pos->replan.blocker_mask = 0;
    pos_clear_deadlock_recover(pos);
    pos->offroute_valid = 0;
    pos->offroute_expected_sensor = NULL;
    pos->target_sensor = target;
    pos->target_offset_mm = offset_mm;
    pos->dist_to_target_mm = 0;
    pos->replan.next_us = 0;
    pos->stop_after_find_pos = 0;
}

void pos_prepare_find_pos_request(train_pos_t *pos) {
    if (!pos) return;

    pos->pending_target = NULL;
    pos->pending_offset_mm = 0;
    pos->orig_user_target = NULL;
    pos->orig_target_offset = 0;
    pos->replan.blocker_mask = 0;
    pos_clear_deadlock_recover(pos);
    pos->target_sensor = NULL;
    pos->target_offset_mm = 0;
    pos->dist_to_target_mm = 0;
    pos->replan.next_us = 0;
    pos->offroute_valid = 0;
    pos->offroute_expected_sensor = NULL;
    pos->stop_after_find_pos = 1;
}

train_pos_t *pos_get(int train_num) {
    return pos_find_slot(train_num);
}

train_pos_t *pos_get_by_index(int i) {
    if (i < 0 || i >= MAX_POS_TRAINS) return NULL;
    return &g_pos[i];
}
