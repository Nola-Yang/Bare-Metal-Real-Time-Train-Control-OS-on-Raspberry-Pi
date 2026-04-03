#include "train_tracking/position_priv.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "../traffic/traffic_manager_internal.h"
#include "../traffic/traffic_window_internal.h"
#include "track.h"
#include "train_tracking/speed_table.h"
#include "syscall.h"
#include "kassert.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>


/* Offsets at end of train with undershoot of 1.5 cm. */
static uint32_t Train_Forward_Stop_Offset = 34;
static uint32_t Train_Reverse_Stop_Offset = 146;
static uint32_t Train_Forward_to_Rev_Stop_Offset = 146;
static uint32_t Train_Rev_to_Forward_Stop_Offset = 34;

void pos_update_accel_velocity(train_pos_t *pos, uint64_t now_us) {
    if (!pos || !pos->is_accelerating) return;

    int64_t t_moving_us = (int64_t)now_us - (int64_t)pos->accel_start_us;
    if (t_moving_us <= 0) {
        pos->effective_v = 0;   /* still within GO_LATENCY_US window */
        return;
    }

    int32_t v_goto = speed_table_get_v(pos->train_ind, pos->goto_speed);
    int64_t t_accel_us = (int64_t)v_goto * 1000000LL / (int64_t)pos->accel_a_eff;

    if (t_moving_us >= t_accel_us) {
        pos->effective_v = v_goto;
        pos->is_accelerating = 0;   /* reached full speed */
    } else {
        pos->effective_v =
            (int32_t)((int64_t)pos->accel_a_eff * t_moving_us / 1000000LL);
    }
}

/* Estimate time (us) for a braking train to reach a full stop. */
static uint64_t calc_brake_us(train_pos_t *pos) {
    int32_t decel = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
    if (pos->effective_v > 0 && decel > 0) {
        return pos_target_early_stop_us(pos) +
               (uint64_t)pos->effective_v * 1500000ULL / (uint64_t)decel;
    }
    return 1000000ULL;
}

static int brake_elapsed(train_pos_t *pos, uint64_t now_us) {
    return now_us >= pos->stopping_since_us + calc_brake_us(pos);
}

static int pos_waiting_for_first_launch_hit(const train_pos_t *pos) {
    return pos != NULL &&
           pos->route_state == TRAIN_STATE_ON_ROUTE &&
           pos->awaiting_post_launch_sensor;
}

static void start_queued_goto_if_any(train_pos_t *pos) {
    if (!pos || !pos->queued_valid || !pos->queued_target) return;
    track_node *qt = pos->queued_target;
    int32_t qo = pos->queued_offset_mm;
    pos->queued_target = NULL;
    pos->queued_offset_mm = 0;
    pos->queued_valid = 0;
    pos_goto(pos->train_num, qt, pos->goto_speed, qo);
}

/* On stop completion, either keep the hit sensor plus the next sensor the
 * train would hit if it kept moving in its current direction, or keep the
 * remaining planned tail up to the current stop target.  If no such next
 * sensor exists at a target hit, fall back to the stopped train body only. */
void pos_refresh_stop_reservation(train_pos_t *pos) {
    track_node *keep_end;
    track_node *keep_after_hit_sensor;
    int keep_remaining_route;

    if (!pos || !pos->cur_sensor) return;

    keep_end = pos_release_keep_end(pos->cur_sensor, NULL);
    keep_after_hit_sensor = NULL;
    if (pos->target_sensor != NULL &&
        pos->cur_sensor == pos->target_sensor) {
        keep_after_hit_sensor = keep_end;
    }
    keep_remaining_route =
        pos->route_path_count > 0 &&
        pos->route_path_cursor >= 0 &&
        pos->route_path_cursor < pos->route_reserved_end_cursor;

    if (keep_after_hit_sensor != NULL) {
        traffic_refresh_sensor_prediction_reservation(pos->train_num,
                                                      pos->cur_sensor,
                                                      keep_after_hit_sensor,
                                                      0);
        return;
    }

    if (pos->target_sensor != NULL &&
        pos->cur_sensor == pos->target_sensor) {
        traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                        TRAIN_BODY_MM, NULL);
        return;
    }

    if (keep_remaining_route) {
        traffic_refresh_route_reservation(pos->train_num, pos->cur_sensor,
                                          keep_end,
                                          pos->route_path,
                                          pos->route_path_cursor,
                                          pos->route_reserved_end_cursor,
                                          pos->route_path_count);
        return;
    }
}

static int pos_targets_same_sensor(track_node *a, track_node *b) {
    if (!a || !b) return 0;
    return a == b || a->reverse == b || b->reverse == a;
}

static int pos_should_block_initial_reverse_restart(track_node *stopped_target) {
    return !route_initial_reverse_restart_allowed(stopped_target);
}

static track_node *pos_frozen_stop_target(const train_pos_t *pos) {
    if (!pos) return NULL;
    return pos->stopping_target_sensor ? pos->stopping_target_sensor
                                       : pos->target_sensor;
}

static track_node *pos_dead_track_resume_target(const train_pos_t *pos,
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

static uint8_t stop_wait_blocker_mask(const train_pos_t *pos) {
    route_plan_t remaining_plan;

    if (!pos) return 0;
    if (pos->route_path_count <= 0) return 0;
    if (pos->route_path_cursor < 0 || pos->route_path_cursor >= pos->route_path_count) {
        return 0;
    }

    if (!traffic_window_build_prefix_plan(pos->route_path, pos->route_path_count,
                                          pos->route_path_cursor,
                                          pos->route_path_count - 1,
                                          &remaining_plan)) {
        return 0;
    }

    return pos_route_blocker_mask_from_plan(pos->train_num, &remaining_plan) |
           pos_route_blocker_mask_from_switches(remaining_plan.sw_nums,
                                                remaining_plan.sw_dirs,
                                                remaining_plan.sw_count,
                                                pos->train_num);
}

static void handle_normal_stop(train_pos_t *pos, uint64_t now_us) {
    track_node *stopped_target;
    int stopped_at_leg_goal;

    pos->route_state = TRAIN_STATE_STOPPED;
    stopped_target = pos_frozen_stop_target(pos);
    pos->stopping_target_sensor = NULL;
    pos->stopped_on_target_hit =
        pos_route_authority_is_leg_goal_stop(pos) &&
        pos->cur_sensor != NULL &&
        pos_targets_same_sensor(pos->cur_sensor, stopped_target);
    stopped_at_leg_goal = pos_route_authority_is_leg_goal_stop(pos);
    /* Use the stop target frozen when braking began so later authority/UI
     * updates cannot change the logical parked direction for this stop. */
    pos->parked_restart_block_initial_reverse =
        (stopped_target != NULL &&
         pos_should_block_initial_reverse_restart(stopped_target))
            ? 1
            : 0;
    if (stopped_target != NULL) {
        pos->parked_target_col =
            stopped_at_leg_goal ? POS_TARGET_COL_FINAL : POS_TARGET_COL_STAGE;
    }
    if (stopped_at_leg_goal && stopped_target != NULL) {
        pos_publish_game_goal_stop(pos, stopped_target, now_us);
    }
    pos_refresh_stop_reservation(pos);
    pos_clear_prediction(pos);

    if (pos->queued_valid && pos->queued_target) {
        pos->orig_user_target = NULL;
        pos->orig_target_offset = 0;
        pos_clear_deadlock_recover(pos);
        start_queued_goto_if_any(pos);
        return;
    }

    if (pos->deadlock_recover.valid &&
        pos_targets_same_sensor(stopped_target, pos->deadlock_recover.yield_target)) {
        if (pos->deadlock_recover.resume_target != NULL) {
            /* Keep the train logically parked here until
             * the deadlock recovery logic explicitly decides to resume. */
            pos->deadlock_recover.parked_at_yield = 1;
            pos->deadlock_recover.parked_since_us = now_us;
            return;
        }

        /* No-resume deadlock reroutes should park at the staged yield stop
         * instead of resuming the remaining committed route on the next
         * WAIT_RESOURCE retry. */
        pos_clear_committed_route(pos);
        pos->pending_target = NULL;
        pos->pending_offset_mm = 0;
        pos->target_sensor = stopped_target;
        pos->target_offset_mm = 0;
        pos->dist_to_target_mm = 0;
        pos->orig_user_target = stopped_target;
        pos->orig_target_offset = 0;
        pos->parked_target_col = POS_TARGET_COL_FINAL;
        pos->replan.blocker_mask = 0;
        pos->replan.next_us = 0;
        pos_clear_deadlock_recover(pos);
        return;
    }

    if (!pos_route_authority_is_leg_goal_stop(pos)) {
        pos_enter_wait_resource(pos, now_us, stop_wait_blocker_mask(pos),
                                POS_WAIT_RESUME_ROUTE);
        return;
    }

    pos_clear_committed_route(pos);
    pos->orig_user_target = NULL;
    pos->orig_target_offset = 0;
}

static void handle_midrev_stop(train_pos_t *pos, uint64_t now_us) {
    int hit_stage_target;

    if (!pos) return;

    pos->route_state = TRAIN_STATE_STOPPED;
    pos->stopping_target_sensor = NULL;
    hit_stage_target =
        pos->cur_sensor != NULL &&
        pos_targets_same_sensor(pos->cur_sensor, pos->target_sensor);
    pos->stopped_on_target_hit =
        pos_route_authority_is_leg_goal_stop(pos) && hit_stage_target;
    pos->parked_target_col = POS_TARGET_COL_STAGE;
    pos->parked_restart_block_initial_reverse = 0;

    pos_refresh_stop_reservation(pos);
    pos_clear_prediction(pos);

    (void)pos_handle_midrev_resume(pos, now_us);
}

/* Handle STOPPING -> STOPPED transition (with mid-route reversal if active). */
static int tick_handle_stopping(train_pos_t *pos, uint64_t now_us) {
    if (pos->route_state == TRAIN_STATE_STOPPED) return 1;
    if (!brake_elapsed(pos, now_us)) return 1;

    pos_save_ema_and_stop(pos);

    if (pos->midrev.active && pos_route_authority_is_leg_goal_stop(pos)) {
        handle_midrev_stop(pos, now_us);
    } else {
        handle_normal_stop(pos, now_us);
    }
    ui_mark_position_dirty();
    return 1;
}

static int do_recovery_replan(train_pos_t *pos) {
    track_node *keep_hint;

    pos_save_ema_and_stop(pos);
    keep_hint = pos->pred.next_sensor
                ? pos->pred.next_sensor
                : pos->offroute_expected_sensor;
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM,
                                    pos_release_keep_end(pos->cur_sensor,
                                                         keep_hint));

    pos_restore_pending_target(pos);
    KASSERT(pos->pending_target != NULL);
    int ok = pos_try_direct_goto(pos);
    KASSERT(ok);
    return 1;
}

/* Handle RECOVERY_STOPPING -> replan.
 * Returns 1 when braking is complete and replan was attempted. */
static int tick_handle_recovery_stopping(train_pos_t *pos, uint64_t now_us) {
    if (pos->dead_track_rescue_pending && pos->effective_v == 0) {
        pos->dead_track_rescue_pending = 0;
        return do_recovery_replan(pos);
    }

    if (!brake_elapsed(pos, now_us)) return 0;

    pos->dead_track_rescue_pending = 0;
    return do_recovery_replan(pos);
}

/* Handle STOPPING_TR -> STOPPED.
 * Returns 1 when braking is complete. */
static int tick_handle_stopping_tr(train_pos_t *pos, uint64_t now_us) {
    if (!brake_elapsed(pos, now_us)) return 0;

    pos->route_state = TRAIN_STATE_STOPPED;
    pos->stopping_target_sensor = NULL;
    pos->stopped_on_target_hit = 0;
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos->effective_v = 0;
    traffic_release_train_keep_body(pos->train_num, pos->cur_sensor,
                                    TRAIN_BODY_MM,
                                    pos_release_keep_end(pos->cur_sensor,
                                                         pos->pred.next_sensor));
    pos_clear_prediction(pos);
    start_queued_goto_if_any(pos);
    ui_mark_position_dirty();
    return 1;
}

/* Handle STOPPING_GOTO -> replan (or STOPPED if stop_after_find_pos).
 * Returns 1 when braking is complete. */
static int tick_handle_stopping_goto(train_pos_t *pos, uint64_t now_us) {
    if (!brake_elapsed(pos, now_us)) return 0;

    pos_save_ema_and_stop(pos);
    if (pos->stop_after_find_pos) {
        int32_t resume_offset_mm = 0;
        track_node *resume_target = pos_dead_track_resume_target(pos, &resume_offset_mm);
        track_node *keep_pred = pos->cur_sensor
                                ? pos_release_keep_end(pos->cur_sensor,
                                                       pos->pred.next_sensor)
                                : NULL;
        pos->stop_after_find_pos = 0;
        pos->route_state = TRAIN_STATE_STOPPED;
        pos->stopped_on_target_hit = 0;
        pos->parked_target_col = POS_TARGET_COL_NONE;
        traffic_refresh_sensor_prediction_reservation(pos->train_num,
                                                      pos->cur_sensor,
                                                      keep_pred,
                                                      TRAIN_BODY_MM);
        pos_clear_prediction(pos);
        if (resume_target != NULL) {
            pos_restore_pending_target(pos);
            if (pos->pending_target == NULL) {
                pos->pending_target = resume_target;
                pos->pending_offset_mm = resume_offset_mm;
            }
            int ok = pos_try_direct_goto(pos);
            KASSERT(ok);
        } else {
            ui_mark_position_dirty();
        }
    } else {
        int ok = pos_try_direct_goto(pos);
        KASSERT(ok);
    }
    return 1;
}

/* Continuously estimate remaining distance to target and issue the stop
 * command when the train enters the braking window. */
static int tick_check_brake_point(train_pos_t *pos, uint64_t now_us) {
    int waiting_for_first_hit;
    int stop_cmd_sent;

    if (pos->route_state != TRAIN_STATE_ON_ROUTE) return 0;
    if (!pos->target_sensor || !pos->cur_sensor || pos->effective_v <= 0) return 0;

    waiting_for_first_hit = pos_waiting_for_first_launch_hit(pos);
    stop_cmd_sent = pos->stopping_since_us > 0;

    if (pos->route_rem_tick_us > 0 && now_us > pos->route_rem_tick_us) {
        uint64_t dt = now_us - pos->route_rem_tick_us;
        int32_t delta = (int32_t)((int64_t)pos->effective_v * (int64_t)dt / 1000000LL);
        pos->dist_to_target_mm -= delta;
        if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
    }
    pos->route_rem_tick_us = now_us;

    int32_t rem = pos->dist_to_target_mm;

    /* If braking was already commanded before the first post-launch hit,
     * keep the train logically ON_ROUTE until a real sensor confirms progress.
     * Once a hit arrives, convert that pending brake into STOPPING. */
    if (stop_cmd_sent && !waiting_for_first_hit) {
        pos->route_state = TRAIN_STATE_STOPPING;
        pos->dead_track_deadline_us = 0;
        ui_mark_position_dirty();
        return 1;
    }

    int32_t a = speed_table_get_decel(pos->train_ind, pos->user_speed, pos->target_sensor);
    if (a > 0) {
        uint64_t early_stop_us = pos_target_early_stop_us(pos);
        int32_t d_brake = (int32_t)((int64_t)pos->effective_v * pos->effective_v / (2LL * a));
        int32_t d_early = d_brake + (int32_t)((int64_t)pos->effective_v *
                                              (int64_t)early_stop_us / 1000000LL);

        if (pos->going_forward && pos->prev_going_forward != 0) {
            d_early += Train_Forward_Stop_Offset;
        } else if (pos->going_forward) {
            d_early += Train_Rev_to_Forward_Stop_Offset;
        } else if (pos->prev_going_forward == 0) {
            d_early +=  Train_Reverse_Stop_Offset;
        } else {
            d_early += Train_Forward_to_Rev_Stop_Offset;
        }

        d_early += node_get_override_offset(pos->target_sensor);

        if (rem <= d_early) {
            if (!stop_cmd_sent && pos_route_authority_try_top_up(pos, now_us, 1)) {
                pos->route_rem_tick_us = now_us;
                ui_mark_position_dirty();
                return 0;
            }
            if (waiting_for_first_hit) {
                if (!stop_cmd_sent) {
                    pos->stopping_since_us = now_us;
                    pos->stopping_target_sensor = pos->target_sensor;
                    track_set_speed(pos->train_num, 0);
                }
                ui_mark_position_dirty();
                return 1;
            }

            pos->route_state = TRAIN_STATE_STOPPING;
            pos->dead_track_deadline_us = 0;
            if (!stop_cmd_sent) {
                pos->stopping_since_us = now_us;
                pos->stopping_target_sensor = pos->target_sensor;
                track_set_speed(pos->train_num, 0);
            }
            ui_mark_position_dirty();
            return 1;
        }
    }

    ui_mark_position_dirty();
    return 0;
}

static void resume_onroute_after_dead_track(train_pos_t *pos, uint64_t now_us) {
    uint64_t dt = 0;

    if (!pos || pos->cur_sensor == NULL) return;

    if (pos->route_path_count > 0) {
        pos_route_authority_sync_target(pos);
    }

    pos->route_state = TRAIN_STATE_ON_ROUTE;
    pos_launch_at_goto_speed(pos, now_us);
    pos->dead_track_warn_active = 1;

    pos->pred.next_sensor = predict_next_sensor(pos, pos->cur_sensor, &dt);
    pos->pred.trigger_time = (dt > 0) ? (pos->accel_start_us + dt) : 0;
    pos->pred.skipped_sensor_count = 0;
    pos->route_rem_tick_us = now_us;
    pos->stopping_since_us = 0;
    pos->dead_track_retry_due_us = 0;
    pos_refresh_dead_track_deadline(pos, now_us);
    ui_mark_position_dirty();
}

static void enter_terminal_dead_track(train_pos_t *pos, uint64_t now_us) {
    track_node *guessed_end;
    track_node *resume_target;
    int32_t resume_offset_mm = 0;

    if (!pos) return;

    if (pos->route_state == TRAIN_STATE_ON_ROUTE &&
        pos->cur_sensor != NULL &&
        pos->route_path_count > 0) {
        track_set_speed(pos->train_num, 0);
        pos_save_ema_and_stop(pos);
        pos->is_accelerating = 0;
        pos->route_state = TRAIN_STATE_DEAD_TRACK;
        pos->stopping_since_us = now_us;
        pos->route_rem_tick_us = 0;
        pos->dead_track_deadline_us = 0;
        pos->dead_track_retry_due_us =
            (now_us > UINT64_MAX - DEAD_TRACK_RETRY_DELAY_US)
                ? UINT64_MAX
                : now_us + DEAD_TRACK_RETRY_DELAY_US;
        pos->dead_track_warn_active = 1;
        pos->dead_track_recover.valid = 0;
        pos->dead_track_recover.orig_target = NULL;
        pos->dead_track_recover.orig_offset_mm = 0;
        pos->offroute_valid = 0;
        pos->offroute_expected_sensor = NULL;
        return;
    }

    resume_target = pos_dead_track_resume_target(pos, &resume_offset_mm);

    int can_rescue_dead_track =
        pos->route_state == TRAIN_STATE_ON_ROUTE &&
        pos->accel_start_us > 0 &&
        pos->cur_sensor_time < pos->accel_start_us &&
        pos->orig_user_target != NULL;

    if (can_rescue_dead_track) {
        pos->dead_track_recover.valid = 1;
        pos->dead_track_recover.orig_target = pos->orig_user_target;
        pos->dead_track_recover.orig_offset_mm = pos->orig_target_offset;
    } else {
        pos->dead_track_recover.valid = 0;
        pos->dead_track_recover.orig_target = NULL;
        pos->dead_track_recover.orig_offset_mm = 0;
    }

    guessed_end = pos->offroute_expected_sensor
                  ? pos->offroute_expected_sensor
                  : pos->pred.next_sensor;
    if (pos->cur_sensor) {
        guessed_end = pos_release_keep_end(pos->cur_sensor, guessed_end);
    }

    track_set_speed(pos->train_num, 0);
    traffic_refresh_sensor_prediction_reservation(pos->train_num,
                                                  pos->cur_sensor,
                                                  guessed_end,
                                                  TRAIN_BODY_MM);

    pos->effective_v = 0;
    pos->user_speed = 0;
    pos->is_accelerating = 0;
    pos->route_state = TRAIN_STATE_DEAD_TRACK;
    pos->parked_target_col = POS_TARGET_COL_NONE;
    pos->target_sensor = resume_target;
    pos->target_offset_mm = resume_target ? resume_offset_mm : 0;
    pos->dist_to_target_mm = 0;
    pos->pending_target = resume_target;
    pos->pending_offset_mm = resume_target ? resume_offset_mm : 0;
    pos->queued_target = NULL;
    pos->queued_offset_mm = 0;
    pos->queued_valid = 0;
    pos->orig_user_target = resume_target;
    pos->orig_target_offset = resume_target ? resume_offset_mm : 0;
    pos->stop_after_find_pos = 0;
    pos->midrev.active = 0;
    pos->replan.next_us = 0;
    pos->replan.retry_count = 0;
    pos->replan.blocker_mask = 0;
    pos_clear_committed_route(pos);
    pos->force_offroute_on_next_sensor = 0;
    pos->dead_track_rescue_pending = 0;
    pos->dead_track_warn_active = 1;
    pos_clear_deadlock_recover(pos);
    pos->offroute_valid = 1;
    pos->offroute_expected_sensor = guessed_end;
    pos_clear_prediction(pos);
    /* Keep the dead-track reservation anchor above, but allow the train to
     * re-enter bootstrap on the very next tick after the stop command is sent. */
    pos->dead_track_retry_due_us = (now_us == 0) ? 1 : now_us;
}

/* Detect dead-track: no sensor fired before the deadline. */
static int tick_check_dead_track(train_pos_t *pos, uint64_t now_us) {
    if (pos->route_state != TRAIN_STATE_FIND_POS &&
        pos->route_state != TRAIN_STATE_ON_ROUTE) return 0;
    if (pos->dead_track_deadline_us == 0) return 0;
    if (now_us <= pos->dead_track_deadline_us) return 0;

    enter_terminal_dead_track(pos, now_us);
    ui_mark_position_dirty();
    return 1;
}

static int tick_handle_dead_track(train_pos_t *pos, uint64_t now_us) {
    if (!pos || pos->route_state != TRAIN_STATE_DEAD_TRACK) return 0;
    if (pos->dead_track_retry_due_us == 0) return 1;
    if (now_us < pos->dead_track_retry_due_us) return 1;

    if (pos->cur_sensor != NULL && pos->route_path_count > 0) {
        resume_onroute_after_dead_track(pos, now_us);
    } else {
        pos_enter_find_pos(pos, now_us);
    }
    return 1;
}

/* Advance a stale prediction: if more than 2x the expected interval has
 * elapsed without a sensor, skip to the next predicted node and refresh the
 * dead-track deadline from that new predicted progress point. */
static int tick_advance_prediction(train_pos_t *pos, uint64_t now_us) {
    if (pos->pred.trigger_time == 0 || pos->pred.next_sensor == NULL) return 0;
    if (pos->cur_sensor_time == 0) return 0;
    if (pos_waiting_for_first_launch_hit(pos)) return 0;
    if (pos->pred.trigger_time <= pos->cur_sensor_time) return 0;
    if (now_us <= 2 * pos->pred.trigger_time - pos->cur_sensor_time) return 0;
    if (pos->pred.skipped_sensor_count >= 1) return 0;

    track_node *skipped = pos->pred.next_sensor;
    uint64_t prev_trigger_time = pos->pred.trigger_time;
    if (pos->offroute_valid == 0 && pos->offroute_expected_sensor == NULL) {
        pos->offroute_expected_sensor = skipped;
    }
    uint64_t dt = 0;
    pos->pred.next_sensor = predict_next_sensor(pos, skipped, &dt);
    pos->pred.trigger_time = (dt > 0) ? (prev_trigger_time + dt) : 0;
    pos->pred.skipped_sensor_count = 1;
    pos_refresh_dead_track_deadline(pos, now_us);

    if (pos->target_sensor && pos->route_state == TRAIN_STATE_ON_ROUTE) {
        int32_t skip_dist = follow_dist(skipped,
            (pos->pred.next_sensor ? pos->pred.next_sensor : pos->target_sensor), 50);
        if (skip_dist > 0) {
            pos->dist_to_target_mm -= skip_dist;
            if (pos->dist_to_target_mm < 0) pos->dist_to_target_mm = 0;
        }
    }
    return 1;
}

void pos_on_tick(uint64_t now_us) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (pos->train_num < 0) continue;
        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) continue;
        if (pos->route_state == TRAIN_STATE_WAIT_SWITCH_SETTLE) continue;
        if (pos->route_state == TRAIN_STATE_DEAD_TRACK) {
            tick_handle_dead_track(pos, now_us);
            continue;
        }

        /* Update kinematic velocity every tick for accelerating trains.
         * For stopping/stopped states, the train is no longer accelerating:
         * clear the flag. */
        if (pos->is_accelerating) {
            if (pos->route_state == TRAIN_STATE_ON_ROUTE ||
                pos->route_state == TRAIN_STATE_FIND_POS ||
                pos->route_state == TRAIN_STATE_KNOWN) {
                pos_update_accel_velocity(pos, now_us);
            } else {
                pos->is_accelerating = 0;
            }
        }

        if (pos->route_state == TRAIN_STATE_RECOVERY_STOPPING) {
            tick_handle_recovery_stopping(pos, now_us);
            continue;
        }
        if (pos->route_state == TRAIN_STATE_STOPPING_TR) {
            tick_handle_stopping_tr(pos, now_us);
            continue;
        }
        if (pos->route_state == TRAIN_STATE_STOPPING_GOTO) {
            tick_handle_stopping_goto(pos, now_us);
            continue;
        }
        if (pos->route_state == TRAIN_STATE_STOPPING) {
            tick_handle_stopping(pos, now_us);
            continue;
        }
        if (pos->route_state == TRAIN_STATE_STOPPED) {
            if (pos_deadlock_maybe_resume_after_yield(pos, now_us)) continue;
            tick_handle_stopping(pos, now_us);
            continue;
        }

        if (tick_check_dead_track(pos, now_us)) continue;
        if (tick_check_brake_point(pos, now_us)) continue;
        tick_advance_prediction(pos, now_us);
    }
}
