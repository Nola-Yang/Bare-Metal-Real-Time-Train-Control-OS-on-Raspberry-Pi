#include "train_tracking/position_priv.h"
#include "train_tracking/route_priv.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

void pos_complete_switch_settle(train_pos_t *pos, uint64_t now_us) {
    if (!pos) return;

    pos->switch_settle_due_us = 0;
    pos->route_state = TRAIN_STATE_ON_ROUTE;

    if (pos->switch_settle_mode == POS_SWITCH_SETTLE_NORMAL) {
        uint64_t dt = 0;
        pos->pred.next_sensor = predict_next_sensor(pos, pos->cur_sensor, &dt);
        pos->pred.trigger_time = (dt > 0) ? now_us + dt : 0;
        pos->pred.skipped_sensor_count = 0;
    } else if (pos->switch_settle_mode == POS_SWITCH_SETTLE_REVERSED) {
        pos->pred.next_sensor = pos->cur_sensor;
        pos->pred.alt_sensor = NULL;
        pos->pred.branch_node = NULL;
        pos->pred.trigger_time = 0;
        pos->pred.skipped_sensor_count = 0;
    }

    pos_launch_at_goto_speed(pos, now_us);
    pos->route_rem_tick_us = now_us;
    pos->stopping_since_us = 0;
    pos_refresh_dead_track_deadline(pos, now_us);
    pos->switch_settle_mode = POS_SWITCH_SETTLE_NONE;
    ui_mark_position_dirty();
}

void pos_arm_switch_settle(train_pos_t *pos, int sw_count,
                           pos_switch_settle_mode_t mode, uint64_t now_us) {
    if (!pos) return;

    pos->switch_settle_mode = mode;
    if (sw_count <= 0) {
        pos_complete_switch_settle(pos, now_us);
        return;
    }

    pos->switch_settle_due_us = now_us + (uint64_t)SWITCH_SETTLE_TICKS * 10000ULL;
    pos->route_state = TRAIN_STATE_WAIT_SWITCH_SETTLE;
    ui_mark_position_dirty();
}

void pos_on_switch_settle_tick(uint64_t now_us) {
    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (pos->train_num < 0) continue;
        if (pos->route_state != TRAIN_STATE_WAIT_SWITCH_SETTLE) continue;
        if (pos->switch_settle_due_us == 0 || now_us < pos->switch_settle_due_us) continue;
        pos_complete_switch_settle(pos, now_us);
    }
}
