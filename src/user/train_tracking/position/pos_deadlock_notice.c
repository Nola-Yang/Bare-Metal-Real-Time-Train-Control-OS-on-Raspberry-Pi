#include "train_tracking/position_priv.h"
#include "ui.h"
#include <stddef.h>

static pos_deadlock_notice_t g_deadlock_notice;

void pos_get_deadlock_notice(pos_deadlock_notice_t *out) {
    if (!out) return;
    *out = g_deadlock_notice;
}

void pos_set_deadlock_notice(const pos_deadlock_notice_t *notice) {
    if (!notice) {
        pos_clear_deadlock_notice();
        return;
    }
    g_deadlock_notice = *notice;
    ui_mark_position_dirty();
}

void pos_clear_deadlock_notice(void) {
    g_deadlock_notice.active = 0;
    g_deadlock_notice.unresolved = 0;
    g_deadlock_notice.victim_train = -1;
    g_deadlock_notice.cycle_count = 0;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        g_deadlock_notice.cycle_trains[i] = -1;
    }
    g_deadlock_notice.blocked_target = NULL;
    g_deadlock_notice.yield_target = NULL;
    g_deadlock_notice.resume_target = NULL;
    g_deadlock_notice.expire_us = 0;
    ui_mark_position_dirty();
}
