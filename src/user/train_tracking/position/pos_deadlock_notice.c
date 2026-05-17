#include "train_tracking/position_priv.h"
#include "game_manager.h"
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

int pos_deadlock_should_preserve_committed_route(int train_num) {
    if (game_is_active()) return 0;
    if (!g_deadlock_notice.active || g_deadlock_notice.unresolved) return 0;
    if (train_num < 0 || g_deadlock_notice.victim_train == train_num) return 0;

    for (int i = 0; i < g_deadlock_notice.cycle_count; i++) {
        if (g_deadlock_notice.cycle_trains[i] == train_num) return 1;
    }

    return 0;
}

void pos_clear_deadlock_notice(void) {
    pos_deadlock_clear_no_solution_cache();
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
    g_deadlock_notice.detect_us = 0;
    g_deadlock_notice.expire_us = 0;
    ui_mark_position_dirty();
}
