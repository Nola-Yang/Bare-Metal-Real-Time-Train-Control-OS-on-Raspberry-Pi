#include "train_tracking/position_priv.h"
#include "train_tracking/traffic_manager.h"
#include <stddef.h>
#include <stdint.h>

static int replan_tie_rank(int train_num) {
    switch (train_num) {
    case 55: return 0;
    case 18: return 1;
    case 17: return 2;
    case 15: return 3;
    case 14: return 4;
    case 13: return 5;
    default: return 6;
    }
}

static void sort_movers(train_pos_t **arr, int count) {
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            int32_t ai = pos_route_authority_remaining_mm(arr[i]);
            int32_t aj = pos_route_authority_remaining_mm(arr[j]);
            int swap = 0;

            if (aj < ai) {
                swap = 1;
            } else if (aj == ai &&
                       replan_tie_rank(arr[j]->train_num) <
                           replan_tie_rank(arr[i]->train_num)) {
                swap = 1;
            }

            if (swap) {
                train_pos_t *tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
            }
        }
    }
}

static void sort_waiters(train_pos_t **arr, int count) {
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            int swap = 0;

            if (arr[j]->stopping_since_us < arr[i]->stopping_since_us) {
                swap = 1;
            } else if (arr[j]->stopping_since_us == arr[i]->stopping_since_us &&
                       replan_tie_rank(arr[j]->train_num) <
                           replan_tie_rank(arr[i]->train_num)) {
                swap = 1;
            }

            if (swap) {
                train_pos_t *tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
            }
        }
    }
}

static int pos_wait_resource_retry_due(train_pos_t *pos, uint64_t now_us) {
    uint32_t generation;
    int woke_on_change = 0;

    if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    generation = traffic_get_change_generation();
    if (generation != pos->replan.seen_generation) {
        pos->replan.seen_generation = generation;
        pos->replan.retry_count = 0;
        woke_on_change = 1;
    }
    if (!woke_on_change &&
        pos->replan.next_us > 0 && now_us < pos->replan.next_us) {
        return 0;
    }
    return 1;
}

static void pos_wait_resource_schedule_retry(train_pos_t *pos, uint64_t now_us) {
    int backoff_exp;
    uint64_t backoff_us;
    uint64_t jitter_us;

    if (!pos) return;

    backoff_exp = pos->replan.retry_count;
    if (backoff_exp > REPLAN_MAX_BACKOFF) backoff_exp = REPLAN_MAX_BACKOFF;
    backoff_us = REPLAN_INTERVAL_US << backoff_exp;

    pos->replan.rand_state = pos->replan.rand_state * 1664525u + 1013904223u;
    jitter_us = (pos->replan.rand_state >> 16) % (uint32_t)REPLAN_INTERVAL_US;
    pos->replan.next_us = now_us + backoff_us + jitter_us;
    pos->replan.retry_count++;
}

static void pos_replan_service_waiter(train_pos_t *pos, uint64_t now_us) {
    if (!pos_wait_resource_retry_due(pos, now_us)) return;

    pos_wait_resource_schedule_retry(pos, now_us);
    if (pos_deadlock_maybe_reroute_waiter(pos, now_us)) return;
    (void)pos_try_resume_wait_resource(pos, now_us);
}

void pos_replan_on_tick(uint64_t now_us) {
    train_pos_t *movers[MAX_POS_TRAINS];
    train_pos_t *waiters[MAX_POS_TRAINS];
    int mover_count = 0;
    int waiter_count = 0;
    uint32_t generation = traffic_get_change_generation();

    pos_deadlock_refresh_notice_state();

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        train_pos_t *pos = &g_pos[i];
        if (pos->train_num < 0) continue;

        if (pos->route_state == TRAIN_STATE_ON_ROUTE &&
            pos->route_path_count > 0 &&
            pos->route_reserved_end_cursor < pos->route_path_count - 1) {
            int32_t rem = pos_route_authority_remaining_mm(pos);
            int needs_generation_retry = generation != pos->authority_seen_generation;
            int needs_distance_retry = rem <= pos_route_authority_extend_trigger_mm(pos);
            if (needs_generation_retry || needs_distance_retry) {
                movers[mover_count++] = pos;
            }
            continue;
        }

        if (pos->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            waiters[waiter_count++] = pos;
        }
    }

    sort_movers(movers, mover_count);
    for (int i = 0; i < mover_count; i++) {
        (void)pos_route_authority_try_top_up(movers[i], now_us, 0);
    }

    sort_waiters(waiters, waiter_count);
    for (int i = 0; i < waiter_count; i++) {
        pos_replan_service_waiter(waiters[i], now_us);
    }
}
