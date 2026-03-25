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
        pos_deadlock_replan_waiter(waiters[i], now_us);
    }
}
