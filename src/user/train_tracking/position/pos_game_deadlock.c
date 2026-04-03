#include "train_tracking/position_priv.h"
#include "train_tracking/traffic_manager.h"
#include "game_manager.h"
#include <stddef.h>
#include <stdint.h>

typedef struct {
    track_node *target;
    uint8_t wait_mask;
    int priority_reduced;
    int total_blocker_reduction;
    int ready_gain;
} game_deadlock_choice_t;

/* Game deadlock evaluation runs deep inside the position server's 8 KB user
 * stack. Keep the large scratch buffers static to avoid overflowing that
 * stack while searching reroute candidates. */
static pos_route_eval_t g_game_deadlock_eval;
static int g_game_deadlock_snapshot[TRACK_MAX];
static uint8_t g_game_deadlock_seen_keys[TRACK_MAX];
static int g_game_deadlock_victims[DEADLOCK_MAX_TRAINS];

static int game_deadlock_same_physical_sensor(const track_node *a,
                                              const track_node *b) {
    if (!a || !b) return 0;
    return a == b || a->reverse == b || b->reverse == a;
}

static int game_deadlock_physical_key(track_node *node) {
    int idx;
    int ridx;

    if (!node) return -1;
    idx = (int)(node - g_track);
    if (idx < 0 || idx >= TRACK_MAX) return -1;

    ridx = node->reverse ? (int)(node->reverse - g_track) : -1;
    if (ridx >= 0 && ridx < TRACK_MAX && ridx < idx) return ridx;
    return idx;
}

static int game_deadlock_candidate_is_occupied(int requester_train,
                                               const track_node *cand) {
    if (!cand) return 0;

    for (int i = 0; i < MAX_POS_TRAINS; i++) {
        const train_pos_t *other = pos_get_by_index(i);

        if (!other || other->train_num < 0 || other->train_num == requester_train) continue;
        if (!other->cur_sensor) continue;
        if (game_deadlock_same_physical_sensor(cand, other->cur_sensor)) return 1;
    }

    return 0;
}

static track_node *game_deadlock_current_target(const train_pos_t *pos,
                                                int32_t *out_offset_mm) {
    if (out_offset_mm) *out_offset_mm = 0;
    if (!pos) return NULL;

    if (pos->orig_user_target) {
        if (out_offset_mm) *out_offset_mm = pos->orig_target_offset;
        return pos->orig_user_target;
    }
    if (pos->target_sensor) {
        if (out_offset_mm) *out_offset_mm = pos->target_offset_mm;
        return pos->target_sensor;
    }
    if (pos->pending_target) {
        if (out_offset_mm) *out_offset_mm = pos->pending_offset_mm;
        return pos->pending_target;
    }
    return NULL;
}

static uint8_t game_deadlock_wait_mask(const int *cycle_trains, int cycle_count,
                                       int victim_train) {
    uint8_t mask = 0;

    for (int i = 0; i < cycle_count; i++) {
        if (cycle_trains[i] == victim_train) continue;
        mask |= pos_deadlock_train_bit(cycle_trains[i]);
    }
    return mask;
}

static int game_deadlock_collect_counts(const int *cycle_trains, int cycle_count,
                                        int victim_train,
                                        int *out_priority_blockers,
                                        int *out_total_blockers,
                                        int *out_ready_count) {
    int priority_blockers = -1;
    int total_blockers = 0;
    int ready_count = 0;
    int any = 0;

    for (int i = 0; i < cycle_count; i++) {
        train_pos_t *other;
        uint8_t blocker_mask;
        int blocker_count;

        if (cycle_trains[i] == victim_train) continue;

        other = pos_get(cycle_trains[i]);
        if (!other || other->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

        blocker_mask = pos_wait_resource_current_blocker_mask(other);
        blocker_count = pos_route_blocker_mask_bit_count(blocker_mask);
        total_blockers += blocker_count;
        if (blocker_mask == 0) ready_count++;
        if (game_deadlock_victim_rank(other->train_num) == 2) {
            priority_blockers = blocker_count;
        }
        any = 1;
    }

    if (out_priority_blockers) *out_priority_blockers = priority_blockers;
    if (out_total_blockers) *out_total_blockers = total_blockers;
    if (out_ready_count) *out_ready_count = ready_count;
    return any;
}

static int game_deadlock_choice_better(const game_deadlock_choice_t *cand,
                                       const game_deadlock_choice_t *best) {
    if (!cand) return 0;
    if (!best || !best->target) return 1;

    if (cand->priority_reduced != best->priority_reduced) {
        return cand->priority_reduced > best->priority_reduced;
    }
    if (cand->total_blocker_reduction != best->total_blocker_reduction) {
        return cand->total_blocker_reduction > best->total_blocker_reduction;
    }
    if (cand->ready_gain != best->ready_gain) {
        return cand->ready_gain > best->ready_gain;
    }
    return (int)(cand->target - g_track) < (int)(best->target - g_track);
}

static int game_deadlock_candidate_improves(int priority_reduced,
                                            int total_blocker_reduction,
                                            int ready_gain) {
    return priority_reduced || total_blocker_reduction > 0 || ready_gain > 0;
}

static int game_deadlock_evaluate_candidate(const int *cycle_trains,
                                            int cycle_count,
                                            train_pos_t *victim,
                                            track_node *cand,
                                            int use_short_move,
                                            int old_priority_blockers,
                                            int old_total_blockers,
                                            int old_ready_count,
                                            game_deadlock_choice_t *out) {
    pos_route_eval_t *eval = &g_game_deadlock_eval;
    pos_route_eval_result_t result;
    int new_priority_blockers = -1;
    int new_total_blockers = 0;
    int new_ready_count = 0;
    int priority_reduced;
    int total_blocker_reduction;
    int ready_gain;
    track_node *keep_end;

    if (!victim || !cand || !out) return 0;

    result = use_short_move
                 ? pos_evaluate_target_short_ready_now(victim, cand, eval)
                 : pos_evaluate_target_ready_now(victim, cand, eval);
    if (result != POS_ROUTE_EVAL_READY) return 0;

    keep_end = pos_release_keep_end(cand, NULL);
    traffic_snapshot_reservations(g_game_deadlock_snapshot);
    traffic_simulate_parked_train(victim->train_num, cand, TRAIN_BODY_MM, keep_end);
    (void)game_deadlock_collect_counts(cycle_trains, cycle_count, victim->train_num,
                                       &new_priority_blockers,
                                       &new_total_blockers,
                                       &new_ready_count);
    traffic_restore_reservations(g_game_deadlock_snapshot);

    priority_reduced =
        old_priority_blockers >= 0 &&
        new_priority_blockers >= 0 &&
        new_priority_blockers < old_priority_blockers;
    total_blocker_reduction = old_total_blockers - new_total_blockers;
    ready_gain = new_ready_count - old_ready_count;
    if (!game_deadlock_candidate_improves(priority_reduced,
                                          total_blocker_reduction,
                                          ready_gain)) {
        return 0;
    }

    out->target = cand;
    out->wait_mask = game_deadlock_wait_mask(cycle_trains, cycle_count,
                                             victim->train_num);
    out->priority_reduced = priority_reduced;
    out->total_blocker_reduction = total_blocker_reduction;
    out->ready_gain = ready_gain;
    return 1;
}

static int game_deadlock_search_victim_targets(const int *cycle_trains,
                                               int cycle_count,
                                               train_pos_t *victim,
                                               int use_short_move,
                                               game_deadlock_choice_t *out_best) {
    track_node *preferred_target;
    track_node *current_target;
    int32_t target_offset = 0;
    int old_priority_blockers = -1;
    int old_total_blockers = 0;
    int old_ready_count = 0;
    game_deadlock_choice_t best = {0};

    if (!victim || !out_best) return 0;

    current_target = game_deadlock_current_target(victim, &target_offset);
    (void)target_offset;
    (void)game_deadlock_collect_counts(cycle_trains, cycle_count, victim->train_num,
                                       &old_priority_blockers,
                                       &old_total_blockers,
                                       &old_ready_count);

    for (int i = 0; i < TRACK_MAX; i++) g_game_deadlock_seen_keys[i] = 0;

    preferred_target = game_deadlock_preferred_yield_target(victim->train_num);
    if (preferred_target) {
        int key = game_deadlock_physical_key(preferred_target);
        game_deadlock_choice_t choice = {0};

        if (key >= 0 && key < TRACK_MAX) g_game_deadlock_seen_keys[key] = 1;
        if (!game_deadlock_same_physical_sensor(preferred_target, victim->cur_sensor) &&
            !game_deadlock_same_physical_sensor(preferred_target, current_target) &&
            !game_deadlock_candidate_is_occupied(victim->train_num, preferred_target) &&
            game_deadlock_evaluate_candidate(cycle_trains, cycle_count, victim,
                                             preferred_target, use_short_move,
                                             old_priority_blockers,
                                             old_total_blockers,
                                             old_ready_count, &choice)) {
            *out_best = choice;
            return 1;
        }
    }

    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *cand = &g_track[i];
        int key;
        game_deadlock_choice_t choice = {0};

        if (cand->type != NODE_SENSOR) continue;
        key = game_deadlock_physical_key(cand);
        if (key < 0 || key >= TRACK_MAX) continue;
        if (g_game_deadlock_seen_keys[key]) continue;
        g_game_deadlock_seen_keys[key] = 1;

        if (game_deadlock_same_physical_sensor(cand, victim->cur_sensor)) continue;
        if (game_deadlock_same_physical_sensor(cand, current_target)) continue;
        if (game_deadlock_candidate_is_occupied(victim->train_num, cand)) continue;

        if (!game_deadlock_evaluate_candidate(cycle_trains, cycle_count, victim,
                                              cand, use_short_move,
                                              old_priority_blockers,
                                              old_total_blockers,
                                              old_ready_count, &choice)) {
            continue;
        }

        if (game_deadlock_choice_better(&choice, &best)) {
            best = choice;
        }
    }

    if (!best.target) return 0;
    *out_best = best;
    return 1;
}

static int game_deadlock_sort_victims(const int *cycle_trains, int cycle_count,
                                      int *out_victims) {
    int victim_count = 0;

    for (int i = 0; i < cycle_count; i++) {
        train_pos_t *pos = pos_get(cycle_trains[i]);
        int rank;

        if (!pos || pos->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        rank = game_deadlock_victim_rank(cycle_trains[i]);
        if (rank < 0) continue;
        out_victims[victim_count++] = cycle_trains[i];
    }

    for (int i = 0; i < victim_count; i++) {
        for (int j = i + 1; j < victim_count; j++) {
            int rank_i = game_deadlock_victim_rank(out_victims[i]);
            int rank_j = game_deadlock_victim_rank(out_victims[j]);
            int swap = 0;

            if (rank_j < rank_i) {
                swap = 1;
            } else if (rank_j == rank_i && out_victims[j] < out_victims[i]) {
                swap = 1;
            }

            if (swap) {
                int tmp = out_victims[i];
                out_victims[i] = out_victims[j];
                out_victims[j] = tmp;
            }
        }
    }

    return victim_count;
}

int pos_game_deadlock_try_resolve(const int *cycle_trains, int cycle_count,
                                  uint64_t now_us,
                                  int *out_victim_train,
                                  track_node **out_target,
                                  uint8_t *out_wait_mask,
                                  int *out_use_short_move) {
    int victim_count;

    (void)now_us;

    if (out_victim_train) *out_victim_train = -1;
    if (out_target) *out_target = NULL;
    if (out_wait_mask) *out_wait_mask = 0;
    if (out_use_short_move) *out_use_short_move = 0;

    if (!cycle_trains || cycle_count < 2 || cycle_count > DEADLOCK_MAX_TRAINS) return 0;
    if (!game_deadlock_mode_active()) return 0;

    for (int i = 0; i < cycle_count; i++) {
        if (game_deadlock_victim_rank(cycle_trains[i]) < 0) return 0;
    }

    victim_count = game_deadlock_sort_victims(cycle_trains, cycle_count,
                                              g_game_deadlock_victims);
    if (victim_count <= 0) return 0;

    for (int phase = 0; phase < 2; phase++) {
        int use_short_move = (phase == 1);

        for (int i = 0; i < victim_count; i++) {
            train_pos_t *victim = pos_get(g_game_deadlock_victims[i]);
            game_deadlock_choice_t choice = {0};

            if (!victim || victim->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
            if (!game_deadlock_search_victim_targets(cycle_trains, cycle_count,
                                                     victim, use_short_move,
                                                     &choice)) {
                continue;
            }

            if (out_victim_train) *out_victim_train = victim->train_num;
            if (out_target) *out_target = choice.target;
            if (out_wait_mask) *out_wait_mask = choice.wait_mask;
            if (out_use_short_move) *out_use_short_move = use_short_move;
            return 1;
        }
    }

    if (victim_count > 0) {
        train_pos_t *victim = pos_get(g_game_deadlock_victims[0]);
        track_node *blocked_target = game_deadlock_current_target(victim, NULL);
        (void)game_deadlock_handle_no_solution(cycle_trains, cycle_count,
                                               g_game_deadlock_victims[0],
                                               blocked_target);
    }

    return 0;
}
