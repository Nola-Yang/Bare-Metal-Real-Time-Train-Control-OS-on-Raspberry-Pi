#include "train_tracking/deadlock.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "train_tracking/speed_table.h"
#include "demo_manager.h"
#include "../traffic/traffic_window_internal.h"
#include "track.h"
#include <stddef.h>
#include <stdint.h>

typedef enum {
    SNAPSHOT_DEADLOCK_KIND_NONE = 0,
    SNAPSHOT_DEADLOCK_KIND_WAIT_CYCLE = 1,
    SNAPSHOT_DEADLOCK_KIND_STOPPED_BLOCKER = 2,
} snapshot_deadlock_kind_t;

typedef struct {
    route_plan_t plan;
    track_node   *chosen_origin;
    int          need_initial_reverse;
    uint8_t      blocker_mask;
} snapshot_eval_t;

typedef struct {
    int count;
    const deadlock_participant_snapshot_t *participants[DEADLOCK_MAX_TRAINS];
    int train_nums[DEADLOCK_MAX_TRAINS];
    uint8_t global_bits[DEADLOCK_MAX_TRAINS];
    uint8_t wait_mask;
    uint8_t stopped_mask;
} snapshot_participants_t;

typedef struct {
    const deadlock_snapshot_t *snapshot;
    const int *owners;
} snapshot_ctx_t;

static route_plan_t g_snapshot_ready_plan;
static route_plan_t g_snapshot_blocked_plan;
static route_plan_t g_snapshot_authority_candidate_prefix;
static route_plan_t g_snapshot_authority_short_goal_prefix;
static route_plan_t g_snapshot_launch_prefix;
static uint8_t g_snapshot_blocked[TRACK_MAX];
static char g_snapshot_fixed_sw_dirs[TRACK_MAX];
static uint16_t g_snapshot_sorted_from_origin0[TRACK_MAX];
static uint16_t g_snapshot_sorted_from_origin1[TRACK_MAX];
static uint16_t g_snapshot_merged_candidates[TRACK_MAX];
static uint8_t g_snapshot_seen_candidate[TRACK_MAX];

static int snapshot_train_to_index(int train_num) {
    switch (train_num) {
    case 13: return 0;
    case 14: return 1;
    case 15: return 2;
    case 17: return 3;
    case 18: return 4;
    case 55: return 5;
    default: return -1;
    }
}

static uint8_t snapshot_train_bit(int train_num) {
    int idx = snapshot_train_to_index(train_num);
    return (idx >= 0) ? (uint8_t)(1u << idx) : 0;
}

static int snapshot_reverse_index(int idx) {
    track_node *rev;

    if (idx < 0 || idx >= TRACK_MAX) return -1;
    rev = g_track[idx].reverse;
    if (!rev) return -1;
    idx = (int)(rev - g_track);
    return (idx >= 0 && idx < TRACK_MAX) ? idx : -1;
}

static track_node *snapshot_node_from_index(int idx) {
    if (idx < 0 || idx >= TRACK_MAX) return NULL;
    return &g_track[idx];
}

static int snapshot_same_physical_sensor_idx(int a_idx, int b_idx) {
    track_node *a = snapshot_node_from_index(a_idx);
    track_node *b = snapshot_node_from_index(b_idx);

    if (!a || !b) return 0;
    return a == b || a->reverse == b || b->reverse == a;
}

static void snapshot_deadlock_route_plan_clear(deadlock_route_plan_t *out) {
    if (!out) return;
    *out = (deadlock_route_plan_t){0};
    out->chosen_target_idx = -1;
    out->reversal_sensor_idx = -1;
}

static void snapshot_deadlock_result_clear(deadlock_result_t *out) {
    if (!out) return;
    *out = (deadlock_result_t){0};
    out->action = DEADLOCK_RESULT_NONE;
    out->victim_train = -1;
    out->blocked_target_idx = -1;
    out->yield_target_idx = -1;
    out->resume_target_idx = -1;
    out->chosen_origin_idx = -1;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        out->cycle_trains[i] = -1;
        out->participants[i].train_num = -1;
    }
    snapshot_deadlock_route_plan_clear(&out->route_plan);
}

static void snapshot_serialize_route_plan(deadlock_route_plan_t *out,
                                          const route_plan_t *plan) {
    if (!out) return;

    snapshot_deadlock_route_plan_clear(out);
    if (!plan) return;

    out->sw_count = plan->sw_count;
    for (int i = 0; i < plan->sw_count; i++) {
        out->sw_nums[i] = plan->sw_nums[i];
        out->sw_dirs[i] = plan->sw_dirs[i];
    }
    out->total_dist_mm = plan->total_dist_mm;
    out->chosen_target_idx = plan->chosen_target ? (int)(plan->chosen_target - g_track) : -1;
    out->has_reversal = plan->has_reversal;
    out->reversal_sensor_idx =
        plan->reversal_sensor ? (int)(plan->reversal_sensor - g_track) : -1;
    out->dist_to_reversal_mm = plan->dist_to_reversal_mm;
    out->sw_count2 = plan->sw_count2;
    for (int i = 0; i < plan->sw_count2; i++) {
        out->sw_nums2[i] = plan->sw_nums2[i];
        out->sw_dirs2[i] = plan->sw_dirs2[i];
    }
    out->dist_after_reversal_mm = plan->dist_after_reversal_mm;
    out->path_count = plan->path_count;
    for (int i = 0; i < plan->path_count; i++) {
        out->path_nodes[i] = plan->path_nodes[i];
    }
    out->path_count2 = plan->path_count2;
    for (int i = 0; i < plan->path_count2; i++) {
        out->path_nodes2[i] = plan->path_nodes2[i];
    }
}

static int snapshot_bit_count(uint8_t mask) {
    int count = 0;

    while (mask) {
        count += (mask & 1u);
        mask >>= 1;
    }
    return count;
}

static int snapshot_route_plan_long_enough(const route_plan_t *plan,
                                           int32_t threshold) {
    int32_t effective_d;

    if (!plan) return 0;
    effective_d = plan->has_reversal
                  ? plan->dist_to_reversal_mm + plan->dist_after_reversal_mm
                  : plan->total_dist_mm;
    return effective_d > threshold;
}

static int snapshot_train_is_manual_stop_blocker(
    const deadlock_snapshot_t *snapshot,
    const deadlock_participant_snapshot_t *part) {
    if (!part || part->route_state != TRAIN_STATE_STOPPED) return 0;
    return !snapshot || !snapshot->auto_dispatching_targets;
}

static int snapshot_train_is_yield_stop_blocker(
    const deadlock_participant_snapshot_t *part) {
    if (!part || part->route_state != TRAIN_STATE_STOPPED) return 0;
    if (part->resume_target_idx < 0) return 0;
    if (part->yield_target_idx < 0) return 0;
    return part->parked_at_yield != 0;
}

static int snapshot_train_is_stopped_blocker(
    const deadlock_snapshot_t *snapshot,
    const deadlock_participant_snapshot_t *part) {
    return snapshot_train_is_manual_stop_blocker(snapshot, part) ||
           snapshot_train_is_yield_stop_blocker(part);
}

static void snapshot_collect_participants(const deadlock_snapshot_t *snapshot,
                                          snapshot_participants_t *parts) {
    if (!parts) return;

    parts->count = 0;
    parts->wait_mask = 0;
    parts->stopped_mask = 0;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        parts->participants[i] = NULL;
        parts->train_nums[i] = -1;
        parts->global_bits[i] = 0;
    }
    if (!snapshot) return;

    for (int i = 0; i < snapshot->participant_count && parts->count < DEADLOCK_MAX_TRAINS; i++) {
        const deadlock_participant_snapshot_t *part = &snapshot->participants[i];
        uint8_t local_bit;

        if (!part || part->train_num < 0) continue;
        if (part->route_state != TRAIN_STATE_WAIT_RESOURCE &&
            !snapshot_train_is_stopped_blocker(snapshot, part)) {
            continue;
        }

        local_bit = (uint8_t)(1u << parts->count);
        parts->participants[parts->count] = part;
        parts->train_nums[parts->count] = part->train_num;
        parts->global_bits[parts->count] = snapshot_train_bit(part->train_num);
        if (part->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            parts->wait_mask |= local_bit;
        } else {
            parts->stopped_mask |= local_bit;
        }
        parts->count++;
    }
}

static int snapshot_participant_index(const snapshot_participants_t *parts,
                                      int train_num) {
    if (!parts) return -1;
    for (int i = 0; i < parts->count; i++) {
        if (parts->train_nums[i] == train_num) return i;
    }
    return -1;
}

static uint8_t snapshot_global_mask_from_local(
    const snapshot_participants_t *parts, uint8_t local_mask) {
    uint8_t global_mask = 0;

    if (!parts) return 0;
    for (int i = 0; i < parts->count; i++) {
        if (local_mask & (uint8_t)(1u << i)) {
            global_mask |= parts->global_bits[i];
        }
    }
    return global_mask;
}

static void snapshot_build_graph(const snapshot_participants_t *parts,
                                 uint8_t adj[DEADLOCK_MAX_TRAINS]) {
    if (!parts || !adj) return;

    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) adj[i] = 0;

    for (int i = 0; i < parts->count; i++) {
        const deadlock_participant_snapshot_t *part;

        if (!(parts->wait_mask & (uint8_t)(1u << i))) continue;
        part = parts->participants[i];
        if (!part || part->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;

        for (int j = 0; j < parts->count; j++) {
            if (i == j) continue;
            if (!(parts->wait_mask & (uint8_t)(1u << j))) continue;
            if (part->blocker_mask & parts->global_bits[j]) {
                adj[i] |= (uint8_t)(1u << j);
            }
        }
    }
}

static void snapshot_compute_reachability(const uint8_t adj[DEADLOCK_MAX_TRAINS],
                                          int count,
                                          uint8_t wait_mask,
                                          uint8_t reach[DEADLOCK_MAX_TRAINS]) {
    for (int i = 0; i < count; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        reach[i] = (wait_mask & bit) ? (adj[i] | bit) : 0;
    }

    for (int pass = 0; pass < count; pass++) {
        for (int i = 0; i < count; i++) {
            uint8_t expanded = reach[i];
            if (!(wait_mask & (uint8_t)(1u << i))) continue;
            for (int j = 0; j < count; j++) {
                if (expanded & (uint8_t)(1u << j)) expanded |= reach[j];
            }
            reach[i] = expanded & wait_mask;
        }
    }
}

static uint8_t snapshot_find_stopped_blocker_mask_for_train(
    int train_num, const snapshot_participants_t *parts) {
    const deadlock_participant_snapshot_t *part;
    int start_idx = snapshot_participant_index(parts, train_num);
    uint8_t blockers = 0;

    if (!parts || start_idx < 0) return 0;
    part = parts->participants[start_idx];
    if (!part || part->route_state != TRAIN_STATE_WAIT_RESOURCE) return 0;

    for (int i = 0; i < parts->count; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        if (!(parts->stopped_mask & bit)) continue;
        if (part->blocker_mask & parts->global_bits[i]) blockers |= bit;
    }

    if (blockers == 0) return 0;
    return (uint8_t)(blockers | (uint8_t)(1u << start_idx));
}

static uint8_t snapshot_find_mask_for_train(int train_num,
                                            const snapshot_participants_t *parts,
                                            snapshot_deadlock_kind_t *out_kind) {
    uint8_t adj[DEADLOCK_MAX_TRAINS];
    uint8_t reach[DEADLOCK_MAX_TRAINS];
    uint8_t cycle = 0;
    uint8_t stopped_blockers;
    int start_idx;

    if (out_kind) *out_kind = SNAPSHOT_DEADLOCK_KIND_NONE;
    if (!parts) return 0;

    start_idx = snapshot_participant_index(parts, train_num);
    if (start_idx < 0) return 0;

    snapshot_build_graph(parts, adj);
    if (parts->wait_mask & (uint8_t)(1u << start_idx)) {
        snapshot_compute_reachability(adj, parts->count, parts->wait_mask, reach);
        for (int i = 0; i < parts->count; i++) {
            uint8_t bit = (uint8_t)(1u << i);
            if (!(parts->wait_mask & bit)) continue;
            if ((reach[start_idx] & bit) &&
                (reach[i] & (uint8_t)(1u << start_idx))) {
                cycle |= bit;
            }
        }
        if (snapshot_bit_count(cycle) >= 2) {
            if (out_kind) *out_kind = SNAPSHOT_DEADLOCK_KIND_WAIT_CYCLE;
            return cycle;
        }
    }

    stopped_blockers = snapshot_find_stopped_blocker_mask_for_train(train_num,
                                                                    parts);
    if (snapshot_bit_count(stopped_blockers) >= 2) {
        if (out_kind) *out_kind = SNAPSHOT_DEADLOCK_KIND_STOPPED_BLOCKER;
        return stopped_blockers;
    }
    return 0;
}

static int snapshot_choose_victim(const snapshot_participants_t *parts,
                                  uint8_t cycle_mask,
                                  snapshot_deadlock_kind_t kind) {
    if (!parts) return -1;

    if (kind == SNAPSHOT_DEADLOCK_KIND_STOPPED_BLOCKER) {
        for (int i = 0; i < parts->count; i++) {
            const deadlock_participant_snapshot_t *part = parts->participants[i];
            if (!(cycle_mask & (uint8_t)(1u << i))) continue;
            if (part && part->route_state == TRAIN_STATE_STOPPED) {
                return parts->train_nums[i];
            }
        }
    }

    for (int i = 0; i < parts->count; i++) {
        const deadlock_participant_snapshot_t *part = parts->participants[i];
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        if (part && part->route_state == TRAIN_STATE_WAIT_RESOURCE) {
            return parts->train_nums[i];
        }
    }

    for (int i = 0; i < parts->count; i++) {
        if (cycle_mask & (uint8_t)(1u << i)) return parts->train_nums[i];
    }
    return -1;
}

static int snapshot_goal_idx(const deadlock_participant_snapshot_t *part) {
    return part ? part->goal_idx : -1;
}

static void snapshot_fill_origins(const deadlock_participant_snapshot_t *part,
                                  track_node *origins[2]) {
    if (!origins) return;
    origins[0] = part ? snapshot_node_from_index(part->origin0_idx) : NULL;
    origins[1] = part ? snapshot_node_from_index(part->origin1_idx) : NULL;
}

static int32_t snapshot_early_stop_mm(const deadlock_participant_snapshot_t *part) {
    int32_t v;

    if (!part) return 0;
    v = speed_table_get_v(part->train_ind, part->goto_speed);
    if (v <= 0) return 0;
    return (int32_t)((int64_t)v *
                     (int64_t)speed_table_get_early_stop(part->train_ind,
                                                         part->goto_speed) /
                     1000000LL);
}

static int32_t snapshot_brake_dist_mm(const deadlock_participant_snapshot_t *part) {
    int32_t v;
    int32_t a;

    if (!part) return 0;
    v = speed_table_get_v(part->train_ind, part->goto_speed);
    a = speed_table_get_nominal_decel(part->train_ind, part->goto_speed);
    if (v <= 0 || a <= 0) return 0;
    return v * v / (2 * a);
}

static int32_t snapshot_stop_dist_mm(const deadlock_participant_snapshot_t *part) {
    int32_t brake = snapshot_brake_dist_mm(part);
    int32_t early = snapshot_early_stop_mm(part);

    if (brake < 0) brake = 0;
    if (early < 0) early = 0;
    return brake + early;
}

static int32_t snapshot_min_window_mm(const deadlock_participant_snapshot_t *part) {
    int32_t brake = snapshot_brake_dist_mm(part);
    int32_t early = snapshot_early_stop_mm(part);

    if (brake < 0) brake = 0;
    if (early < 0) early = 0;
    return GOTO_MIN_DIST_FACTOR * brake + early;
}

static int snapshot_route_switch_needs_change(const snapshot_ctx_t *ctx,
                                              int sw_num,
                                              char desired_dir) {
    int sw_idx;
    char current_dir;

    if (!ctx) return 1;
    sw_idx = track_switch_to_index(sw_num);
    if (sw_idx < 0) return 1;
    current_dir = ctx->snapshot->switch_state[sw_idx];
    return current_dir != desired_dir;
}

static int snapshot_switch_envelope_owner(const int owners[TRACK_MAX], int sw_num) {
    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        int checks[6];
        int c = 0;

        if (n->type != NODE_BRANCH || n->num != sw_num) continue;

        checks[c++] = i;
        checks[c++] = snapshot_reverse_index(i);
        if (n->edge[DIR_STRAIGHT].dest) {
            int idx = (int)(n->edge[DIR_STRAIGHT].dest - g_track);
            checks[c++] = idx;
            checks[c++] = snapshot_reverse_index(idx);
        }
        if (n->edge[DIR_CURVED].dest) {
            int idx = (int)(n->edge[DIR_CURVED].dest - g_track);
            checks[c++] = idx;
            checks[c++] = snapshot_reverse_index(idx);
        }

        for (int j = 0; j < c; j++) {
            int idx = checks[j];
            if (idx < 0 || idx >= TRACK_MAX) continue;
            if (owners[idx] >= 0) return owners[idx];
        }
        break;
    }
    return -1;
}

static int snapshot_route_switch_blocker(const snapshot_ctx_t *ctx,
                                         const int *sw_nums,
                                         const char *sw_dirs,
                                         int sw_count,
                                         int requester_train) {
    (void)requester_train;
    for (int i = sw_count - 1; i >= 0; i--) {
        if (!snapshot_route_switch_needs_change(ctx, sw_nums[i], sw_dirs[i])) continue;
        if (snapshot_switch_envelope_owner(ctx->owners, sw_nums[i]) >= 0) {
            return snapshot_switch_envelope_owner(ctx->owners, sw_nums[i]);
        }
    }
    return -1;
}

static void snapshot_build_constraints_for_train(const snapshot_ctx_t *ctx,
                                                 int requester_train,
                                                 uint8_t blocked[TRACK_MAX],
                                                 char fixed_sw_dirs[TRACK_MAX]) {
    if (blocked) {
        for (int i = 0; i < TRACK_MAX; i++) {
            int owner = ctx->owners[i];
            blocked[i] = (owner >= 0 && owner != requester_train) ? 1 : 0;
        }
        traffic_expand_zone_marks(blocked);
    }

    if (fixed_sw_dirs) {
        for (int i = 0; i < TRACK_MAX; i++) fixed_sw_dirs[i] = '?';
        for (int i = 0; i < TRACK_MAX; i++) {
            track_node *n = &g_track[i];
            int sw_idx;
            char current_dir;

            if (n->type != NODE_BRANCH) continue;
            sw_idx = track_switch_to_index(n->num);
            if (sw_idx < 0) continue;
            current_dir = ctx->snapshot->switch_state[sw_idx];
            if (current_dir != 'S' && current_dir != 'C') continue;
            if (snapshot_switch_envelope_owner(ctx->owners, n->num) == requester_train) {
                fixed_sw_dirs[i] = current_dir;
            }
        }
    }
}

static int snapshot_plan_has_conflict(const int owners[TRACK_MAX],
                                      int train_num,
                                      const uint8_t want[TRACK_MAX]) {
    for (int i = 0; i < TRACK_MAX; i++) {
        if (!want[i]) continue;
        if (owners[i] >= 0 && owners[i] != train_num) return 1;
    }
    return 0;
}

static int snapshot_can_reserve_plan(const snapshot_ctx_t *ctx,
                                     int train_num,
                                     const route_plan_t *plan) {
    uint8_t want[TRACK_MAX];

    if (!ctx || !plan || train_num < 0) return 0;
    traffic_build_plan_marks_copy(plan, want);
    return !snapshot_plan_has_conflict(ctx->owners, train_num, want);
}

static uint8_t snapshot_blocker_mask_from_trains(int requester_train,
                                                 const int *trains,
                                                 int count) {
    uint8_t mask = 0;

    for (int i = 0; i < count; i++) {
        if (trains[i] == requester_train) continue;
        mask |= snapshot_train_bit(trains[i]);
    }
    return mask;
}

static int snapshot_collect_plan_blockers(const snapshot_ctx_t *ctx,
                                          int requester_train,
                                          const route_plan_t *plan,
                                          int *out_trains,
                                          int max_trains) {
    uint8_t want[TRACK_MAX];
    int unique[6];
    int total = 0;

    if (!ctx || !plan) return 0;

    traffic_build_plan_marks_copy(plan, want);
    for (int i = 0; i < TRACK_MAX; i++) {
        int owner;
        int seen = 0;

        if (!want[i]) continue;
        owner = ctx->owners[i];
        if (owner < 0 || owner == requester_train) continue;
        for (int j = 0; j < total; j++) {
            if (unique[j] == owner) {
                seen = 1;
                break;
            }
        }
        if (seen) continue;
        if (total < 6) unique[total] = owner;
        total++;
    }

    for (int i = 0; out_trains && i < total && i < max_trains; i++) {
        out_trains[i] = unique[i];
    }
    return total;
}

static int snapshot_collect_switch_blockers(const snapshot_ctx_t *ctx,
                                            int sw_num,
                                            int *out_trains,
                                            int max_trains) {
    int unique[6];
    int total = 0;

    if (!ctx) return 0;

    for (int i = 0; i < TRACK_MAX; i++) {
        track_node *n = &g_track[i];
        int checks[6];
        int c = 0;

        if (n->type != NODE_BRANCH || n->num != sw_num) continue;

        checks[c++] = i;
        checks[c++] = snapshot_reverse_index(i);
        if (n->edge[DIR_STRAIGHT].dest) {
            int idx = (int)(n->edge[DIR_STRAIGHT].dest - g_track);
            checks[c++] = idx;
            checks[c++] = snapshot_reverse_index(idx);
        }
        if (n->edge[DIR_CURVED].dest) {
            int idx = (int)(n->edge[DIR_CURVED].dest - g_track);
            checks[c++] = idx;
            checks[c++] = snapshot_reverse_index(idx);
        }

        for (int j = 0; j < c; j++) {
            int idx = checks[j];
            int owner;
            int seen = 0;

            if (idx < 0 || idx >= TRACK_MAX) continue;
            owner = ctx->owners[idx];
            if (owner < 0) continue;
            for (int k = 0; k < total; k++) {
                if (unique[k] == owner) {
                    seen = 1;
                    break;
                }
            }
            if (seen) continue;
            if (total < 6) unique[total] = owner;
            total++;
        }
        break;
    }

    for (int i = 0; out_trains && i < total && i < max_trains; i++) {
        out_trains[i] = unique[i];
    }
    return total;
}

static uint8_t snapshot_route_blocker_mask_from_plan(const snapshot_ctx_t *ctx,
                                                     int requester_train,
                                                     const route_plan_t *plan) {
    int blockers[6];
    int count = snapshot_collect_plan_blockers(ctx, requester_train, plan,
                                               blockers, 6);
    return snapshot_blocker_mask_from_trains(requester_train, blockers, count);
}

static uint8_t snapshot_route_blocker_mask_from_switches(
    const snapshot_ctx_t *ctx, const int *sw_nums, const char *sw_dirs,
    int sw_count, int requester_train) {
    uint8_t mask = 0;

    if (!ctx) return 0;
    for (int i = sw_count - 1; i >= 0; i--) {
        int blockers[6];
        int count;

        if (!snapshot_route_switch_needs_change(ctx, sw_nums[i], sw_dirs[i])) continue;
        count = snapshot_collect_switch_blockers(ctx, sw_nums[i], blockers, 6);
        mask |= snapshot_blocker_mask_from_trains(requester_train, blockers, count);
    }
    return mask;
}

static void snapshot_note_best_plan(const route_plan_t *cand,
                                    track_node *cand_origin,
                                    int cand_need_initial_reverse,
                                    route_plan_t *best_plan,
                                    track_node **best_origin,
                                    int *best_need_initial_reverse,
                                    int *have_best_plan) {
    if (!cand || !best_plan || !best_origin ||
        !best_need_initial_reverse || !have_best_plan) {
        return;
    }
    if (!*have_best_plan || cand->total_dist_mm < best_plan->total_dist_mm) {
        *best_plan = *cand;
        *best_origin = cand_origin;
        *best_need_initial_reverse = cand_need_initial_reverse;
        *have_best_plan = 1;
    }
}

static int snapshot_select_best_route_for_origins(
    track_node *origins[2], track_node *user_target, int32_t d_stop,
    int32_t threshold, const uint8_t *blocked, const char *fixed_sw_dirs,
    route_plan_t *out_plan, track_node **out_origin,
    int *out_need_initial_reverse) {
    int have_best_plan = 0;
    route_plan_t cand_plan;
    route_plan_t best_plan;
    track_node *best_origin = NULL;
    int best_need_initial_reverse = 0;

    if (out_plan) *out_plan = (route_plan_t){0};
    if (out_origin) *out_origin = NULL;
    if (out_need_initial_reverse) *out_need_initial_reverse = 0;

    for (int o = 0; o < 2; o++) {
        if (!origins[o]) continue;

        if (bfs_find_route_optimal_constrained(origins[o], user_target, d_stop,
                                               blocked, fixed_sw_dirs,
                                               &cand_plan) &&
            snapshot_route_plan_long_enough(&cand_plan, threshold)) {
            snapshot_note_best_plan(&cand_plan, origins[o], o == 1,
                                    &best_plan, &best_origin,
                                    &best_need_initial_reverse,
                                    &have_best_plan);
        }
    }

    if (!have_best_plan && origins[1] &&
        bfs_find_bootstrap_midrev(origins[1], user_target, d_stop, blocked,
                                  fixed_sw_dirs, &cand_plan) &&
        snapshot_route_plan_long_enough(&cand_plan, threshold)) {
        snapshot_note_best_plan(&cand_plan, origins[1], 1, &best_plan,
                                &best_origin, &best_need_initial_reverse,
                                &have_best_plan);
    }

    if (!have_best_plan) return 0;

    if (out_plan) *out_plan = best_plan;
    if (out_origin) *out_origin = best_origin;
    if (out_need_initial_reverse) *out_need_initial_reverse = best_need_initial_reverse;
    return 1;
}

static pos_route_eval_result_t snapshot_evaluate_target_plan_internal(
    const snapshot_ctx_t *ctx, const deadlock_participant_snapshot_t *part,
    track_node *user_target, int allow_blocked_fallback, snapshot_eval_t *out) {
    route_plan_t *ready_plan = &g_snapshot_ready_plan;
    route_plan_t *blocked_plan = &g_snapshot_blocked_plan;
    track_node *origins[2];
    track_node *chosen_origin = NULL;
    track_node *blocked_origin = NULL;
    int need_initial_reverse = 0;
    int blocked_need_initial_reverse = 0;
    int32_t d_stop;
    int32_t threshold;

    if (out) {
        out->plan = (route_plan_t){0};
        out->chosen_origin = NULL;
        out->need_initial_reverse = 0;
        out->blocker_mask = 0;
    }
    if (!ctx || !part || !user_target || part->cur_sensor_idx < 0) {
        return POS_ROUTE_EVAL_UNREACHABLE;
    }

    snapshot_fill_origins(part, origins);
    d_stop = snapshot_stop_dist_mm(part);
    threshold = snapshot_min_window_mm(part);
    if (d_stop <= 0 || threshold <= 0) return POS_ROUTE_EVAL_UNREACHABLE;

    snapshot_build_constraints_for_train(ctx, part->train_num, g_snapshot_blocked,
                                         g_snapshot_fixed_sw_dirs);

    if (snapshot_select_best_route_for_origins(origins, user_target, d_stop,
                                               threshold, g_snapshot_blocked,
                                               g_snapshot_fixed_sw_dirs,
                                               ready_plan, &chosen_origin,
                                               &need_initial_reverse)) {
        if (out) {
            out->plan = *ready_plan;
            out->chosen_origin = chosen_origin;
            out->need_initial_reverse = need_initial_reverse;
            out->blocker_mask = 0;
        }
        return POS_ROUTE_EVAL_READY;
    }

    if (!snapshot_select_best_route_for_origins(origins, user_target, d_stop,
                                                threshold, NULL,
                                                g_snapshot_fixed_sw_dirs,
                                                blocked_plan, &blocked_origin,
                                                &blocked_need_initial_reverse)) {
        return POS_ROUTE_EVAL_UNREACHABLE;
    }

    if (allow_blocked_fallback) {
        if (out) {
            out->plan = *blocked_plan;
            out->chosen_origin = blocked_origin;
            out->need_initial_reverse = blocked_need_initial_reverse;
            out->blocker_mask = 0;
        }
        return POS_ROUTE_EVAL_READY;
    }

    if (out) {
        out->plan = *blocked_plan;
        out->chosen_origin = blocked_origin;
        out->need_initial_reverse = blocked_need_initial_reverse;
        out->blocker_mask =
            snapshot_route_blocker_mask_from_plan(ctx, part->train_num,
                                                  blocked_plan) |
            snapshot_route_blocker_mask_from_switches(
                ctx, blocked_plan->sw_nums, blocked_plan->sw_dirs,
                blocked_plan->sw_count, part->train_num);
    }
    return POS_ROUTE_EVAL_BLOCKED;
}

static pos_route_eval_result_t snapshot_evaluate_target_plan(
    const snapshot_ctx_t *ctx, const deadlock_participant_snapshot_t *part,
    track_node *user_target, snapshot_eval_t *out) {
    return snapshot_evaluate_target_plan_internal(ctx, part, user_target, 1,
                                                  out);
}

static pos_route_eval_result_t snapshot_evaluate_target_ready_now(
    const snapshot_ctx_t *ctx, const deadlock_participant_snapshot_t *part,
    track_node *user_target, snapshot_eval_t *out) {
    snapshot_eval_t local_eval;
    snapshot_eval_t *eval = out ? out : &local_eval;
    route_plan_t reserve_plan;
    pos_route_eval_result_t result =
        snapshot_evaluate_target_plan_internal(ctx, part, user_target, 0, eval);

    if (result != POS_ROUTE_EVAL_READY) return result;

    reserve_plan = eval->plan;
    if (reserve_plan.has_reversal) reserve_plan.path_count2 = 0;

    if (!snapshot_can_reserve_plan(ctx, part->train_num, &reserve_plan)) {
        eval->blocker_mask = snapshot_route_blocker_mask_from_plan(
            ctx, part->train_num, &reserve_plan);
        return POS_ROUTE_EVAL_BLOCKED;
    }

    if (snapshot_route_switch_blocker(ctx, eval->plan.sw_nums, eval->plan.sw_dirs,
                                      eval->plan.sw_count, part->train_num) >= 0) {
        eval->blocker_mask = snapshot_route_blocker_mask_from_switches(
            ctx, eval->plan.sw_nums, eval->plan.sw_dirs, eval->plan.sw_count,
            part->train_num);
        return POS_ROUTE_EVAL_BLOCKED;
    }

    eval->blocker_mask = 0;
    return POS_ROUTE_EVAL_READY;
}

static int32_t snapshot_authority_path_dist(const uint16_t *path,
                                            int start_cursor,
                                            int end_cursor) {
    if (!path) return -1;
    if (end_cursor < start_cursor) return 0;
    return route_path_dist_from(path, start_cursor, end_cursor + 1);
}

static int snapshot_authority_build_best_prefix(
    const snapshot_ctx_t *ctx, const deadlock_participant_snapshot_t *part,
    int requester_train, const uint16_t *path, int path_count, int start_cursor,
    int32_t min_window_mm, int32_t stop_dist_mm, int allow_short_goal,
    int min_end_cursor, route_plan_t *out_prefix, int *out_end_cursor,
    int *out_switch_blocker_owner) {
    int have_short_goal = 0;
    int short_goal_end_cursor = -1;

    if (!ctx || !part || !path || !out_prefix || !out_end_cursor) return 0;
    if (path_count <= 0 || start_cursor < 0 || start_cursor >= path_count) return 0;
    if (out_switch_blocker_owner) *out_switch_blocker_owner = -1;

    for (int end_cursor = start_cursor; end_cursor < path_count; end_cursor++) {
        int32_t dist_mm;
        int switch_blocker;

        if (!traffic_window_build_prefix_plan(path, path_count, start_cursor,
                                              end_cursor,
                                              &g_snapshot_authority_candidate_prefix)) {
            break;
        }

        dist_mm = snapshot_authority_path_dist(path, start_cursor, end_cursor);
        if (dist_mm < 0) break;
        if (dist_mm <= 0) continue;
        if (end_cursor != path_count - 1 &&
            g_track[path[end_cursor]].type != NODE_SENSOR) {
            continue;
        }

        switch_blocker = snapshot_route_switch_blocker(
            ctx, g_snapshot_authority_candidate_prefix.sw_nums,
            g_snapshot_authority_candidate_prefix.sw_dirs,
            g_snapshot_authority_candidate_prefix.sw_count, requester_train);
        if (switch_blocker >= 0) {
            if (out_switch_blocker_owner) *out_switch_blocker_owner = switch_blocker;
            break;
        }
        if (!snapshot_can_reserve_plan(ctx, requester_train,
                                       &g_snapshot_authority_candidate_prefix)) {
            break;
        }

        if (allow_short_goal &&
            end_cursor == path_count - 1 &&
            dist_mm >= stop_dist_mm &&
            end_cursor > min_end_cursor) {
            g_snapshot_authority_short_goal_prefix =
                g_snapshot_authority_candidate_prefix;
            short_goal_end_cursor = end_cursor;
            have_short_goal = 1;
        }

        if (dist_mm < min_window_mm) continue;
        if (end_cursor <= min_end_cursor) continue;

        *out_prefix = g_snapshot_authority_candidate_prefix;
        *out_end_cursor = end_cursor;
        return 1;
    }

    if (have_short_goal) {
        *out_prefix = g_snapshot_authority_short_goal_prefix;
        *out_end_cursor = short_goal_end_cursor;
        return 1;
    }

    return 0;
}

static int snapshot_prepare_launch_strict(const snapshot_ctx_t *ctx,
                                          const deadlock_participant_snapshot_t *part,
                                          const route_plan_t *full_plan,
                                          route_plan_t *out_prefix,
                                          int *out_reserved_end_cursor,
                                          int *out_switch_blocker_owner) {
    return snapshot_authority_build_best_prefix(
        ctx, part, part->train_num, full_plan->path_nodes, full_plan->path_count,
        0, snapshot_min_window_mm(part), snapshot_stop_dist_mm(part), 0, -1,
        out_prefix, out_reserved_end_cursor, out_switch_blocker_owner);
}

static int snapshot_local_get_next_dir(const deadlock_snapshot_t *snapshot,
                                       track_node *node) {
    int sw_idx;
    char state;

    if (!snapshot || !node) return -1;
    switch (node->type) {
    case NODE_SENSOR:
    case NODE_MERGE:
    case NODE_ENTER:
        return DIR_AHEAD;
    case NODE_BRANCH:
        sw_idx = track_switch_to_index(node->num);
        if (sw_idx < 0) return -1;
        state = snapshot->switch_state[sw_idx];
        if (state == 'S') return DIR_STRAIGHT;
        if (state == 'C') return DIR_CURVED;
        return -1;
    default:
        return -1;
    }
}

static track_edge *snapshot_local_get_next_edge(const deadlock_snapshot_t *snapshot,
                                                track_node *node) {
    int dir = snapshot_local_get_next_dir(snapshot, node);
    if (!node || dir < 0) return NULL;
    return &node->edge[dir];
}

static int snapshot_local_follow_dist(const deadlock_snapshot_t *snapshot,
                                      track_node *cur,
                                      track_node *to,
                                      int max_hops) {
    int32_t dist = 0;

    if (!snapshot || !cur || !to) return -1;
    if (cur == to) return 0;
    for (int h = 0; h < max_hops; h++) {
        track_edge *edge = snapshot_local_get_next_edge(snapshot, cur);
        if (!edge || !edge->dest) return -1;
        dist += edge->dist;
        cur = edge->dest;
        if (cur == to) return dist;
        if (cur->type == NODE_EXIT) return -1;
    }
    return -1;
}

static track_node *snapshot_predict_next_sensor(const deadlock_snapshot_t *snapshot,
                                                track_node *cur) {
    if (!snapshot || !cur) return NULL;

    for (int hops = 0; hops < 80; hops++) {
        track_edge *edge = snapshot_local_get_next_edge(snapshot, cur);
        if (!edge || !edge->dest) return NULL;
        cur = edge->dest;
        if (cur->type == NODE_SENSOR) return cur;
        if (cur->type == NODE_EXIT) return NULL;
    }
    return NULL;
}

static void snapshot_keep_mark_node(uint8_t keep[TRACK_MAX], track_node *node) {
    int idx;
    int ridx;

    if (!keep || !node) return;
    idx = (int)(node - g_track);
    ridx = snapshot_reverse_index(idx);
    if (idx >= 0 && idx < TRACK_MAX) keep[idx] = 1;
    if (ridx >= 0 && ridx < TRACK_MAX) keep[ridx] = 1;
}

static void snapshot_keep_mark_walk_dist(const deadlock_snapshot_t *snapshot,
                                         uint8_t keep[TRACK_MAX],
                                         track_node *start,
                                         int32_t dist_mm) {
    track_node *cur;
    int32_t dist = 0;

    if (!snapshot || !keep || !start) return;
    cur = start;
    for (int h = 0; h < 200; h++) {
        track_edge *edge;

        snapshot_keep_mark_node(keep, cur);
        if (dist >= dist_mm) break;
        edge = snapshot_local_get_next_edge(snapshot, cur);
        if (!edge || !edge->dest) break;
        dist += edge->dist;
        cur = edge->dest;
    }
}

static void snapshot_keep_mark_walk_to(const deadlock_snapshot_t *snapshot,
                                       uint8_t keep[TRACK_MAX],
                                       track_node *start,
                                       track_node *end) {
    track_node *cur;

    if (!snapshot || !keep || !start || !end) return;
    cur = start;
    for (int h = 0; h < 200; h++) {
        track_edge *edge;

        snapshot_keep_mark_node(keep, cur);
        if (cur == end) break;
        edge = snapshot_local_get_next_edge(snapshot, cur);
        if (!edge || !edge->dest) break;
        cur = edge->dest;
    }
}

static void snapshot_build_keep_body_marks(const deadlock_snapshot_t *snapshot,
                                           track_node *last_hit,
                                           int32_t body_mm,
                                           track_node *next_hit,
                                           uint8_t keep[TRACK_MAX]) {
    int keep_to_next = 0;

    for (int i = 0; i < TRACK_MAX; i++) keep[i] = 0;
    if (last_hit && next_hit) {
        keep_to_next = (last_hit == next_hit) ||
                       (snapshot_local_follow_dist(snapshot, last_hit, next_hit, 120) >= 0);
    }

    if (!last_hit) {
        traffic_expand_zone_marks(keep);
        return;
    }

    snapshot_keep_mark_node(keep, last_hit);
    if (last_hit->reverse) {
        snapshot_keep_mark_walk_dist(snapshot, keep, last_hit->reverse, body_mm);
    }
    if (keep_to_next) {
        if (last_hit != next_hit) {
            snapshot_keep_mark_walk_to(snapshot, keep, last_hit, next_hit);
        }
        snapshot_keep_mark_walk_dist(snapshot, keep, next_hit, body_mm);
    }
    traffic_expand_zone_marks(keep);
}

static uint8_t snapshot_simulate_deadlock_unblocked_mask(
    const snapshot_ctx_t *ctx, const deadlock_participant_snapshot_t *victim,
    uint8_t cycle_mask, track_node *yield_target) {
    int owners[TRACK_MAX];
    snapshot_ctx_t sim_ctx;
    uint8_t ready_mask = 0;
    uint8_t victim_bit;
    uint8_t keep[TRACK_MAX];
    track_node *keep_end;

    if (!ctx || !victim || !yield_target) return 0;

    victim_bit = snapshot_train_bit(victim->train_num);
    for (int i = 0; i < TRACK_MAX; i++) owners[i] = ctx->owners[i];

    keep_end = snapshot_predict_next_sensor(ctx->snapshot, yield_target);
    snapshot_build_keep_body_marks(ctx->snapshot, yield_target, TRAIN_BODY_MM,
                                   keep_end, keep);

    for (int i = 0; i < TRACK_MAX; i++) {
        if (owners[i] == victim->train_num) owners[i] = -1;
    }
    for (int i = 0; i < TRACK_MAX; i++) {
        if (keep[i]) owners[i] = victim->train_num;
    }

    sim_ctx = *ctx;
    sim_ctx.owners = owners;

    for (int i = 0; i < ctx->snapshot->participant_count; i++) {
        const deadlock_participant_snapshot_t *other = &ctx->snapshot->participants[i];
        uint8_t bit;
        track_node *target;

        if (!other || other->train_num < 0) continue;
        bit = snapshot_train_bit(other->train_num);
        if (!(cycle_mask & bit)) continue;
        if (other->train_num == victim->train_num) continue;
        if (other->route_state != TRAIN_STATE_WAIT_RESOURCE) continue;
        if (!(other->blocker_mask & victim_bit)) continue;

        target = snapshot_node_from_index(snapshot_goal_idx(other));
        if (!target) continue;

        if (snapshot_evaluate_target_ready_now(&sim_ctx, other, target, NULL) ==
            POS_ROUTE_EVAL_READY) {
            ready_mask |= bit;
        }
    }

    return ready_mask;
}

static int32_t snapshot_candidate_sort_dist(track_node *origins[2],
                                            track_node *cand) {
    int32_t best = -1;

    if (!cand) return -1;
    for (int i = 0; i < 2; i++) {
        int32_t dist;
        if (!origins[i]) continue;
        dist = route_direct_sensor_dist(origins[i], cand);
        if (dist < 0) continue;
        if (best < 0 || dist < best) best = dist;
    }
    return best;
}

static int snapshot_merge_sorted_candidates(track_node *origins[2]) {
    int count0 = 0;
    int count1 = 0;
    int i0 = 0;
    int i1 = 0;
    int out_count = 0;

    for (int i = 0; i < TRACK_MAX; i++) g_snapshot_seen_candidate[i] = 0;

    if (origins[0]) {
        count0 = route_fill_sorted_direct_sensor_candidates(origins[0],
                                                            g_snapshot_sorted_from_origin0,
                                                            TRACK_MAX);
    }
    if (origins[1]) {
        count1 = route_fill_sorted_direct_sensor_candidates(origins[1],
                                                            g_snapshot_sorted_from_origin1,
                                                            TRACK_MAX);
    }

    while (i0 < count0 || i1 < count1) {
        track_node *cand0 = NULL;
        track_node *cand1 = NULL;
        int32_t dist0 = -1;
        int32_t dist1 = -1;
        track_node *chosen;
        int idx;

        if (i0 < count0) {
            cand0 = &g_track[g_snapshot_sorted_from_origin0[i0]];
            dist0 = snapshot_candidate_sort_dist(origins, cand0);
        }
        if (i1 < count1) {
            cand1 = &g_track[g_snapshot_sorted_from_origin1[i1]];
            dist1 = snapshot_candidate_sort_dist(origins, cand1);
        }

        if (!cand1 ||
            (cand0 != NULL &&
             (dist1 < 0 || dist0 < dist1 ||
              (dist0 == dist1 &&
               g_snapshot_sorted_from_origin0[i0] <
                   g_snapshot_sorted_from_origin1[i1])))) {
            chosen = cand0;
            i0++;
        } else {
            chosen = cand1;
            i1++;
        }

        if (!chosen) continue;
        idx = (int)(chosen - g_track);
        if (idx < 0 || idx >= TRACK_MAX) continue;
        if (g_snapshot_seen_candidate[idx]) continue;

        g_snapshot_seen_candidate[idx] = 1;
        g_snapshot_merged_candidates[out_count++] = (uint16_t)idx;
    }

    return out_count;
}

static int snapshot_candidate_can_force_move(
    const snapshot_ctx_t *ctx, const deadlock_participant_snapshot_t *part,
    track_node *target, snapshot_eval_t *out_eval) {
    snapshot_eval_t eval;
    int reserved_end_cursor = -1;
    int switch_blocker_owner = -1;

    if (!ctx || !part || !target) return 0;

    if (snapshot_evaluate_target_plan(ctx, part, target, &eval) !=
        POS_ROUTE_EVAL_READY) {
        return 0;
    }
    if (!snapshot_prepare_launch_strict(ctx, part, &eval.plan,
                                        &g_snapshot_launch_prefix,
                                        &reserved_end_cursor,
                                        &switch_blocker_owner)) {
        return 0;
    }
    if (out_eval) *out_eval = eval;
    return 1;
}

static int snapshot_pick_yield_target(const snapshot_ctx_t *ctx,
                                      const deadlock_participant_snapshot_t *part,
                                      uint8_t cycle_mask,
                                      int *out_target_idx,
                                      uint8_t *out_unblocked_mask,
                                      snapshot_eval_t *out_eval) {
    track_node *origins[2];
    track_node *current_target;
    track_node *best_unlock_target = NULL;
    track_node *best_ready_reloc_target = NULL;
    snapshot_eval_t best_unlock_eval = {0};
    snapshot_eval_t best_ready_reloc_eval = {0};
    uint8_t best_unblocked_mask = 0;
    uint8_t fallback_wait_mask;
    int merged_count;
    int32_t min_dist_mm;

    if (out_target_idx) *out_target_idx = -1;
    if (out_unblocked_mask) *out_unblocked_mask = 0;
    if (out_eval) *out_eval = (snapshot_eval_t){0};
    if (!ctx || !part || part->cur_sensor_idx < 0) return 0;

    snapshot_fill_origins(part, origins);
    if (!origins[0] && !origins[1]) return 0;

    current_target = snapshot_node_from_index(snapshot_goal_idx(part));
    fallback_wait_mask = cycle_mask & (uint8_t)~snapshot_train_bit(part->train_num);
    min_dist_mm = snapshot_min_window_mm(part);
    merged_count = snapshot_merge_sorted_candidates(origins);

    for (int i = 0; i < merged_count; i++) {
        track_node *cand = &g_track[g_snapshot_merged_candidates[i]];
        snapshot_eval_t eval;
        uint8_t unblocked_mask;
        int32_t sort_dist;

        sort_dist = snapshot_candidate_sort_dist(origins, cand);
        if (sort_dist < 0 || sort_dist < min_dist_mm) continue;
        if (snapshot_same_physical_sensor_idx((int)(cand - g_track),
                                              part->cur_sensor_idx)) {
            continue;
        }
        if (current_target &&
            snapshot_same_physical_sensor_idx((int)(cand - g_track),
                                              (int)(current_target - g_track))) {
            continue;
        }

        if (snapshot_evaluate_target_ready_now(ctx, part, cand, &eval) !=
            POS_ROUTE_EVAL_READY) {
            continue;
        }

        best_ready_reloc_target = cand;
        best_ready_reloc_eval = eval;
        unblocked_mask = snapshot_simulate_deadlock_unblocked_mask(ctx, part,
                                                                   cycle_mask, cand);
        if (unblocked_mask == 0) continue;

        best_unlock_target = cand;
        best_unlock_eval = eval;
        best_unblocked_mask = unblocked_mask;
        break;
    }

    if (best_unlock_target) {
        if (out_target_idx) *out_target_idx = (int)(best_unlock_target - g_track);
        if (out_unblocked_mask) *out_unblocked_mask = best_unblocked_mask;
        if (out_eval) *out_eval = best_unlock_eval;
        return 1;
    }

    if (best_ready_reloc_target) {
        if (out_target_idx) *out_target_idx = (int)(best_ready_reloc_target - g_track);
        if (out_unblocked_mask) *out_unblocked_mask = fallback_wait_mask;
        if (out_eval) *out_eval = best_ready_reloc_eval;
        return 1;
    }

    for (int i = merged_count - 1; i >= 0; i--) {
        track_node *cand = &g_track[g_snapshot_merged_candidates[i]];
        snapshot_eval_t eval;
        int32_t sort_dist = snapshot_candidate_sort_dist(origins, cand);

        if (sort_dist < 0 || sort_dist < min_dist_mm) continue;
        if (snapshot_same_physical_sensor_idx((int)(cand - g_track),
                                              part->cur_sensor_idx)) {
            continue;
        }
        if (current_target &&
            snapshot_same_physical_sensor_idx((int)(cand - g_track),
                                              (int)(current_target - g_track))) {
            continue;
        }
        if (!snapshot_candidate_can_force_move(ctx, part, cand, &eval)) continue;

        if (out_target_idx) *out_target_idx = (int)(cand - g_track);
        if (out_unblocked_mask) *out_unblocked_mask = fallback_wait_mask;
        if (out_eval) *out_eval = eval;
        return 1;
    }

    return 0;
}

static void snapshot_fill_cycle_trains(deadlock_result_t *out,
                                       const snapshot_participants_t *parts,
                                       uint8_t cycle_mask) {
    if (!out) return;

    out->cycle_count = 0;
    for (int i = 0; i < DEADLOCK_MAX_TRAINS; i++) {
        out->cycle_trains[i] = -1;
    }
    if (!parts) return;

    for (int i = 0; i < parts->count; i++) {
        if (!(cycle_mask & (uint8_t)(1u << i))) continue;
        if (out->cycle_count >= DEADLOCK_MAX_TRAINS) break;
        out->cycle_trains[out->cycle_count++] = parts->train_nums[i];
    }
}

static void snapshot_copy_participants(deadlock_result_t *out,
                                       const deadlock_snapshot_t *snapshot) {
    if (!out || !snapshot) return;

    out->snapshot_now_us = snapshot->now_us;
    out->traffic_generation = snapshot->traffic_generation;
    out->switch_generation = snapshot->switch_generation;
    out->auto_dispatching_targets = snapshot->auto_dispatching_targets;
    out->participant_count = snapshot->participant_count;
    for (int i = 0; i < snapshot->participant_count && i < DEADLOCK_MAX_TRAINS; i++) {
        out->participants[i] = snapshot->participants[i];
    }
}

static int snapshot_plan_reroute(const snapshot_ctx_t *ctx,
                                 const snapshot_participants_t *parts,
                                 const deadlock_participant_snapshot_t *victim,
                                 uint8_t cycle_mask,
                                 int resume_after_yield,
                                 deadlock_result_t *out) {
    uint8_t global_cycle_mask;
    uint8_t unblocked_mask = 0;
    snapshot_eval_t eval;
    int yield_target_idx = -1;
    int blocked_target_idx;

    if (!ctx || !parts || !victim || !out) return 0;

    blocked_target_idx = snapshot_goal_idx(victim);
    if (blocked_target_idx < 0) return 0;

    global_cycle_mask = snapshot_global_mask_from_local(parts, cycle_mask);
    if (!snapshot_pick_yield_target(ctx, victim, global_cycle_mask,
                                    &yield_target_idx, &unblocked_mask,
                                    &eval) ||
        yield_target_idx < 0 || !eval.chosen_origin) {
        return 0;
    }

    out->action = DEADLOCK_RESULT_REROUTE;
    out->victim_train = victim->train_num;
    out->blocked_target_idx = blocked_target_idx;
    out->yield_target_idx = yield_target_idx;
    out->resume_target_idx = -1;
    out->resume_offset_mm = 0;
    out->wait_start_mask = resume_after_yield ? unblocked_mask : 0;
    out->chosen_origin_idx = (int)(eval.chosen_origin - g_track);
    out->need_initial_reverse = eval.need_initial_reverse;
    snapshot_serialize_route_plan(&out->route_plan, &eval.plan);

    if (resume_after_yield) {
        if (victim->resume_target_idx >= 0) {
            out->resume_target_idx = victim->resume_target_idx;
            out->resume_offset_mm = victim->resume_offset_mm;
        } else {
            out->resume_target_idx = blocked_target_idx;
            out->resume_offset_mm = victim->goal_offset_mm;
        }
    }

    snapshot_fill_cycle_trains(out, parts, cycle_mask);
    return 1;
}

static int snapshot_plan_notice_only(const deadlock_snapshot_t *snapshot,
                                     const snapshot_participants_t *parts,
                                     uint8_t cycle_mask,
                                     int victim_train,
                                     deadlock_result_t *out) {
    (void)snapshot;
    if (!snapshot || !parts || !out || victim_train < 0) return 0;

    out->action = DEADLOCK_RESULT_NOTICE_ONLY;
    out->victim_train = victim_train;
    snapshot_fill_cycle_trains(out, parts, cycle_mask);
    return 1;
}

int deadlock_plan_from_snapshot(const deadlock_snapshot_t *snapshot,
                                deadlock_result_t *out) {
    snapshot_ctx_t ctx;
    snapshot_participants_t parts;
    uint8_t cycle_mask = 0;
    snapshot_deadlock_kind_t kind = SNAPSHOT_DEADLOCK_KIND_NONE;
    int victim_train = -1;

    snapshot_deadlock_result_clear(out);
    if (!snapshot || !out) return 0;

    snapshot_copy_participants(out, snapshot);
    snapshot_collect_participants(snapshot, &parts);
    if (parts.count < 2) return 0;

    ctx.snapshot = snapshot;
    ctx.owners = snapshot->reservation_owner;

    for (int i = 0; i < parts.count; i++) {
        if (!(parts.wait_mask & (uint8_t)(1u << i))) continue;
        cycle_mask = snapshot_find_mask_for_train(parts.train_nums[i], &parts,
                                                  &kind);
        if (cycle_mask != 0) break;
    }
    if (cycle_mask == 0) return 0;

    victim_train = snapshot_choose_victim(&parts, cycle_mask, kind);
    if (victim_train < 0) return 0;

    if (kind == SNAPSHOT_DEADLOCK_KIND_STOPPED_BLOCKER) {
        for (int i = 0; i < parts.count; i++) {
            const deadlock_participant_snapshot_t *victim;
            int keep_resume;

            if (!(cycle_mask & (uint8_t)(1u << i))) continue;
            victim = parts.participants[i];
            if (!victim || victim->route_state != TRAIN_STATE_STOPPED) continue;

            keep_resume = victim->resume_target_idx >= 0;
            if (snapshot_plan_reroute(&ctx, &parts, victim, cycle_mask,
                                      keep_resume, out)) {
                return 1;
            }
        }
        snapshot_plan_notice_only(snapshot, &parts, cycle_mask, victim_train, out);
        return (out->action != DEADLOCK_RESULT_NONE);
    }

    for (int i = 0; i < parts.count; i++) {
        const deadlock_participant_snapshot_t *victim = parts.participants[i];
        if (!victim || victim->train_num != victim_train) continue;
        if (snapshot_plan_reroute(&ctx, &parts, victim, cycle_mask, 1, out)) {
            return 1;
        }
        break;
    }

    snapshot_plan_notice_only(snapshot, &parts, cycle_mask, victim_train, out);
    return (out->action != DEADLOCK_RESULT_NONE);
}
