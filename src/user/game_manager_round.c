#include "game_manager_internal.h"
#include "train_tracking/pos_route_internal.h"
#include "train_tracking/traffic_manager.h"

#define GAME_SENSOR_BIT_WORDS ((GAME_SENSOR_KEYS + 31) / 32)
#define GAME_TRACK_BIT_WORDS ((TRACK_MAX + 31) / 32)
#define GAME_AI_EVAL_CACHE_SIZE 256

typedef struct {
    pos_target_query_status_t status;
    int32_t total_dist_mm;
    uint8_t blocker_mask;
    uint32_t sensor_bits[GAME_SENSOR_BIT_WORDS];
    uint32_t node_bits[GAME_TRACK_BIT_WORDS];
} game_ai_target_eval_t;

typedef struct {
    uint8_t valid;
    int train_num;
    int target_idx;
    int cur_sensor_idx;
    int origin_idx[2];
    int goto_speed;
    /* Reservation/switch snapshot hash: reuse only when route inputs match. */
    uint64_t traffic_hash;
    game_ai_target_eval_t eval;
} game_ai_eval_cache_entry_t;

static game_ai_eval_cache_entry_t g_game_ai_eval_cache[GAME_AI_EVAL_CACHE_SIZE];
static int g_game_ai_eval_cache_next = 0;

static int game_train_bit(int train_num) {
    switch (train_num) {
    case 13: return 1 << 0;
    case 14: return 1 << 1;
    case 15: return 1 << 2;
    case 17: return 1 << 3;
    case 18: return 1 << 4;
    case 55: return 1 << 5;
    default: return 0;
    }
}

static int game_node_index(track_node *node) {
    if (!node) return -1;
    return (int)(node - g_track);
}

static void game_bits_clear(uint32_t *bits, int words) {
    if (!bits || words <= 0) return;
    for (int i = 0; i < words; i++) bits[i] = 0;
}

static void game_bits_set(uint32_t *bits, int words, int idx) {
    int bit_count = words * 32;

    if (!bits || words <= 0) return;
    if (idx < 0 || idx >= bit_count) return;
    bits[idx / 32] |= (uint32_t)1u << (idx % 32);
}

static int game_bits_test(const uint32_t *bits, int words, int idx) {
    int bit_count = words * 32;

    if (!bits || words <= 0) return 0;
    if (idx < 0 || idx >= bit_count) return 0;
    return (bits[idx / 32] & ((uint32_t)1u << (idx % 32))) != 0;
}

static int game_bits_overlap_count(const uint32_t *lhs, const uint32_t *rhs, int words) {
    int count = 0;

    if (!lhs || !rhs || words <= 0) return 0;

    for (int i = 0; i < words; i++) {
        uint32_t overlap = lhs[i] & rhs[i];
        while (overlap) {
            count += (int)(overlap & 1u);
            overlap >>= 1;
        }
    }

    return count;
}

static void game_sensor_bits_clear(uint32_t bits[GAME_SENSOR_BIT_WORDS]) {
    game_bits_clear(bits, GAME_SENSOR_BIT_WORDS);
}

static void game_sensor_bits_set(uint32_t bits[GAME_SENSOR_BIT_WORDS], int sensor_num) {
    game_bits_set(bits, GAME_SENSOR_BIT_WORDS, sensor_num);
}

static int game_sensor_bits_test(const uint32_t bits[GAME_SENSOR_BIT_WORDS], int sensor_num) {
    return game_bits_test(bits, GAME_SENSOR_BIT_WORDS, sensor_num);
}

static void game_track_bits_clear(uint32_t bits[GAME_TRACK_BIT_WORDS]) {
    game_bits_clear(bits, GAME_TRACK_BIT_WORDS);
}

static void game_track_bits_set(uint32_t bits[GAME_TRACK_BIT_WORDS], int node_idx) {
    game_bits_set(bits, GAME_TRACK_BIT_WORDS, node_idx);
}

static void game_ai_target_eval_reset(game_ai_target_eval_t *eval) {
    if (!eval) return;
    eval->status = POS_TARGET_UNREACHABLE;
    eval->total_dist_mm = 0;
    eval->blocker_mask = 0;
    game_sensor_bits_clear(eval->sensor_bits);
    game_track_bits_clear(eval->node_bits);
}

static uint64_t game_hash_u32(uint64_t hash, uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8) {
        hash ^= (uint64_t)((value >> shift) & 0xffu);
        hash *= 1099511628211ull;
    }
    return hash;
}

static uint64_t game_ai_eval_traffic_hash(void) {
    int owners[TRACK_MAX];
    const switch_entry_t *switches = track_get_switch_state();
    uint64_t hash = 1469598103934665603ull;

    traffic_snapshot_reservations(owners);
    for (int i = 0; i < TRACK_MAX; i++) {
        hash = game_hash_u32(hash, (uint32_t)(owners[i] + 2));
    }
    for (int i = 0; i < MAX_SWITCHES; i++) {
        hash = game_hash_u32(hash, (uint32_t)(uint8_t)switches[i].state);
    }

    return hash;
}

static void game_build_route_sensor_bits(const pos_target_query_t *query,
                                         track_node *requested_target,
                                         uint32_t out_bits[GAME_SENSOR_BIT_WORDS]) {
    uint16_t exclude_num = requested_target ? (uint16_t)(requested_target->num + 1) : 0;
    uint16_t exclude_chosen = (query && query->plan.chosen_target)
                              ? (uint16_t)(query->plan.chosen_target->num + 1)
                              : 0;

    game_sensor_bits_clear(out_bits);
    if (!query) return;

    for (int pass = 0; pass < 2; pass++) {
        const uint16_t *path = (pass == 0) ? query->plan.path_nodes : query->plan.path_nodes2;
        int path_count = (pass == 0) ? query->plan.path_count : query->plan.path_count2;

        for (int i = 0; i < path_count; i++) {
            int idx = (int)path[i];
            int sensor_num;

            if (idx < 0 || idx >= TRACK_MAX) continue;
            if (g_track[idx].type != NODE_SENSOR) continue;
            sensor_num = g_track[idx].num;
            if (sensor_num < 0 || sensor_num >= GAME_SENSOR_KEYS) continue;
            if ((uint16_t)(sensor_num + 1) == exclude_num ||
                (uint16_t)(sensor_num + 1) == exclude_chosen) {
                continue;
            }
            game_sensor_bits_set(out_bits, sensor_num);
        }
    }
}

static void game_build_route_node_bits(const pos_target_query_t *query,
                                       uint32_t out_bits[GAME_TRACK_BIT_WORDS]) {
    game_track_bits_clear(out_bits);
    if (!query) return;

    for (int pass = 0; pass < 2; pass++) {
        const uint16_t *path = (pass == 0) ? query->plan.path_nodes : query->plan.path_nodes2;
        int path_count = (pass == 0) ? query->plan.path_count : query->plan.path_count2;

        for (int i = 0; i < path_count; i++) {
            int idx = (int)path[i];
            if (idx < 0 || idx >= TRACK_MAX) continue;
            game_track_bits_set(out_bits, idx);
        }
    }

    for (int i = 0; i < query->plan.extra_reserve_count; i++) {
        int idx = (int)query->plan.extra_reserve_nodes[i];
        if (idx < 0 || idx >= TRACK_MAX) continue;
        game_track_bits_set(out_bits, idx);
    }
}

static int game_collect_route_value_from_bits(game_context_t *ctx,
                                              const uint32_t bits[GAME_SENSOR_BIT_WORDS],
                                              game_role_t role) {
    uint8_t role_bit = (role == GAME_ROLE_HUMAN) ? 1u : 2u;
    uint8_t other_bit = (role == GAME_ROLE_HUMAN) ? 2u : 1u;
    int score = 0;

    if (!ctx || !bits) return 0;

    for (int sensor_num = 0; sensor_num < GAME_SENSOR_KEYS; sensor_num++) {
        if (!game_sensor_bits_test(bits, sensor_num)) continue;
        if (ctx->claim_mask[sensor_num] & role_bit) continue;
        score += (ctx->claim_mask[sensor_num] & other_bit) ? 2 : 3;
    }

    return score;
}

static int game_lookup_ai_target_eval(train_pos_t *pos, track_node *target,
                                      track_node *origins[2],
                                      uint64_t traffic_hash,
                                      game_ai_target_eval_t *out) {
    int target_idx;
    int cur_sensor_idx;
    int origin0_idx;
    int origin1_idx;

    if (out) game_ai_target_eval_reset(out);
    if (!pos || !target || !out) return 0;

    target_idx = game_node_index(target);
    cur_sensor_idx = game_node_index(pos->cur_sensor);
    origin0_idx = game_node_index((track_node *)origins[0]);
    origin1_idx = game_node_index((track_node *)origins[1]);

    for (int i = 0; i < GAME_AI_EVAL_CACHE_SIZE; i++) {
        const game_ai_eval_cache_entry_t *entry = &g_game_ai_eval_cache[i];

        if (!entry->valid) continue;
        if (entry->train_num != pos->train_num) continue;
        if (entry->target_idx != target_idx) continue;
        if (entry->cur_sensor_idx != cur_sensor_idx) continue;
        if (entry->origin_idx[0] != origin0_idx || entry->origin_idx[1] != origin1_idx) continue;
        if (entry->goto_speed != pos->goto_speed) continue;
        if (entry->traffic_hash != traffic_hash) continue;

        *out = entry->eval;
        return 1;
    }

    return 0;
}

static void game_store_ai_target_eval(train_pos_t *pos, track_node *target,
                                      track_node *origins[2],
                                      uint64_t traffic_hash,
                                      const game_ai_target_eval_t *eval) {
    game_ai_eval_cache_entry_t *entry;

    if (!pos || !target || !eval) return;

    entry = &g_game_ai_eval_cache[g_game_ai_eval_cache_next];
    g_game_ai_eval_cache_next++;
    if (g_game_ai_eval_cache_next >= GAME_AI_EVAL_CACHE_SIZE) {
        g_game_ai_eval_cache_next = 0;
    }

    entry->valid = 1;
    entry->train_num = pos->train_num;
    entry->target_idx = game_node_index(target);
    entry->cur_sensor_idx = game_node_index(pos->cur_sensor);
    entry->origin_idx[0] = game_node_index((track_node *)origins[0]);
    entry->origin_idx[1] = game_node_index((track_node *)origins[1]);
    entry->goto_speed = pos->goto_speed;
    entry->traffic_hash = traffic_hash;
    entry->eval = *eval;
}

static void game_get_ai_target_eval(train_pos_t *pos, track_node *target,
                                    track_node *origins[2],
                                    uint64_t traffic_hash,
                                    game_ai_target_eval_t *out) {
    pos_target_query_t query;

    game_ai_target_eval_reset(out);
    if (!pos || !target || !out) return;

    if (game_lookup_ai_target_eval(pos, target, origins, traffic_hash, out)) return;

    out->status = pos_query_target(pos->train_num, target, &query);
    if (out->status != POS_TARGET_UNREACHABLE) {
        out->total_dist_mm = query.plan.total_dist_mm;
        out->blocker_mask = query.blocker_mask;
        game_build_route_sensor_bits(&query, target, out->sensor_bits);
        game_build_route_node_bits(&query, out->node_bits);
    }

    game_store_ai_target_eval(pos, target, origins, traffic_hash, out);
}

static int game_role_index_from_train(game_context_t *ctx, int train_num) {
    for (int i = 0; i < GAME_ROLES; i++) {
        if (ctx->slots[i].train_num == train_num) return i;
    }
    return -1;
}

static int game_current_sensor_num(int train_num) {
    train_pos_t *pos = pos_get(train_num);
    if (!pos || !pos->cur_sensor) return -1;
    return pos->cur_sensor->num;
}

static int game_is_sensor_currently_occupied(game_context_t *ctx, int sensor_num) {
    for (int i = 0; i < GAME_ROLES; i++) {
        if (ctx->slots[i].train_num < 0) continue;
        if (game_current_sensor_num(ctx->slots[i].train_num) == sensor_num) return 1;
    }
    return 0;
}

static int game_sensor_matches_target(uint16_t sensor_num, track_node *target) {
    if (sensor_num == 0 || !target) return 0;
    if (target->type == NODE_SENSOR &&
        sensor_num == (uint16_t)(target->num + 1)) {
        return 1;
    }
    if (target->reverse &&
        target->reverse->type == NODE_SENSOR &&
        sensor_num == (uint16_t)(target->reverse->num + 1)) {
        return 1;
    }
    return 0;
}

static int game_targets_same_sensor(track_node *a, track_node *b) {
    if (!a || !b) return 0;
    return a == b || a->reverse == b || b->reverse == a;
}

static track_node *game_pos_final_target_node(const train_pos_t *pos) {
    if (!pos) return NULL;
    if (pos->orig_user_target) return pos->orig_user_target;
    if (pos->midrev.active && pos->midrev.final_target) return pos->midrev.final_target;
    return pos->target_sensor;
}

static int game_pos_is_stopped_at_target(const train_pos_t *pos, track_node *target) {
    track_node *final_target;

    if (!pos || !target) return 0;
    if (pos->route_state != TRAIN_STATE_STOPPED) return 0;

    final_target = game_pos_final_target_node(pos);
    if (pos->parked_target_col == POS_TARGET_COL_FINAL &&
        game_targets_same_sensor(final_target, target)) {
        return 1;
    }

    return 0;
}

void game_sync_slot_completion_from_position(game_context_t *ctx, game_role_t role) {
    game_role_slot_t *slot;
    train_pos_t *pos;
    int changed = 0;

    if (role < 0 || role >= GAME_ROLES) return;

    slot = &ctx->slots[role];
    if (slot->train_num < 0) return;

    pos = pos_get(slot->train_num);
    if (!pos) return;

    if (!slot->completed && game_pos_is_stopped_at_target(pos, slot->target)) {
        slot->completed = 1;
        changed = 1;
    }

    if (role == GAME_ROLE_NEUTRAL &&
        !slot->standby_completed &&
        slot->standby_target != NULL &&
        game_pos_is_stopped_at_target(pos, slot->standby_target)) {
        slot->standby_completed = 1;
        changed = 1;
    }

    if (changed) ui_mark_position_dirty();
}

static void game_log_neutral_detour_if_needed(game_context_t *ctx) {
    game_role_slot_t *neutral = &ctx->slots[GAME_ROLE_NEUTRAL];
    train_pos_t *pos;
    track_node *detour = NULL;
    char buf[128];
    char *p = buf;
    char *end = buf + sizeof(buf) - 1;

    if (neutral->train_num < 0 || neutral->target == NULL) {
        neutral->reported_detour_target = NULL;
        return;
    }

    pos = pos_get(neutral->train_num);
    if (!pos) {
        neutral->reported_detour_target = NULL;
        return;
    }

    if (pos->deadlock_recover.valid &&
        pos->deadlock_recover.yield_target != NULL &&
        !game_targets_same_sensor(pos->deadlock_recover.yield_target,
                                  neutral->target)) {
        detour = pos->deadlock_recover.yield_target;
    }

    if (!detour) {
        neutral->reported_detour_target = NULL;
        return;
    }

    if (neutral->reported_detour_target == detour) return;
    neutral->reported_detour_target = detour;

    p = buf_append_cap(p, end, "game: neutral rerouted ");
    p = buf_append_cap(p, end,
                       neutral->target && neutral->target->name ? neutral->target->name : "-");
    p = buf_append_cap(p, end, " -> ");
    p = buf_append_cap(p, end, detour->name ? detour->name : "-");
    p = buf_append_cap(p, end, " (deadlock yield)");
    *p = '\0';
    game_log_line(buf);
}

static int game_train_has_min_trip_candidate(game_context_t *ctx, int train_num) {
    pos_target_query_t *query = &ctx->game_query_secondary;

    for (int i = 0; i < ctx->sensor_pool_count; i++) {
        track_node *cand = ctx->sensor_pool[i];

        if (!cand) continue;
        if (game_current_sensor_num(train_num) == cand->num) continue;
        if (pos_query_target(train_num, cand, query) == POS_TARGET_UNREACHABLE) continue;
        if (query->plan.total_dist_mm >= GAME_MIN_TRIP_MM) return 1;
    }
    return 0;
}

static game_role_t game_current_priority_role(game_context_t *ctx) {
    if ((ctx->round_index & 1) == 0) return ctx->round1_priority;
    return (ctx->round1_priority == GAME_ROLE_HUMAN) ? GAME_ROLE_AI : GAME_ROLE_HUMAN;
}

static int game_bit_count(uint8_t mask) {
    int count = 0;
    while (mask) {
        count += (mask & 1u);
        mask >>= 1;
    }
    return count;
}

void game_reset_round_state(game_context_t *ctx) {
    for (int i = 0; i < GAME_ROLES; i++) {
        ctx->slots[i].target = NULL;
        ctx->slots[i].target_sensor_num = 0;
        ctx->slots[i].dispatched = 0;
        ctx->slots[i].completed = 0;
        ctx->slots[i].score_delta_half = 0;
        ctx->slots[i].standby_target = NULL;
        ctx->slots[i].standby_sensor_num = 0;
        ctx->slots[i].standby_dispatched = 0;
        ctx->slots[i].standby_completed = 0;
        ctx->slots[i].reported_detour_target = NULL;
    }
}

static track_node *game_pick_best_ai_target(game_context_t *ctx) {
    game_role_slot_t *slot = &ctx->slots[GAME_ROLE_AI];
    train_pos_t *pos = pos_get(slot->train_num);
    track_node *origins[2];
    track_node *best = NULL;
    game_ai_target_eval_t eval;
    uint64_t traffic_hash;
    int best_score = -0x7fffffff;
    int best_dist = 0x7fffffff;

    if (!pos || !pos->cur_sensor) return NULL;

    pos_route_fill_origins(pos, origins);
    traffic_hash = game_ai_eval_traffic_hash();

    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < ctx->sensor_pool_count; i++) {
            track_node *cand = ctx->sensor_pool[i];
            int score;
            int dist;

            if (!cand) continue;
            if (game_current_sensor_num(slot->train_num) == cand->num) continue;
            if (pass == 0 && cand->name == NULL) continue;
            game_get_ai_target_eval(pos, cand, origins, traffic_hash, &eval);
            if (eval.status == POS_TARGET_UNREACHABLE) {
                continue;
            }
            dist = eval.total_dist_mm;
            if (pass == 0 && dist < GAME_MIN_TRIP_MM) continue;

            score = game_collect_route_value_from_bits(ctx, eval.sensor_bits, GAME_ROLE_AI);
            score -= dist / 400;
            if (eval.status == POS_TARGET_BLOCKED) {
                score -= 2 + game_bit_count(eval.blocker_mask);
            }

            if (!best ||
                score > best_score ||
                (score == best_score && dist < best_dist) ||
                (score == best_score && dist == best_dist && (game_rand_u32(ctx) & 1u))) {
                best = cand;
                best_score = score;
                best_dist = dist;
            }
        }
        if (best) return best;
    }

    return NULL;
}

static track_node *game_draw_neutral_target(game_context_t *ctx) {
    game_role_slot_t *slot = &ctx->slots[GAME_ROLE_NEUTRAL];
    game_role_slot_t *ai_slot = &ctx->slots[GAME_ROLE_AI];
    pos_target_query_t *query = &ctx->game_query_primary;
    train_pos_t *ai_pos = pos_get(ai_slot->train_num);
    track_node *ai_origins[2];
    game_ai_target_eval_t ai_eval;
    uint32_t cand_sensor_bits[GAME_SENSOR_BIT_WORDS];
    uint32_t cand_node_bits[GAME_TRACK_BIT_WORDS];
    uint64_t traffic_hash = 0;
    int have_ai_eval = 0;

    game_ai_target_eval_reset(&ai_eval);
    if (ai_pos && ai_pos->cur_sensor && ai_slot->target) {
        pos_route_fill_origins(ai_pos, ai_origins);
        traffic_hash = game_ai_eval_traffic_hash();
        game_get_ai_target_eval(ai_pos, ai_slot->target, ai_origins, traffic_hash, &ai_eval);
        have_ai_eval = (ai_eval.status != POS_TARGET_UNREACHABLE);
    }

    for (int ready_only = 1; ready_only >= 0; ready_only--) {
        for (int pass = 0; pass < 2; pass++) {
            track_node *best = NULL;
            int best_score = -0x7fffffff;
            int best_dist = 0x7fffffff;
            int best_node_overlap = 0x7fffffff;
            int best_sensor_overlap = 0x7fffffff;

            for (int i = 0; i < ctx->sensor_pool_count; i++) {
                track_node *cand = ctx->sensor_pool[i];
                pos_target_query_status_t status;
                int score;
                int dist;
                int node_overlap = 0;
                int sensor_overlap = 0;

                if (!cand) continue;
                if (cand->num < 0 || cand->num >= GAME_SENSOR_KEYS) continue;
                if (ctx->neutral_used[cand->num]) continue;
                if (game_is_sensor_currently_occupied(ctx, cand->num)) continue;

                status = pos_query_target(slot->train_num, cand, query);
                if (status == POS_TARGET_UNREACHABLE) continue;
                if (ready_only && status != POS_TARGET_READY) continue;

                dist = query->plan.total_dist_mm;
                if (pass == 0 && dist < GAME_MIN_TRIP_MM) continue;

                game_build_route_sensor_bits(query, cand, cand_sensor_bits);
                game_build_route_node_bits(query, cand_node_bits);

                if (have_ai_eval) {
                    node_overlap = game_bits_overlap_count(cand_node_bits, ai_eval.node_bits,
                                                           GAME_TRACK_BIT_WORDS);
                    sensor_overlap = game_bits_overlap_count(cand_sensor_bits, ai_eval.sensor_bits,
                                                             GAME_SENSOR_BIT_WORDS);
                }

                score = 0;
                if (ai_slot->target && game_targets_same_sensor(cand, ai_slot->target)) {
                    score -= 1000;
                }
                score -= node_overlap * 12;
                score -= sensor_overlap * 48;
                score -= (dist / 40);

                if (query->blocker_mask) {
                    score -= 150 * game_bit_count(query->blocker_mask);
                    if (query->blocker_mask & game_train_bit(ai_slot->train_num)) {
                        score -= 300;
                    }
                    if (query->blocker_mask &
                        game_train_bit(ctx->slots[GAME_ROLE_HUMAN].train_num)) {
                        score -= 180;
                    }
                }

                if (!best ||
                    score > best_score ||
                    (score == best_score && node_overlap < best_node_overlap) ||
                    (score == best_score && node_overlap == best_node_overlap &&
                     sensor_overlap < best_sensor_overlap) ||
                    (score == best_score && node_overlap == best_node_overlap &&
                     sensor_overlap == best_sensor_overlap && dist < best_dist) ||
                    (score == best_score && node_overlap == best_node_overlap &&
                     sensor_overlap == best_sensor_overlap && dist == best_dist &&
                     cand->num < best->num)) {
                    best = cand;
                    best_score = score;
                    best_dist = dist;
                    best_node_overlap = node_overlap;
                    best_sensor_overlap = sensor_overlap;
                }
            }

            if (best) {
                ctx->neutral_used[best->num] = 1;
                return best;
            }
        }
    }

    return NULL;
}

static track_node *game_pick_neutral_standby_target(game_context_t *ctx) {
    game_role_slot_t *slot = &ctx->slots[GAME_ROLE_NEUTRAL];
    pos_target_query_t *query = &ctx->game_query_primary;
    track_node *best = NULL;
    int best_dist = 0x7fffffff;

    for (int i = 0; i < ctx->sensor_pool_count; i++) {
        track_node *cand = ctx->sensor_pool[i];

        if (!cand) continue;
        if (game_current_sensor_num(slot->train_num) == cand->num) continue;
        if (slot->target && slot->target->num == cand->num) continue;
        if (pos_query_target(slot->train_num, cand, query) != POS_TARGET_READY) continue;
        if (query->plan.total_dist_mm < best_dist) {
            best = cand;
            best_dist = query->plan.total_dist_mm;
        }
    }

    return best;
}

static int game_dispatch_target(game_context_t *ctx, game_role_t role, track_node *target,
                                int is_standby) {
    game_role_slot_t *slot = &ctx->slots[role];
    int pos_tid = game_position_server_tid(ctx);

    if (pos_tid < 0 || !slot || !target) return 0;
    if (!PositionServerGoto(pos_tid, slot->train_num,
                            (int)(target - g_track), DEFAULT_SPEED_LEVEL, 0)) {
        return 0;
    }

    if (is_standby) {
        slot->standby_target = target;
        slot->standby_sensor_num = (uint16_t)(target->num + 1);
        slot->standby_dispatched = 1;
    } else {
        slot->target = target;
        slot->target_sensor_num = (uint16_t)(target->num + 1);
        slot->dispatched = 1;
    }
    return 1;
}

void game_begin_wait_pick(game_context_t *ctx) {
    ctx->round_priority = game_current_priority_role(ctx);
    ctx->slots[GAME_ROLE_AI].target = game_pick_best_ai_target(ctx);
    ctx->slots[GAME_ROLE_AI].target_sensor_num =
        ctx->slots[GAME_ROLE_AI].target ? (uint16_t)(ctx->slots[GAME_ROLE_AI].target->num + 1) : 0;
    ctx->slots[GAME_ROLE_NEUTRAL].target = game_draw_neutral_target(ctx);
    ctx->slots[GAME_ROLE_NEUTRAL].target_sensor_num =
        ctx->slots[GAME_ROLE_NEUTRAL].target ? (uint16_t)(ctx->slots[GAME_ROLE_NEUTRAL].target->num + 1) : 0;

    if (!ctx->slots[GAME_ROLE_AI].target || !ctx->slots[GAME_ROLE_NEUTRAL].target) {
        ctx->state = GAME_STATE_STOPPING;
        game_set_hint(ctx, "target selection failed");
        game_log_line("game: failed to pick AI or neutral target");
        return;
    }

    ctx->state = GAME_STATE_WAIT_PICK;
    game_set_hint(ctx, "Enter sensor name");
    game_set_result(ctx, "-");
    ui_set_cmd_prompt_label("pick> ");
    ui_cmd_newprompt();
    ui_mark_position_dirty();
}

int game_start_next_unstarted(game_context_t *ctx) {
    int pos_tid = game_position_server_tid(ctx);

    for (int i = 0; i < GAME_ROLES; i++) {
        game_role_slot_t *slot = &ctx->slots[i];
        if (slot->train_num < 0 || slot->started) continue;
        if (game_train_is_known_stopped(slot->train_num)) {
            slot->started = 1;
            return 1;
        }
        if (pos_tid < 0) return 0;
        if (!PositionServerStartFindPos(pos_tid, slot->train_num)) return 0;
        slot->started = 1;
        return 1;
    }
    return 0;
}

static void game_emergency_stop_train(game_context_t *ctx, int train_num) {
    int pos_tid = game_position_server_tid(ctx);

    if (train_num < 0) return;
    if (pos_tid >= 0) {
        (void)PositionServerSpeedChange(pos_tid, train_num, 0);
    }
    track_set_speed(train_num, 0);
    pos_reset_dead_train(train_num);
    if (pos_tid >= 0) {
        (void)PositionServerReleaseTrain(pos_tid, train_num);
    }
}

void game_force_stop_now(game_context_t *ctx) {
    for (int i = 0; i < GAME_ROLES; i++) {
        if (ctx->slots[i].train_num < 0) continue;
        game_emergency_stop_train(ctx, ctx->slots[i].train_num);
    }
    game_reset_all(ctx);
    ui_mark_position_dirty();
    ui_set_cmd_prompt_label("cmd> ");
    ui_cmd_newprompt();
}

static void game_finish_match(game_context_t *ctx) {
    ui_set_cmd_prompt_label("cmd> ");

    if (ctx->score_half[GAME_ROLE_HUMAN] > ctx->score_half[GAME_ROLE_AI]) {
        game_set_result(ctx, "Human wins");
        game_log_line("game: match over, Human wins");
    } else if (ctx->score_half[GAME_ROLE_HUMAN] < ctx->score_half[GAME_ROLE_AI]) {
        game_set_result(ctx, "AI wins");
        game_log_line("game: match over, AI wins");
    } else {
        game_set_result(ctx, "Draw");
        game_log_line("game: match over, draw");
    }
    ctx->state = GAME_STATE_MATCH_OVER;
    ui_mark_position_dirty();
}

void game_advance_round_or_finish(game_context_t *ctx) {
    game_log_round_result(ctx);
    if (ctx->round_index + 1 >= GAME_ROUNDS) {
        game_finish_match(ctx);
        return;
    }
    ctx->round_index++;
    game_reset_round_state(ctx);
    game_begin_wait_pick(ctx);
}

int game_round_done(game_context_t *ctx) {
    return ctx->slots[GAME_ROLE_HUMAN].completed &&
           ctx->slots[GAME_ROLE_AI].completed &&
           (ctx->slots[GAME_ROLE_NEUTRAL].completed ||
            ctx->slots[GAME_ROLE_NEUTRAL].standby_completed);
}

void game_consume_events(game_context_t *ctx) {
    pos_game_event_t *events = ctx->game_event_batch;
    int count;

    count = pos_read_game_events(&ctx->event_seq, events, GAME_EVENT_BATCH);
    while (count > 0) {
        for (int i = 0; i < count; i++) {
            int role = game_role_index_from_train(ctx, events[i].train_num);
            if (role < 0) continue;

            if (events[i].type == POS_GAME_EVENT_SENSOR_HIT &&
                ctx->state == GAME_STATE_ROUND_RUNNING &&
                role != GAME_ROLE_NEUTRAL) {
                int idx = (int)events[i].sensor_num - 1;
                uint8_t bit = (role == GAME_ROLE_HUMAN) ? 1u : 2u;
                uint8_t other = (role == GAME_ROLE_HUMAN) ? 2u : 1u;
                int add = 0;

                if (idx < 0 || idx >= GAME_SENSOR_KEYS) continue;
                if (ctx->claim_mask[idx] & bit) continue;
                add = (ctx->claim_mask[idx] & other) ? 2 : 3;
                ctx->claim_mask[idx] |= bit;
                ctx->score_half[role] += add;
                ctx->slots[role].score_delta_half += add;
                ui_mark_position_dirty();
            } else if (events[i].type == POS_GAME_EVENT_GOAL_STOP &&
                       ctx->state == GAME_STATE_ROUND_RUNNING) {
                if (game_sensor_matches_target(events[i].sensor_num,
                                               ctx->slots[role].target)) {
                    ctx->slots[role].completed = 1;
                    ui_mark_position_dirty();
                }
                if (role == GAME_ROLE_NEUTRAL &&
                    ctx->slots[role].standby_dispatched &&
                    game_sensor_matches_target(events[i].sensor_num,
                                               ctx->slots[role].standby_target)) {
                    ctx->slots[role].standby_completed = 1;
                    ui_mark_position_dirty();
                }
            }
        }
        count = pos_read_game_events(&ctx->event_seq, events, GAME_EVENT_BATCH);
    }
}

static void game_try_redirect_neutral(game_context_t *ctx, track_node *standby, const char *reason) {
    char buf[96];
    char *p = buf;
    char *end = buf + sizeof(buf) - 1;
    game_role_slot_t *neutral = &ctx->slots[GAME_ROLE_NEUTRAL];

    if (!standby) return;
    if (!game_dispatch_target(ctx, GAME_ROLE_NEUTRAL, standby, 1)) return;

    p = buf_append_cap(p, end, "game: neutral rerouted ");
    p = buf_append_cap(p, end,
                       neutral->target && neutral->target->name ? neutral->target->name : "-");
    p = buf_append_cap(p, end, " -> ");
    p = buf_append_cap(p, end, standby->name ? standby->name : "-");
    if (reason && reason[0]) {
        p = buf_append_cap(p, end, " (");
        p = buf_append_cap(p, end, reason);
        p = buf_append_cap(p, end, ")");
    }
    *p = '\0';
    game_log_line(buf);
    ui_mark_position_dirty();
}

static void game_mark_neutral_resolved_at_standby(game_context_t *ctx, track_node *standby,
                                                  const char *reason) {
    game_role_slot_t *neutral = &ctx->slots[GAME_ROLE_NEUTRAL];
    char buf[144];
    char *p = buf;
    char *end = buf + sizeof(buf) - 1;

    if (!standby) return;

    neutral->standby_target = standby;
    neutral->standby_sensor_num = (uint16_t)(standby->num + 1);
    neutral->standby_dispatched = 1;
    neutral->standby_completed = 1;

    p = buf_append_cap(p, end, "game: neutral resolved at ");
    p = buf_append_cap(p, end, standby->name ? standby->name : "-");
    if (reason && reason[0]) {
        p = buf_append_cap(p, end, " (");
        p = buf_append_cap(p, end, reason);
        p = buf_append_cap(p, end, ")");
    }
    *p = '\0';
    game_log_line(buf);
    game_set_hint(ctx, "Neutral resolved at standby");
    ui_mark_position_dirty();
}

void game_try_resolve_neutral_cases(game_context_t *ctx) {
    game_role_slot_t *neutral = &ctx->slots[GAME_ROLE_NEUTRAL];
    pos_target_query_t *query = &ctx->game_query_primary;
    train_pos_t *neutral_pos;

    if (ctx->state != GAME_STATE_ROUND_RUNNING) return;
    game_log_neutral_detour_if_needed(ctx);
    if (neutral->standby_dispatched || neutral->completed) return;

    neutral_pos = pos_get(neutral->train_num);
    if (!neutral_pos) return;

    if (neutral_pos->route_state == TRAIN_STATE_STOPPED) {
        for (int role = GAME_ROLE_HUMAN; role <= GAME_ROLE_AI; role++) {
            int neutral_bit;

            if (ctx->slots[role].completed) continue;
            if (!ctx->slots[role].target) continue;
            if (pos_query_target(ctx->slots[role].train_num, ctx->slots[role].target, query) != POS_TARGET_BLOCKED) {
                continue;
            }
            neutral_bit = game_train_bit(neutral->train_num);
            if (query->blocker_mask == neutral_bit) {
                game_try_redirect_neutral(ctx, game_pick_neutral_standby_target(ctx),
                                          "player blocked by neutral");
                return;
            }
        }
    }

    if (ctx->slots[GAME_ROLE_HUMAN].completed && ctx->slots[GAME_ROLE_AI].completed &&
        neutral->target != NULL) {
        int player_mask = game_train_bit(ctx->slots[GAME_ROLE_HUMAN].train_num) |
                          game_train_bit(ctx->slots[GAME_ROLE_AI].train_num);

        if (pos_query_target(neutral->train_num, neutral->target, query) == POS_TARGET_BLOCKED &&
            query->blocker_mask != 0 &&
            (query->blocker_mask & (uint8_t)~player_mask) == 0) {
            if (neutral_pos->deadlock_recover.valid &&
                neutral_pos->deadlock_recover.parked_at_yield &&
                neutral_pos->deadlock_recover.yield_target != NULL) {
                game_mark_neutral_resolved_at_standby(
                    ctx,
                    neutral_pos->deadlock_recover.yield_target,
                    "official target blocked by completed players");
            } else {
                game_try_redirect_neutral(ctx, game_pick_neutral_standby_target(ctx),
                                          "official target blocked by completed players");
            }
            return;
        }
    }
}

static int game_round_dispatch_failed(game_context_t *ctx) {
    ctx->state = GAME_STATE_STOPPING;
    game_set_hint(ctx, "Dispatch failed");
    game_log_line("game: round dispatch failed");
    return 2;
}

int game_handle_pick(game_context_t *ctx, const train_command_t *cmd) {
    track_node *target;
    pos_target_query_t *query = &ctx->game_query_primary;
    int require_min_trip;
    game_role_t first_role;
    game_role_t second_role;

    if (!cmd) return 2;
    if (ctx->state != GAME_STATE_WAIT_PICK) {
        game_log_line("game: not waiting for pick");
        return 2;
    }
    if (cmd->target_idx < 0 || cmd->target_idx >= TRACK_MAX) {
        game_log_line("pick: invalid sensor");
        return 2;
    }

    target = &g_track[cmd->target_idx];
    if (target->type != NODE_SENSOR || !target->name) {
        game_log_line("pick: sensor target required");
        return 2;
    }
    if (game_current_sensor_num(ctx->slots[GAME_ROLE_HUMAN].train_num) == target->num) {
        game_log_line("pick: cannot choose current sensor");
        return 2;
    }
    if (pos_query_target(ctx->slots[GAME_ROLE_HUMAN].train_num, target, query) == POS_TARGET_UNREACHABLE) {
        game_log_line("pick: target unreachable");
        return 2;
    }
    require_min_trip = game_train_has_min_trip_candidate(ctx, ctx->slots[GAME_ROLE_HUMAN].train_num);
    if (require_min_trip && query->plan.total_dist_mm < GAME_MIN_TRIP_MM) {
        game_log_line("pick: choose a longer reachable target");
        return 2;
    }

    ctx->slots[GAME_ROLE_HUMAN].target = target;
    ctx->slots[GAME_ROLE_HUMAN].target_sensor_num = (uint16_t)(target->num + 1);
    ctx->state = GAME_STATE_ROUND_RUNNING;
    ui_set_cmd_prompt_label("game> ");
    game_set_hint(ctx, "Targets revealed");
    game_set_result(ctx, "-");
    game_log_targets(ctx);

    first_role = ctx->round_priority;
    second_role = (first_role == GAME_ROLE_HUMAN) ? GAME_ROLE_AI : GAME_ROLE_HUMAN;

    if (!game_dispatch_target(ctx, first_role, ctx->slots[first_role].target, 0) ||
        !game_dispatch_target(ctx, second_role, ctx->slots[second_role].target, 0) ||
        !game_dispatch_target(ctx, GAME_ROLE_NEUTRAL, ctx->slots[GAME_ROLE_NEUTRAL].target, 0)) {
        return game_round_dispatch_failed(ctx);
    }
    ui_mark_position_dirty();
    return 2;
}
