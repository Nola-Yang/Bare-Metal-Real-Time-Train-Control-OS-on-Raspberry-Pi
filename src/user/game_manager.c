#include "game_manager.h"
#include "demo_manager.h"
#include "server/nameserver.h"
#include "server/position_server.h"
#include "track.h"
#include "train_tracking/position.h"
#include "ui.h"
#include "util.h"
#include "timer.h"
#include <stddef.h>
#include <stdint.h>

#define GAME_ROUNDS 4
#define GAME_ROLES 3
#define GAME_SCORE_PLAYERS 2
#define GAME_SENSOR_KEYS 80
#define GAME_MIN_TRIP_MM 1400
#define GAME_EVENT_BATCH 16

typedef enum {
    GAME_STATE_OFF = 0,
    GAME_STATE_STARTING = 1,
    GAME_STATE_WAIT_PICK = 2,
    GAME_STATE_ROUND_RUNNING = 3,
    GAME_STATE_MATCH_OVER = 4,
    GAME_STATE_STOPPING = 5,
} game_state_t;

typedef enum {
    GAME_ROLE_HUMAN = 0,
    GAME_ROLE_AI = 1,
    GAME_ROLE_NEUTRAL = 2,
} game_role_t;

typedef struct {
    int train_num;
    int started;
    track_node *target;
    uint16_t target_sensor_num;
    int dispatched;
    int completed;
    int score_delta_half;
    track_node *standby_target;
    uint16_t standby_sensor_num;
    int standby_dispatched;
    int standby_completed;
} game_role_slot_t;

static game_state_t g_game_state = GAME_STATE_OFF;
static uint32_t g_game_seed = 1;
static uint32_t g_game_rng_state = 1;
static uint64_t g_game_start_us = 0;
static int g_position_server_tid = -1;

static game_role_slot_t g_slots[GAME_ROLES];
static int g_round_index = -1;
static game_role_t g_round1_priority = GAME_ROLE_HUMAN;
static game_role_t g_round_priority = GAME_ROLE_HUMAN;
static uint8_t g_claim_mask[GAME_SENSOR_KEYS];
static int g_score_half[GAME_SCORE_PLAYERS];
static uint8_t g_neutral_used[GAME_SENSOR_KEYS];
static uint32_t g_event_seq = 0;
static track_node *g_sensor_pool[TRACK_MAX];
static int g_sensor_pool_count = 0;
static char g_hint_text[80] = "-";
static char g_result_text[96] = "-";

static int game_position_server_tid(void) {
    if (g_position_server_tid < 0) {
        g_position_server_tid = WhoIs(POSITION_SERVER_NAME);
    }
    return g_position_server_tid;
}

static const char *game_state_name(game_state_t state) {
    switch (state) {
    case GAME_STATE_OFF: return "OFF";
    case GAME_STATE_STARTING: return "STARTING";
    case GAME_STATE_WAIT_PICK: return "WAIT_PICK";
    case GAME_STATE_ROUND_RUNNING: return "ROUND_RUNNING";
    case GAME_STATE_MATCH_OVER: return "MATCH_OVER";
    case GAME_STATE_STOPPING: return "STOPPING";
    default: return "UNK";
    }
}

static const char *game_role_name(game_role_t role) {
    switch (role) {
    case GAME_ROLE_HUMAN: return "Human";
    case GAME_ROLE_AI: return "AI";
    case GAME_ROLE_NEUTRAL: return "Neutral";
    default: return "-";
    }
}

static void game_set_hint(const char *text) {
    int i = 0;
    if (!text || !text[0]) text = "-";
    while (text[i] && i + 1 < (int)sizeof(g_hint_text)) {
        g_hint_text[i] = text[i];
        i++;
    }
    g_hint_text[i] = '\0';
}

static void game_set_result(const char *text) {
    int i = 0;
    if (!text || !text[0]) text = "-";
    while (text[i] && i + 1 < (int)sizeof(g_result_text)) {
        g_result_text[i] = text[i];
        i++;
    }
    g_result_text[i] = '\0';
}

static void game_log_line(const char *text) {
    if (!text) return;
    ui_cmd_puts(text);
    ui_cmd_puts("\r\n");
}

static void game_log_targets(void) {
    char *buf = buf_get_temp();
    char *p = buf;
    game_role_slot_t *human = &g_slots[GAME_ROLE_HUMAN];
    game_role_slot_t *ai = &g_slots[GAME_ROLE_AI];
    game_role_slot_t *neutral = &g_slots[GAME_ROLE_NEUTRAL];

    p = buf_append(p, "game: reveal H=");
    p = buf_append(p, human->target && human->target->name ? human->target->name : "-");
    p = buf_append(p, " AI=");
    p = buf_append(p, ai->target && ai->target->name ? ai->target->name : "-");
    p = buf_append(p, " N=");
    p = buf_append(p, neutral->target && neutral->target->name ? neutral->target->name : "-");
    *p = '\0';
    game_log_line(buf);
}

static void game_log_round_result(void) {
    char *buf = buf_get_temp();
    char *p = buf;

    p = buf_append(p, "game: round ");
    p = buf_append_int(p, g_round_index + 1);
    p = buf_append(p, " end H=");
    p = buf_append_int(p, g_score_half[GAME_ROLE_HUMAN]);
    p = buf_append(p, "/2 AI=");
    p = buf_append_int(p, g_score_half[GAME_ROLE_AI]);
    p = buf_append(p, "/2");
    *p = '\0';
    game_log_line(buf);
}

static void game_reset_slot(game_role_slot_t *slot) {
    if (!slot) return;
    slot->started = 0;
    slot->target = NULL;
    slot->target_sensor_num = 0;
    slot->dispatched = 0;
    slot->completed = 0;
    slot->score_delta_half = 0;
    slot->standby_target = NULL;
    slot->standby_sensor_num = 0;
    slot->standby_dispatched = 0;
    slot->standby_completed = 0;
}

static void game_reset_all(void) {
    for (int i = 0; i < GAME_ROLES; i++) {
        g_slots[i].train_num = -1;
        game_reset_slot(&g_slots[i]);
    }
    for (int i = 0; i < GAME_SENSOR_KEYS; i++) {
        g_claim_mask[i] = 0;
        g_neutral_used[i] = 0;
    }
    for (int i = 0; i < GAME_SCORE_PLAYERS; i++) {
        g_score_half[i] = 0;
    }
    g_game_state = GAME_STATE_OFF;
    g_round_index = -1;
    g_round1_priority = GAME_ROLE_HUMAN;
    g_round_priority = GAME_ROLE_HUMAN;
    g_game_start_us = 0;
    g_event_seq = 0;
    g_position_server_tid = -1;
    game_set_hint("-");
    game_set_result("-");
    ui_set_cmd_prompt_label("cmd> ");
}

static int parse_int_token_local(const char *tok, int *out) {
    const char *p;
    if (!tok || !tok[0] || !out) return 0;
    p = tok;
    if (*p == '+' || *p == '-') p++;
    if (!*p) return 0;
    while (*p) {
        if (*p < '0' || *p > '9') return 0;
        p++;
    }
    *out = str2int(tok);
    return 1;
}

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

static uint32_t game_rand_u32(void) {
    uint32_t x = g_game_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_game_rng_state = (x == 0) ? 1U : x;
    return g_game_rng_state;
}

static void game_seed_rng(uint32_t seed) {
    g_game_seed = (seed == 0) ? 1U : seed;
    g_game_rng_state = g_game_seed;
}

static int game_build_sensor_pool(void) {
    g_sensor_pool_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type != NODE_SENSOR) continue;
        if (!g_track[i].name) continue;
        if (g_track[i].num < 0 || g_track[i].num >= GAME_SENSOR_KEYS) continue;
        g_sensor_pool[g_sensor_pool_count++] = &g_track[i];
    }
    return g_sensor_pool_count;
}

static int game_role_index_from_train(int train_num) {
    for (int i = 0; i < GAME_ROLES; i++) {
        if (g_slots[i].train_num == train_num) return i;
    }
    return -1;
}

static int game_train_is_known_stopped(int train_num) {
    train_pos_t *pos = pos_get(train_num);
    return pos && pos->cur_sensor != NULL && pos->route_state == TRAIN_STATE_STOPPED;
}

static int game_train_is_position_confirmed(int train_num) {
    train_pos_t *pos = pos_get(train_num);
    if (!pos || pos->cur_sensor == NULL) return 0;
    return pos->route_state != TRAIN_STATE_UNKNOWN &&
           pos->route_state != TRAIN_STATE_FIND_POS &&
           pos->route_state != TRAIN_STATE_STOPPING_GOTO;
}

static int game_any_active_goto(void) {
    for (int i = 0; i < GAME_ROLES; i++) {
        if (g_slots[i].train_num < 0) continue;
        if (pos_is_train_goto_active(g_slots[i].train_num)) return 1;
    }
    return 0;
}

static int game_current_sensor_num(int train_num) {
    train_pos_t *pos = pos_get(train_num);
    if (!pos || !pos->cur_sensor) return -1;
    return pos->cur_sensor->num;
}

static int game_is_sensor_currently_occupied(int sensor_num) {
    for (int i = 0; i < GAME_ROLES; i++) {
        if (g_slots[i].train_num < 0) continue;
        if (game_current_sensor_num(g_slots[i].train_num) == sensor_num) return 1;
    }
    return 0;
}

static int game_train_has_min_trip_candidate(int train_num) {
    for (int i = 0; i < g_sensor_pool_count; i++) {
        track_node *cand = g_sensor_pool[i];
        pos_target_query_t query;

        if (!cand) continue;
        if (game_current_sensor_num(train_num) == cand->num) continue;
        if (pos_query_target(train_num, cand, &query) == POS_TARGET_UNREACHABLE) continue;
        if (query.plan.total_dist_mm >= GAME_MIN_TRIP_MM) return 1;
    }
    return 0;
}

static game_role_t game_current_priority_role(void) {
    if ((g_round_index & 1) == 0) return g_round1_priority;
    return (g_round1_priority == GAME_ROLE_HUMAN) ? GAME_ROLE_AI : GAME_ROLE_HUMAN;
}

static int game_bit_count(uint8_t mask) {
    int count = 0;
    while (mask) {
        count += (mask & 1u);
        mask >>= 1;
    }
    return count;
}

static void game_reset_round_state(void) {
    for (int i = 0; i < GAME_ROLES; i++) {
        g_slots[i].target = NULL;
        g_slots[i].target_sensor_num = 0;
        g_slots[i].dispatched = 0;
        g_slots[i].completed = 0;
        g_slots[i].score_delta_half = 0;
        g_slots[i].standby_target = NULL;
        g_slots[i].standby_sensor_num = 0;
        g_slots[i].standby_dispatched = 0;
        g_slots[i].standby_completed = 0;
    }
}

static int game_collect_route_value(const pos_target_query_t *query, track_node *requested_target,
                                    game_role_t role) {
    uint8_t seen[GAME_SENSOR_KEYS];
    int score = 0;
    uint16_t exclude_num = requested_target ? (uint16_t)(requested_target->num + 1) : 0;
    uint16_t exclude_chosen = (query && query->plan.chosen_target)
                              ? (uint16_t)(query->plan.chosen_target->num + 1)
                              : 0;

    for (int i = 0; i < GAME_SENSOR_KEYS; i++) seen[i] = 0;
    if (!query) return 0;

    for (int pass = 0; pass < 2; pass++) {
        const uint16_t *path = (pass == 0) ? query->plan.path_nodes : query->plan.path_nodes2;
        int path_count = (pass == 0) ? query->plan.path_count : query->plan.path_count2;
        for (int i = 0; i < path_count; i++) {
            int idx = (int)path[i];
            int sensor_num;
            uint8_t role_bit;
            uint8_t other_bit;

            if (idx < 0 || idx >= TRACK_MAX) continue;
            if (g_track[idx].type != NODE_SENSOR) continue;
            sensor_num = g_track[idx].num;
            if (sensor_num < 0 || sensor_num >= GAME_SENSOR_KEYS) continue;
            if ((uint16_t)(sensor_num + 1) == exclude_num ||
                (uint16_t)(sensor_num + 1) == exclude_chosen) {
                continue;
            }
            if (seen[sensor_num]) continue;
            seen[sensor_num] = 1;
            role_bit = (role == GAME_ROLE_HUMAN) ? 1u : 2u;
            other_bit = (role == GAME_ROLE_HUMAN) ? 2u : 1u;
            if (g_claim_mask[sensor_num] & role_bit) continue;
            score += (g_claim_mask[sensor_num] & other_bit) ? 2 : 3;
        }
    }

    return score;
}

static track_node *game_pick_best_ai_target(void) {
    game_role_slot_t *slot = &g_slots[GAME_ROLE_AI];
    track_node *best = NULL;
    pos_target_query_t best_query = {0};
    int best_score = -0x7fffffff;
    int best_dist = 0x7fffffff;

    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < g_sensor_pool_count; i++) {
            track_node *cand = g_sensor_pool[i];
            pos_target_query_t query;
            int score;
            int dist;

            if (!cand) continue;
            if (game_current_sensor_num(slot->train_num) == cand->num) continue;
            if (pass == 0 && cand->name == NULL) continue;
            if (pos_query_target(slot->train_num, cand, &query) == POS_TARGET_UNREACHABLE) {
                continue;
            }
            dist = query.plan.total_dist_mm;
            if (pass == 0 && dist < GAME_MIN_TRIP_MM) continue;

            score = game_collect_route_value(&query, cand, GAME_ROLE_AI);
            score -= dist / 400;
            if (query.status == POS_TARGET_BLOCKED) {
                score -= 2 + game_bit_count(query.blocker_mask);
            }

            if (!best ||
                score > best_score ||
                (score == best_score && dist < best_dist) ||
                (score == best_score && dist == best_dist && (game_rand_u32() & 1u))) {
                best = cand;
                best_query = query;
                best_score = score;
                best_dist = dist;
            }
        }
        if (best) {
            (void)best_query;
            return best;
        }
    }

    return NULL;
}

static track_node *game_draw_neutral_target(void) {
    game_role_slot_t *slot = &g_slots[GAME_ROLE_NEUTRAL];
    track_node *eligible[TRACK_MAX];
    int eligible_count = 0;

    for (int pass = 0; pass < 2; pass++) {
        eligible_count = 0;
        for (int i = 0; i < g_sensor_pool_count; i++) {
            track_node *cand = g_sensor_pool[i];
            pos_target_query_t query;

            if (!cand) continue;
            if (cand->num < 0 || cand->num >= GAME_SENSOR_KEYS) continue;
            if (g_neutral_used[cand->num]) continue;
            if (game_is_sensor_currently_occupied(cand->num)) continue;
            if (pos_query_target(slot->train_num, cand, &query) == POS_TARGET_UNREACHABLE) continue;
            if (pass == 0 && query.plan.total_dist_mm < GAME_MIN_TRIP_MM) continue;
            eligible[eligible_count++] = cand;
        }

        if (eligible_count > 0) {
            int pick = (int)(game_rand_u32() % (uint32_t)eligible_count);
            g_neutral_used[eligible[pick]->num] = 1;
            return eligible[pick];
        }
    }

    return NULL;
}

static track_node *game_pick_neutral_standby_target(void) {
    game_role_slot_t *slot = &g_slots[GAME_ROLE_NEUTRAL];
    track_node *best = NULL;
    int best_dist = 0x7fffffff;

    for (int i = 0; i < g_sensor_pool_count; i++) {
        track_node *cand = g_sensor_pool[i];
        pos_target_query_t query;

        if (!cand) continue;
        if (game_current_sensor_num(slot->train_num) == cand->num) continue;
        if (slot->target && slot->target->num == cand->num) continue;
        if (pos_query_target(slot->train_num, cand, &query) != POS_TARGET_READY) continue;
        if (query.plan.total_dist_mm < best_dist) {
            best = cand;
            best_dist = query.plan.total_dist_mm;
        }
    }

    return best;
}

static int game_dispatch_target(game_role_t role, track_node *target, int is_standby) {
    game_role_slot_t *slot = &g_slots[role];
    int pos_tid = game_position_server_tid();

    if (pos_tid < 0 || !slot || !target) return 0;
    if (!PositionServerGoto(pos_tid, slot->train_num, (int)(target - g_track), 0)) {
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

static void game_begin_wait_pick(void) {
    g_round_priority = game_current_priority_role();
    g_slots[GAME_ROLE_AI].target = game_pick_best_ai_target();
    g_slots[GAME_ROLE_AI].target_sensor_num =
        g_slots[GAME_ROLE_AI].target ? (uint16_t)(g_slots[GAME_ROLE_AI].target->num + 1) : 0;
    g_slots[GAME_ROLE_NEUTRAL].target = game_draw_neutral_target();
    g_slots[GAME_ROLE_NEUTRAL].target_sensor_num =
        g_slots[GAME_ROLE_NEUTRAL].target ? (uint16_t)(g_slots[GAME_ROLE_NEUTRAL].target->num + 1) : 0;

    if (!g_slots[GAME_ROLE_AI].target || !g_slots[GAME_ROLE_NEUTRAL].target) {
        g_game_state = GAME_STATE_STOPPING;
        game_set_hint("target selection failed");
        game_log_line("game: failed to pick AI or neutral target");
        return;
    }

    g_game_state = GAME_STATE_WAIT_PICK;
    game_set_hint("Use pick <sensor>");
    game_set_result("-");
    ui_set_cmd_prompt_label("pick> ");
    ui_cmd_newprompt();
    ui_mark_position_dirty();
}

static int game_start_next_unstarted(void) {
    int pos_tid = game_position_server_tid();

    for (int i = 0; i < GAME_ROLES; i++) {
        game_role_slot_t *slot = &g_slots[i];
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

static void game_emergency_stop_train(int train_num) {
    int pos_tid = game_position_server_tid();

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

static void game_force_stop_now(void) {
    for (int i = 0; i < GAME_ROLES; i++) {
        if (g_slots[i].train_num < 0) continue;
        game_emergency_stop_train(g_slots[i].train_num);
    }
    game_reset_all();
    ui_mark_position_dirty();
    ui_set_cmd_prompt_label("cmd> ");
    ui_cmd_newprompt();
}

static void game_finish_match(void) {
    if (g_score_half[GAME_ROLE_HUMAN] > g_score_half[GAME_ROLE_AI]) {
        game_set_result("Human wins");
        game_log_line("game: match over, Human wins");
    } else if (g_score_half[GAME_ROLE_HUMAN] < g_score_half[GAME_ROLE_AI]) {
        game_set_result("AI wins");
        game_log_line("game: match over, AI wins");
    } else {
        game_set_result("Draw");
        game_log_line("game: match over, draw");
    }
    g_game_state = GAME_STATE_MATCH_OVER;
    ui_set_cmd_prompt_label("cmd> ");
    ui_mark_position_dirty();
}

static void game_advance_round_or_finish(void) {
    game_log_round_result();
    if (g_round_index + 1 >= GAME_ROUNDS) {
        game_finish_match();
        return;
    }
    g_round_index++;
    game_reset_round_state();
    game_begin_wait_pick();
}

static int game_round_done(void) {
    return g_slots[GAME_ROLE_HUMAN].completed &&
           g_slots[GAME_ROLE_AI].completed &&
           (g_slots[GAME_ROLE_NEUTRAL].completed ||
            g_slots[GAME_ROLE_NEUTRAL].standby_completed);
}

static void game_consume_events(void) {
    pos_game_event_t events[GAME_EVENT_BATCH];
    int count;

    count = pos_read_game_events(&g_event_seq, events, GAME_EVENT_BATCH);
    while (count > 0) {
        for (int i = 0; i < count; i++) {
            int role = game_role_index_from_train(events[i].train_num);
            if (role < 0) continue;

            if (events[i].type == POS_GAME_EVENT_SENSOR_HIT &&
                g_game_state == GAME_STATE_ROUND_RUNNING &&
                role != GAME_ROLE_NEUTRAL) {
                int idx = (int)events[i].sensor_num - 1;
                uint8_t bit = (role == GAME_ROLE_HUMAN) ? 1u : 2u;
                uint8_t other = (role == GAME_ROLE_HUMAN) ? 2u : 1u;
                int add = 0;

                if (idx < 0 || idx >= GAME_SENSOR_KEYS) continue;
                if (g_claim_mask[idx] & bit) continue;
                add = (g_claim_mask[idx] & other) ? 2 : 3;
                g_claim_mask[idx] |= bit;
                g_score_half[role] += add;
                g_slots[role].score_delta_half += add;
                ui_mark_position_dirty();
            } else if (events[i].type == POS_GAME_EVENT_GOAL_STOP &&
                       g_game_state == GAME_STATE_ROUND_RUNNING) {
                if (events[i].sensor_num == g_slots[role].target_sensor_num &&
                    g_slots[role].target_sensor_num != 0) {
                    g_slots[role].completed = 1;
                    ui_mark_position_dirty();
                }
                if (role == GAME_ROLE_NEUTRAL &&
                    events[i].sensor_num == g_slots[role].standby_sensor_num &&
                    g_slots[role].standby_dispatched) {
                    g_slots[role].standby_completed = 1;
                    ui_mark_position_dirty();
                }
            }
        }
        count = pos_read_game_events(&g_event_seq, events, GAME_EVENT_BATCH);
    }
}

static void game_try_redirect_neutral(track_node *standby, const char *reason) {
    char *buf;
    char *p;

    if (!standby) return;
    if (!game_dispatch_target(GAME_ROLE_NEUTRAL, standby, 1)) return;

    buf = buf_get_temp();
    p = buf;
    p = buf_append(p, "game: neutral standby ");
    p = buf_append(p, standby->name ? standby->name : "-");
    if (reason && reason[0]) {
        p = buf_append(p, " (");
        p = buf_append(p, reason);
        p = buf_append(p, ")");
    }
    *p = '\0';
    game_log_line(buf);
    ui_mark_position_dirty();
}

static void game_try_resolve_neutral_cases(void) {
    game_role_slot_t *neutral = &g_slots[GAME_ROLE_NEUTRAL];
    train_pos_t *neutral_pos;

    if (g_game_state != GAME_STATE_ROUND_RUNNING) return;
    if (neutral->standby_dispatched || neutral->completed) return;

    neutral_pos = pos_get(neutral->train_num);
    if (!neutral_pos) return;

    if (neutral_pos->route_state == TRAIN_STATE_STOPPED) {
        for (int role = GAME_ROLE_HUMAN; role <= GAME_ROLE_AI; role++) {
            pos_target_query_t query;
            int neutral_bit;

            if (g_slots[role].completed) continue;
            if (!g_slots[role].target) continue;
            if (pos_query_target(g_slots[role].train_num, g_slots[role].target, &query) != POS_TARGET_BLOCKED) {
                continue;
            }
            neutral_bit = game_train_bit(neutral->train_num);
            if (query.blocker_mask == neutral_bit) {
                game_try_redirect_neutral(game_pick_neutral_standby_target(),
                                          "player blocked by neutral");
                return;
            }
        }
    }

    if (g_slots[GAME_ROLE_HUMAN].completed && g_slots[GAME_ROLE_AI].completed &&
        neutral->target != NULL) {
        pos_target_query_t query;
        int player_mask = game_train_bit(g_slots[GAME_ROLE_HUMAN].train_num) |
                          game_train_bit(g_slots[GAME_ROLE_AI].train_num);

        if (pos_query_target(neutral->train_num, neutral->target, &query) == POS_TARGET_BLOCKED &&
            query.blocker_mask != 0 &&
            (query.blocker_mask & (uint8_t)~player_mask) == 0) {
            game_try_redirect_neutral(game_pick_neutral_standby_target(),
                                      "neutral blocked by completed players");
        }
    }
}

static int game_handle_pick(const train_command_t *cmd) {
    track_node *target;
    pos_target_query_t query;
    int require_min_trip;

    if (!cmd) return 2;
    if (g_game_state != GAME_STATE_WAIT_PICK) {
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
    if (game_current_sensor_num(g_slots[GAME_ROLE_HUMAN].train_num) == target->num) {
        game_log_line("pick: cannot choose current sensor");
        return 2;
    }
    if (pos_query_target(g_slots[GAME_ROLE_HUMAN].train_num, target, &query) == POS_TARGET_UNREACHABLE) {
        game_log_line("pick: target unreachable");
        return 2;
    }
    require_min_trip = game_train_has_min_trip_candidate(g_slots[GAME_ROLE_HUMAN].train_num);
    if (require_min_trip && query.plan.total_dist_mm < GAME_MIN_TRIP_MM) {
        game_log_line("pick: choose a longer reachable target");
        return 2;
    }

    g_slots[GAME_ROLE_HUMAN].target = target;
    g_slots[GAME_ROLE_HUMAN].target_sensor_num = (uint16_t)(target->num + 1);
    g_game_state = GAME_STATE_ROUND_RUNNING;
    ui_set_cmd_prompt_label("cmd> ");
    game_set_hint("Targets revealed");
    game_set_result("-");
    game_log_targets();

    if (g_round_priority == GAME_ROLE_HUMAN) {
        if (!game_dispatch_target(GAME_ROLE_HUMAN, g_slots[GAME_ROLE_HUMAN].target, 0) ||
            !game_dispatch_target(GAME_ROLE_AI, g_slots[GAME_ROLE_AI].target, 0)) {
            g_game_state = GAME_STATE_STOPPING;
            game_set_hint("Dispatch failed");
            game_log_line("game: round dispatch failed");
            return 2;
        }
    } else {
        if (!game_dispatch_target(GAME_ROLE_AI, g_slots[GAME_ROLE_AI].target, 0) ||
            !game_dispatch_target(GAME_ROLE_HUMAN, g_slots[GAME_ROLE_HUMAN].target, 0)) {
            g_game_state = GAME_STATE_STOPPING;
            game_set_hint("Dispatch failed");
            game_log_line("game: round dispatch failed");
            return 2;
        }
    }
    if (!game_dispatch_target(GAME_ROLE_NEUTRAL, g_slots[GAME_ROLE_NEUTRAL].target, 0)) {
        g_game_state = GAME_STATE_STOPPING;
        game_set_hint("Dispatch failed");
        game_log_line("game: round dispatch failed");
        return 2;
    }
    ui_mark_position_dirty();
    return 2;
}

static int game_print_status(void) {
    char *buf = buf_get_temp();
    char *p = buf;

    p = buf_append(p, "game: state=");
    p = buf_append(p, game_state_name(g_game_state));
    if (g_round_index >= 0) {
        p = buf_append(p, " round=");
        p = buf_append_int(p, g_round_index + 1);
    }
    p = buf_append(p, " H=");
    p = buf_append_int(p, g_score_half[GAME_ROLE_HUMAN]);
    p = buf_append(p, "/2 AI=");
    p = buf_append_int(p, g_score_half[GAME_ROLE_AI]);
    p = buf_append(p, "/2");
    *p = '\0';
    game_log_line(buf);
    return 2;
}

static int game_start_match(const train_command_t *cmd) {
    int trains[GAME_ROLES];
    int seed_override = -1;

    if (!cmd) return 2;
    if (demo_is_active()) {
        game_log_line("game: demo mode is active");
        return 2;
    }
    if (g_game_state != GAME_STATE_OFF) {
        game_log_line("game: already running");
        return 2;
    }
    if (cmd->argc != 5 && cmd->argc != 6) {
        game_log_line("Usage: game start <human> <ai> <neutral> [seed]");
        return 2;
    }

    for (int i = 0; i < GAME_ROLES; i++) {
        if (!parse_int_token_local(cmd->argv[2 + i], &trains[i]) ||
            !track_is_valid_train(trains[i])) {
            game_log_line("game start: invalid train number");
            return 2;
        }
    }
    if (trains[0] == trains[1] || trains[0] == trains[2] || trains[1] == trains[2]) {
        game_log_line("game start: duplicate train");
        return 2;
    }
    if (cmd->argc == 6 && !parse_int_token_local(cmd->argv[5], &seed_override)) {
        game_log_line("game start: invalid seed");
        return 2;
    }
    for (int i = 0; i < GAME_ROLES; i++) {
        train_pos_t *pos = pos_get(trains[i]);
        if (pos_is_train_goto_active(trains[i])) {
            game_log_line("game start: train busy with active goto");
            return 2;
        }
        if (pos && pos->route_state != TRAIN_STATE_STOPPED &&
            pos->route_state != TRAIN_STATE_UNKNOWN &&
            pos->route_state != TRAIN_STATE_DEAD_TRACK) {
            game_log_line("game start: train must be stopped or unknown");
            return 2;
        }
    }

    game_reset_all();
    game_build_sensor_pool();
    game_seed_rng((seed_override >= 0) ? (uint32_t)seed_override : (uint32_t)read_timer());
    g_slots[GAME_ROLE_HUMAN].train_num = trains[0];
    g_slots[GAME_ROLE_AI].train_num = trains[1];
    g_slots[GAME_ROLE_NEUTRAL].train_num = trains[2];
    g_round1_priority = (game_rand_u32() & 1u) ? GAME_ROLE_AI : GAME_ROLE_HUMAN;
    g_round_index = 0;
    g_game_state = GAME_STATE_STARTING;
    g_game_start_us = read_timer();
    game_set_hint("Bootstrapping trains");
    game_set_result("-");
    ui_set_cmd_prompt_label("cmd> ");
    ui_mark_position_dirty();
    (void)game_start_next_unstarted();
    game_log_line("game: bootstrapping trains one by one");
    return 2;
}

void game_init(void) {
    game_reset_all();
}

int game_handle_command(const train_command_t *cmd) {
    if (!cmd) return 2;

    if (cmd->type == TRAIN_CMD_PICK) {
        return game_handle_pick(cmd);
    }

    if (cmd->type != TRAIN_CMD_GAME) {
        return 2;
    }

    if (cmd->argc < 2) {
        game_log_line("Usage: game <start|stop|status> ...");
        return 2;
    }

    if (cmd->argv[1][0] == 's' && cmd->argv[1][1] == 't' && cmd->argv[1][2] == 'a' &&
        cmd->argv[1][3] == 'r' && cmd->argv[1][4] == 't' && cmd->argv[1][5] == '\0') {
        return game_start_match(cmd);
    }

    if (cmd->argv[1][0] == 's' && cmd->argv[1][1] == 't' && cmd->argv[1][2] == 'o' &&
        cmd->argv[1][3] == 'p' && cmd->argv[1][4] == '\0') {
        if (g_game_state == GAME_STATE_OFF) {
            game_log_line("game: already stopped");
            return 2;
        }
        if (cmd->argc >= 3 &&
            cmd->argv[2][0] == 'f' && cmd->argv[2][1] == 'o' && cmd->argv[2][2] == 'r' &&
            cmd->argv[2][3] == 'c' && cmd->argv[2][4] == 'e' && cmd->argv[2][5] == '\0') {
            game_force_stop_now();
            game_log_line("game: force-stopped");
            return 2;
        }
        g_game_state = GAME_STATE_STOPPING;
        game_set_hint("Stopping");
        game_log_line("game: stopping");
        return 2;
    }

    if (cmd->argv[1][0] == 's' && cmd->argv[1][1] == 't' && cmd->argv[1][2] == 'a' &&
        cmd->argv[1][3] == 't' && cmd->argv[1][4] == 'u' && cmd->argv[1][5] == 's' &&
        cmd->argv[1][6] == '\0') {
        return game_print_status();
    }

    game_log_line("game: unknown subcommand");
    return 2;
}

void game_on_tick(uint64_t now_us) {
    (void)now_us;

    if (g_game_state == GAME_STATE_OFF) return;

    game_consume_events();

    if (g_game_state == GAME_STATE_STOPPING) {
        if (!game_any_active_goto()) {
            game_force_stop_now();
            game_log_line("game: stopped");
        }
        return;
    }

    if (g_game_state == GAME_STATE_STARTING) {
        int any_unstarted = 0;
        int all_stopped = 1;

        for (int i = 0; i < GAME_ROLES; i++) {
            if (!g_slots[i].started) {
                any_unstarted = 1;
                all_stopped = 0;
                continue;
            }
            if (!game_train_is_position_confirmed(g_slots[i].train_num)) {
                return;
            }
            if (!game_train_is_known_stopped(g_slots[i].train_num)) {
                all_stopped = 0;
            }
        }

        if (any_unstarted) {
            (void)game_start_next_unstarted();
            return;
        }
        if (!all_stopped) return;

        game_reset_round_state();
        game_begin_wait_pick();
        return;
    }

    if (g_game_state != GAME_STATE_ROUND_RUNNING) return;

    game_try_resolve_neutral_cases();
    if (game_round_done()) {
        game_advance_round_or_finish();
    }
}

int game_is_active(void) {
    return g_game_state != GAME_STATE_OFF;
}

void game_get_ui_summary(game_ui_summary_t *out, uint64_t now_us) {
    (void)now_us;
    if (!out) return;

    out->active = (g_game_state != GAME_STATE_OFF);
    out->state_name = game_state_name(g_game_state);
    out->seed = g_game_seed;
    out->round_num = (g_round_index >= 0) ? (g_round_index + 1) : 0;
    out->priority_name = game_role_name(g_round_priority);
    out->human_train = g_slots[GAME_ROLE_HUMAN].train_num;
    out->ai_train = g_slots[GAME_ROLE_AI].train_num;
    out->neutral_train = g_slots[GAME_ROLE_NEUTRAL].train_num;
    out->human_score_half = g_score_half[GAME_ROLE_HUMAN];
    out->ai_score_half = g_score_half[GAME_ROLE_AI];
    out->reveal_targets = (g_game_state == GAME_STATE_ROUND_RUNNING ||
                           g_game_state == GAME_STATE_MATCH_OVER ||
                           g_game_state == GAME_STATE_STOPPING);
    out->human_target_name = g_slots[GAME_ROLE_HUMAN].target && g_slots[GAME_ROLE_HUMAN].target->name
                                 ? g_slots[GAME_ROLE_HUMAN].target->name : "-";
    out->ai_target_name = g_slots[GAME_ROLE_AI].target && g_slots[GAME_ROLE_AI].target->name
                              ? g_slots[GAME_ROLE_AI].target->name : "-";
    out->neutral_target_name = g_slots[GAME_ROLE_NEUTRAL].target && g_slots[GAME_ROLE_NEUTRAL].target->name
                                   ? g_slots[GAME_ROLE_NEUTRAL].target->name : "-";
    out->hint_text = g_hint_text;
    out->result_text = g_result_text;
}
