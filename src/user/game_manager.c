#include "game_manager_internal.h"
#include "train_tracking/position_priv.h"

game_context_t g_game = {
    .seed = 1,
    .rng_state = 1,
    .hint_text = "-",
    .result_text = "-",
};

int game_position_server_tid(game_context_t *ctx) {
    if (ctx->position_server_tid < 0) {
        ctx->position_server_tid = WhoIs(POSITION_SERVER_NAME);
    }
    return ctx->position_server_tid;
}

const char *game_state_name(game_state_t state) {
    switch (state) {
    case GAME_STATE_OFF: return "OFF";
    case GAME_STATE_STARTING: return "STARTING";
    case GAME_STATE_WAIT_PICK: return "WAIT_PICK";
    case GAME_STATE_ROUND_RUNNING: return "ROUND_RUNNING";
    case GAME_STATE_MATCH_OVER: return "MATCH_OVER";
    case GAME_STATE_STOPPING: return "STOPPING";
    case GAME_STATE_SETUP: return "SETUP";
    default: return "UNK";
    }
}

const char *game_role_name(game_role_t role) {
    switch (role) {
    case GAME_ROLE_HUMAN: return "Human";
    case GAME_ROLE_AI: return "AI";
    case GAME_ROLE_NEUTRAL: return "Neutral";
    default: return "-";
    }
}

void game_set_hint(game_context_t *ctx, const char *text) {
    int i = 0;
    if (!text || !text[0]) text = "-";
    while (text[i] && i + 1 < (int)sizeof(ctx->hint_text)) {
        ctx->hint_text[i] = text[i];
        i++;
    }
    ctx->hint_text[i] = '\0';
}

void game_set_result(game_context_t *ctx, const char *text) {
    int i = 0;
    if (!text || !text[0]) text = "-";
    while (text[i] && i + 1 < (int)sizeof(ctx->result_text)) {
        ctx->result_text[i] = text[i];
        i++;
    }
    ctx->result_text[i] = '\0';
}

void game_log_line(const char *text) {
    if (!text) return;
    ui_cmd_log_line(text);
}

static int game_stop_force_arg(const char *arg) {
    return arg != NULL && (str_eq(arg, "force") || str_eq(arg, "foce"));
}

void game_log_targets(game_context_t *ctx) {
    char buf[96];
    char *p = buf;
    char *end = buf + sizeof(buf) - 1;
    game_role_slot_t *human = &ctx->slots[GAME_ROLE_HUMAN];
    game_role_slot_t *ai = &ctx->slots[GAME_ROLE_AI];
    game_role_slot_t *neutral = &ctx->slots[GAME_ROLE_NEUTRAL];

    p = buf_append_cap(p, end, "game: reveal H=");
    p = buf_append_cap(p, end, human->target && human->target->name ? human->target->name : "-");
    p = buf_append_cap(p, end, " AI=");
    p = buf_append_cap(p, end, ai->target && ai->target->name ? ai->target->name : "-");
    p = buf_append_cap(p, end, " N=");
    p = buf_append_cap(p, end, neutral->target && neutral->target->name ? neutral->target->name : "-");
    *p = '\0';
    game_log_line(buf);
}

void game_log_round_result(game_context_t *ctx) {
    char buf[64];
    char *p = buf;
    char *end = buf + sizeof(buf) - 1;

    p = buf_append_cap(p, end, "game: round ");
    p = buf_append_int_cap(p, end, ctx->round_index + 1);
    p = buf_append_cap(p, end, " end H=");
    p = buf_append_int_cap(p, end, ctx->score_half[GAME_ROLE_HUMAN]);
    p = buf_append_cap(p, end, "/2 AI=");
    p = buf_append_int_cap(p, end, ctx->score_half[GAME_ROLE_AI]);
    p = buf_append_cap(p, end, "/2");
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
    slot->reported_detour_target = NULL;
}

void game_reset_all(game_context_t *ctx) {
    for (int i = 0; i < GAME_ROLES; i++) {
        ctx->slots[i].train_num = -1;
        game_reset_slot(&ctx->slots[i]);
    }
    ctx->setup_step = 0;
    ctx->setup_pending_role = -1;
    for (int i = 0; i < GAME_ROLES; i++) {
        ctx->setup_trains[i] = -1;
    }
    for (int i = 0; i < GAME_SENSOR_KEYS; i++) {
        ctx->claim_mask[i] = 0;
        ctx->neutral_used[i] = 0;
    }
    for (int i = 0; i < GAME_SCORE_PLAYERS; i++) {
        ctx->score_half[i] = 0;
    }
    ctx->state = GAME_STATE_OFF;
    ctx->round_index = -1;
    ctx->round1_priority = GAME_ROLE_HUMAN;
    ctx->round_priority = GAME_ROLE_HUMAN;
    ctx->start_us = 0;
    ctx->event_seq = 0;
    ctx->position_server_tid = -1;
    game_set_hint(ctx, "-");
    game_set_result(ctx, "-");
    ui_set_cmd_prompt_label("cmd> ");
}

uint32_t game_rand_u32(game_context_t *ctx) {
    uint32_t x = ctx->rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    ctx->rng_state = (x == 0) ? 1U : x;
    return ctx->rng_state;
}

void game_seed_rng(game_context_t *ctx, uint32_t seed) {
    ctx->seed = (seed == 0) ? 1U : seed;
    ctx->rng_state = ctx->seed;
}

int game_train_is_known_stopped(int train_num) {
    train_pos_t *pos = pos_get(train_num);
    return pos && pos->cur_sensor != NULL && pos->route_state == TRAIN_STATE_STOPPED;
}

int game_train_is_position_confirmed(int train_num) {
    train_pos_t *pos = pos_get(train_num);
    if (!pos || pos->cur_sensor == NULL) return 0;
    return pos->route_state != TRAIN_STATE_UNKNOWN &&
           pos->route_state != TRAIN_STATE_FIND_POS &&
           pos->route_state != TRAIN_STATE_STOPPING_GOTO;
}

int game_any_active_goto(game_context_t *ctx) {
    for (int i = 0; i < GAME_ROLES; i++) {
        if (ctx->slots[i].train_num < 0) continue;
        if (pos_is_train_goto_active(ctx->slots[i].train_num)) return 1;
    }
    return 0;
}

static int game_role_index_from_train_num(game_context_t *ctx, int train_num) {
    if (!ctx) return -1;
    for (int i = 0; i < GAME_ROLES; i++) {
        if (ctx->slots[i].train_num == train_num) return i;
    }
    return -1;
}

static int game_current_sensor_num_internal(int train_num) {
    train_pos_t *pos = pos_get(train_num);
    if (!pos || !pos->cur_sensor) return -1;
    return pos->cur_sensor->num;
}

static track_node *game_find_neutral_standby_target(game_context_t *ctx) {
    game_role_slot_t *slot;
    pos_target_query_t *query;
    track_node *best = NULL;
    int best_dist = 0x7fffffff;

    if (!ctx) return NULL;

    slot = &ctx->slots[GAME_ROLE_NEUTRAL];
    query = &ctx->game_query_primary;
    if (slot->train_num < 0) return NULL;

    for (int i = 0; i < ctx->sensor_pool_count; i++) {
        track_node *cand = ctx->sensor_pool[i];

        if (!cand) continue;
        if (game_current_sensor_num_internal(slot->train_num) == cand->num) continue;
        if (slot->target && slot->target->num == cand->num) continue;
        if (pos_query_target(slot->train_num, cand, query) != POS_TARGET_READY) continue;
        if (query->plan.total_dist_mm < best_dist) {
            best = cand;
            best_dist = query->plan.total_dist_mm;
        }
    }

    return best;
}

int game_deadlock_mode_active(void) {
    return g_game.state == GAME_STATE_ROUND_RUNNING;
}

int game_deadlock_victim_rank(int train_num) {
    game_context_t *ctx = &g_game;
    int role;

    if (!game_deadlock_mode_active()) return -1;

    role = game_role_index_from_train_num(ctx, train_num);
    if (role < 0) return -1;
    if (role == GAME_ROLE_NEUTRAL) return 0;
    if (ctx->round_priority != GAME_ROLE_HUMAN &&
        ctx->round_priority != GAME_ROLE_AI) {
        return 1;
    }
    return (role == (int)ctx->round_priority) ? 2 : 1;
}

track_node *game_deadlock_preferred_yield_target(int train_num) {
    game_context_t *ctx = &g_game;
    game_role_slot_t *slot;

    if (!game_deadlock_mode_active()) return NULL;
    if (game_role_index_from_train_num(ctx, train_num) != GAME_ROLE_NEUTRAL) {
        return NULL;
    }

    slot = &ctx->slots[GAME_ROLE_NEUTRAL];
    if (slot->standby_target != NULL && !slot->standby_completed) {
        return slot->standby_target;
    }

    return game_find_neutral_standby_target(ctx);
}

int game_deadlock_handle_no_solution(const int *cycle_trains, int cycle_count,
                                     int victim_train, track_node *blocked_target) {
    (void)cycle_trains;
    (void)cycle_count;
    (void)victim_train;
    (void)blocked_target;
    return 0;
}

void game_init(void) {
    game_reset_all(&g_game);
}

static int game_print_status(game_context_t *ctx) {
    char buf[80];
    char *p = buf;
    char *end = buf + sizeof(buf) - 1;

    p = buf_append_cap(p, end, "game: state=");
    p = buf_append_cap(p, end, game_state_name(ctx->state));
    if (ctx->round_index >= 0) {
        p = buf_append_cap(p, end, " round=");
        p = buf_append_int_cap(p, end, ctx->round_index + 1);
    }
    p = buf_append_cap(p, end, " H=");
    p = buf_append_int_cap(p, end, ctx->score_half[GAME_ROLE_HUMAN]);
    p = buf_append_cap(p, end, "/2 AI=");
    p = buf_append_int_cap(p, end, ctx->score_half[GAME_ROLE_AI]);
    p = buf_append_cap(p, end, "/2");
    *p = '\0';
    game_log_line(buf);
    return 2;
}

int game_handle_command(const train_command_t *cmd) {
    game_context_t *ctx = &g_game;

    if (!cmd) return 2;

    if (cmd->type == TRAIN_CMD_PICK) {
        return game_handle_pick(ctx, cmd);
    }

    if (ctx->state == GAME_STATE_WAIT_PICK && cmd->type == TRAIN_CMD_UNKNOWN && cmd->argc == 1) {
        track_node *target = track_find_node(cmd->argv[0]);
        if (!target) {
            game_log_line("pick: unknown sensor");
            return 2;
        }
        train_command_t pick_cmd = *cmd;
        pick_cmd.type = TRAIN_CMD_PICK;
        pick_cmd.target_idx = (int)(target - g_track);
        return game_handle_pick(ctx, &pick_cmd);
    }

    if (cmd->type != TRAIN_CMD_GAME) {
        if (ctx->state == GAME_STATE_SETUP) {
            return game_handle_setup_input(ctx, cmd);
        }
        return 2;
    }

    if (cmd->argc == 1) {
        return game_start_interactive_setup(ctx);
    }

    if (cmd->argc < 2) {
        game_log_line("Usage: game [stop [force] | status]");
        return 2;
    }

    if (str_eq(cmd->argv[1], "stop")) {
        if (cmd->argc >= 3 && game_stop_force_arg(cmd->argv[2])) {
            if (ctx->state == GAME_STATE_OFF) {
                game_log_line("game: already stopped");
                return 2;
            }
            game_force_stop_now(ctx);
            game_log_line("game: force reset to startup state");
            return 2;
        }
        if (ctx->state == GAME_STATE_OFF) {
            game_log_line("game: already stopped");
            return 2;
        }
        ctx->state = GAME_STATE_STOPPING;
        game_set_hint(ctx, "Stopping");
        game_log_line("game: stopping");
        return 2;
    }

    if (str_eq(cmd->argv[1], "status")) {
        return game_print_status(ctx);
    }

    game_log_line("game: unknown subcommand");
    return 2;
}

void game_on_tick(uint64_t now_us) {
    game_context_t *ctx = &g_game;
    (void)now_us;

    if (ctx->state == GAME_STATE_OFF) return;

    if (ctx->state == GAME_STATE_SETUP) {
        if (ctx->setup_pending_role >= 0 && ctx->setup_pending_role < GAME_ROLES) {
            int train_num = ctx->setup_trains[ctx->setup_pending_role];

            if (!game_train_is_position_confirmed(train_num)) return;
            if (!game_train_is_known_stopped(train_num)) return;

            game_setup_log_role_line((game_role_t)ctx->setup_pending_role,
                                     train_num,
                                     "ready");
            ctx->setup_step = ctx->setup_pending_role + 1;
            ctx->setup_pending_role = -1;

            if (ctx->setup_step < GAME_ROLES) {
                ui_set_cmd_prompt_label("game> ");
                game_setup_print_prompt(ctx);
                ui_cmd_newprompt();
                ui_mark_position_dirty();
                return;
            }

            (void)game_launch_match(ctx, ctx->setup_trains, -1);
        }
        return;
    }

    game_consume_events(ctx);

    if (ctx->state == GAME_STATE_STOPPING) {
        if (!game_any_active_goto(ctx)) {
            game_force_stop_now(ctx);
            game_log_line("game: stopped");
        }
        return;
    }

    if (ctx->state == GAME_STATE_STARTING) {
        int any_unstarted = 0;
        int all_stopped = 1;

        for (int i = 0; i < GAME_ROLES; i++) {
            if (!ctx->slots[i].started) {
                any_unstarted = 1;
                all_stopped = 0;
                continue;
            }
            if (!game_train_is_position_confirmed(ctx->slots[i].train_num)) {
                return;
            }
            if (!game_train_is_known_stopped(ctx->slots[i].train_num)) {
                all_stopped = 0;
            }
        }

        if (any_unstarted) {
            (void)game_start_next_unstarted(ctx);
            return;
        }
        if (!all_stopped) return;

        game_reset_round_state(ctx);
        game_begin_wait_pick(ctx);
        return;
    }

    if (ctx->state != GAME_STATE_ROUND_RUNNING) return;

    game_sync_slot_completion_from_position(ctx, GAME_ROLE_HUMAN);
    game_sync_slot_completion_from_position(ctx, GAME_ROLE_AI);
    game_sync_slot_completion_from_position(ctx, GAME_ROLE_NEUTRAL);
    game_try_resolve_neutral_cases(ctx);
    game_sync_slot_completion_from_position(ctx, GAME_ROLE_NEUTRAL);
    if (game_round_done(ctx)) {
        game_advance_round_or_finish(ctx);
    }
}

int game_is_active(void) {
    return g_game.state != GAME_STATE_OFF;
}

int game_is_setup_active(void) {
    return g_game.state == GAME_STATE_SETUP;
}

void game_get_ui_summary(game_ui_summary_t *out, uint64_t now_us) {
    game_context_t *ctx = &g_game;

    (void)now_us;
    if (!out) return;

    out->active = (ctx->state != GAME_STATE_OFF);
    out->state_name = game_state_name(ctx->state);
    out->seed = ctx->seed;
    out->round_num = (ctx->round_index >= 0) ? (ctx->round_index + 1) : 0;
    out->priority_name = game_role_name(ctx->round_priority);
    out->human_train = ctx->slots[GAME_ROLE_HUMAN].train_num;
    out->ai_train = ctx->slots[GAME_ROLE_AI].train_num;
    out->neutral_train = ctx->slots[GAME_ROLE_NEUTRAL].train_num;
    out->human_score_half = ctx->score_half[GAME_ROLE_HUMAN];
    out->ai_score_half = ctx->score_half[GAME_ROLE_AI];
    out->reveal_targets = (ctx->state == GAME_STATE_ROUND_RUNNING ||
                           ctx->state == GAME_STATE_MATCH_OVER ||
                           ctx->state == GAME_STATE_STOPPING);
    out->human_target_name = ctx->slots[GAME_ROLE_HUMAN].target && ctx->slots[GAME_ROLE_HUMAN].target->name
                                 ? ctx->slots[GAME_ROLE_HUMAN].target->name : "-";
    out->ai_target_name = ctx->slots[GAME_ROLE_AI].target && ctx->slots[GAME_ROLE_AI].target->name
                              ? ctx->slots[GAME_ROLE_AI].target->name : "-";
    out->neutral_target_name = ctx->slots[GAME_ROLE_NEUTRAL].target && ctx->slots[GAME_ROLE_NEUTRAL].target->name
                                   ? ctx->slots[GAME_ROLE_NEUTRAL].target->name : "-";
    out->hint_text = ctx->hint_text;
    out->result_text = ctx->result_text;
}
