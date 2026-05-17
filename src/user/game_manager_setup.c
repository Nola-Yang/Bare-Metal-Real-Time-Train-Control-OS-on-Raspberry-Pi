#include "game_manager_internal.h"
#include "demo_manager.h"

static int game_build_sensor_pool(game_context_t *ctx) {
    ctx->sensor_pool_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type != NODE_SENSOR) continue;
        if (!g_track[i].name) continue;
        if (g_track[i].num < 0 || g_track[i].num >= GAME_SENSOR_KEYS) continue;
        ctx->sensor_pool[ctx->sensor_pool_count++] = &g_track[i];
    }
    return ctx->sensor_pool_count;
}

void game_setup_print_prompt(game_context_t *ctx) {
    static const char *labels[GAME_ROLES] = {"Human", "AI", "Neutral"};
    if (ctx->setup_step < GAME_ROLES) {
        ui_cmd_clear_line();
        ui_cmd_puts("Game setup: enter ");
        ui_cmd_puts(labels[ctx->setup_step]);
        ui_cmd_puts(" train number (valid: 13 14 15 17 18 55):\r\n");
    }
}

void game_setup_log_role_line(game_role_t role, int train_num, const char *suffix) {
    char buf[80];
    char *p = buf;
    char *end = buf + sizeof(buf) - 1;

    p = buf_append_cap(p, end, "game setup: ");
    p = buf_append_cap(p, end, game_role_name(role));
    p = buf_append_cap(p, end, " train ");
    p = buf_append_int_cap(p, end, train_num);
    if (suffix && suffix[0]) {
        p = buf_append_cap(p, end, " ");
        p = buf_append_cap(p, end, suffix);
    }
    *p = '\0';
    game_log_line(buf);
}

static int game_train_is_setup_ready(int train_num) {
    train_pos_t *pos = pos_get(train_num);

    if (pos_is_train_goto_active(train_num)) {
        game_log_line("game setup: train busy with active goto");
        return 0;
    }
    if (pos && pos->route_state != TRAIN_STATE_STOPPED &&
        pos->route_state != TRAIN_STATE_UNKNOWN &&
        pos->route_state != TRAIN_STATE_DEAD_TRACK) {
        game_log_line("game setup: train must be stopped or unknown");
        return 0;
    }
    return 1;
}

int game_launch_match(game_context_t *ctx, const int trains[GAME_ROLES], int seed_override) {
    int launch_trains[GAME_ROLES];

    for (int i = 0; i < GAME_ROLES; i++) {
        launch_trains[i] = trains[i];
    }

    for (int i = 0; i < GAME_ROLES; i++) {
        train_pos_t *pos = pos_get(launch_trains[i]);
        if (pos_is_train_goto_active(launch_trains[i])) {
            game_log_line("game: train busy with active goto");
            return 2;
        }
        if (pos && pos->route_state != TRAIN_STATE_STOPPED &&
            pos->route_state != TRAIN_STATE_UNKNOWN &&
            pos->route_state != TRAIN_STATE_DEAD_TRACK) {
            game_log_line("game: train must be stopped or unknown");
            return 2;
        }
    }

    game_reset_all(ctx);
    game_build_sensor_pool(ctx);
    game_seed_rng(ctx, (seed_override >= 0) ? (uint32_t)seed_override : (uint32_t)read_timer());
    ctx->slots[GAME_ROLE_HUMAN].train_num = launch_trains[0];
    ctx->slots[GAME_ROLE_AI].train_num = launch_trains[1];
    ctx->slots[GAME_ROLE_NEUTRAL].train_num = launch_trains[2];
    ctx->round1_priority = (game_rand_u32(ctx) & 1u) ? GAME_ROLE_AI : GAME_ROLE_HUMAN;
    ctx->round_index = 0;
    ctx->state = GAME_STATE_WAIT_PICK;
    ctx->start_us = read_timer();
    game_set_hint(ctx, "Preparing round 1");
    game_set_result(ctx, "-");
    ui_set_cmd_prompt_label("game> ");
    ui_mark_position_dirty();
    game_reset_round_state(ctx);
    game_log_line("game: setup complete");
    game_begin_wait_pick(ctx);
    return 2;
}

int game_start_interactive_setup(game_context_t *ctx) {
    if (demo_is_active()) {
        game_log_line("game: demo mode is active");
        return 2;
    }
    if (ctx->state != GAME_STATE_OFF) {
        game_log_line("game: already running");
        return 2;
    }
    game_reset_all(ctx);
    ctx->setup_step = 0;
    ctx->setup_pending_role = -1;
    ctx->setup_trains[0] = -1;
    ctx->setup_trains[1] = -1;
    ctx->setup_trains[2] = -1;
    ctx->state = GAME_STATE_SETUP;
    ui_set_cmd_prompt_label("game> ");
    game_setup_print_prompt(ctx);
    return 2;
}

int game_handle_setup_input(game_context_t *ctx, const train_command_t *cmd) {
    int train_num;
    game_role_t role;
    game_role_slot_t *slot;
    int pos_tid;

    if (ctx->state != GAME_STATE_SETUP) return 2;

    if (ctx->setup_pending_role >= 0 && ctx->setup_pending_role < GAME_ROLES) {
        game_setup_log_role_line((game_role_t)ctx->setup_pending_role,
                                 ctx->setup_trains[ctx->setup_pending_role],
                                 "is still finding position");
        return 2;
    }

    /* Accept a bare train number entered directly (TRAIN_CMD_UNKNOWN, argc==1). */
    const char *tok = NULL;
    if (cmd->type == TRAIN_CMD_UNKNOWN && cmd->argc == 1) {
        tok = cmd->argv[0];
    } else {
        return 2;
    }

    if (!str_parse_int(tok, &train_num) || !track_is_valid_train(train_num)) {
        game_log_line("game setup: invalid train number (valid: 13 14 15 17 18 55)");
        game_setup_print_prompt(ctx);
        return 2;
    }

    for (int i = 0; i < ctx->setup_step; i++) {
        if (ctx->setup_trains[i] == train_num) {
            game_log_line("game setup: duplicate train number");
            game_setup_print_prompt(ctx);
            return 2;
        }
    }

    if (!game_train_is_setup_ready(train_num)) {
        game_setup_print_prompt(ctx);
        return 2;
    }

    role = (game_role_t)ctx->setup_step;
    slot = &ctx->slots[role];
    ctx->setup_trains[ctx->setup_step] = train_num;
    slot->train_num = train_num;
    slot->started = 1;
    ui_mark_position_dirty();

    if (game_train_is_known_stopped(train_num)) {
        game_setup_log_role_line(role, train_num, "ready");
        ctx->setup_step++;
        if (ctx->setup_step < GAME_ROLES) {
            ui_set_cmd_prompt_label("game> ");
            game_setup_print_prompt(ctx);
            return 2;
        }
        return game_launch_match(ctx, ctx->setup_trains, -1);
    }

    pos_tid = game_position_server_tid(ctx);
    if (pos_tid < 0 || !PositionServerStartFindPos(pos_tid, train_num)) {
        slot->train_num = -1;
        slot->started = 0;
        ctx->setup_trains[ctx->setup_step] = -1;
        game_log_line("game setup: failed to start findpos");
        game_setup_print_prompt(ctx);
        return 2;
    }

    ctx->setup_pending_role = ctx->setup_step;
    ui_set_cmd_prompt_label("wait> ");
    game_setup_log_role_line(role, train_num, "finding position");
    return 2;
}
