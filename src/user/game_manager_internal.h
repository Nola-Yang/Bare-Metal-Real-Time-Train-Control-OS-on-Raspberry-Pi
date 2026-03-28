#ifndef GAME_MANAGER_INTERNAL_H
#define GAME_MANAGER_INTERNAL_H 1

#include "game_manager.h"
#include "server/nameserver.h"
#include "server/position_server.h"
#include "timer.h"
#include "track.h"
#include "train_tracking/position.h"
#include "train_tracking/speed_table.h"
#include "ui.h"
#include "util.h"
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
    GAME_STATE_SETUP = 6,
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
    track_node *reported_detour_target;
} game_role_slot_t;

typedef struct {
    game_state_t state;
    uint32_t seed;
    uint32_t rng_state;
    uint64_t start_us;
    int position_server_tid;
    int setup_step;
    int setup_pending_role;
    int setup_trains[GAME_ROLES];
    game_role_slot_t slots[GAME_ROLES];
    int round_index;
    game_role_t round1_priority;
    game_role_t round_priority;
    uint8_t claim_mask[GAME_SENSOR_KEYS];
    int score_half[GAME_SCORE_PLAYERS];
    uint8_t neutral_used[GAME_SENSOR_KEYS];
    uint32_t event_seq;
    track_node *sensor_pool[TRACK_MAX];
    int sensor_pool_count;
    char hint_text[80];
    char result_text[96];
    pos_target_query_t game_query_primary;
    pos_target_query_t game_query_secondary;
    track_node *game_eligible_targets[TRACK_MAX];
    pos_game_event_t game_event_batch[GAME_EVENT_BATCH];
} game_context_t;

extern game_context_t g_game;

int game_position_server_tid(game_context_t *ctx);
const char *game_state_name(game_state_t state);
const char *game_role_name(game_role_t role);
void game_set_hint(game_context_t *ctx, const char *text);
void game_set_result(game_context_t *ctx, const char *text);
void game_log_line(const char *text);
void game_log_targets(game_context_t *ctx);
void game_log_round_result(game_context_t *ctx);
void game_reset_all(game_context_t *ctx);
uint32_t game_rand_u32(game_context_t *ctx);
void game_seed_rng(game_context_t *ctx, uint32_t seed);
int game_train_is_known_stopped(int train_num);
int game_train_is_position_confirmed(int train_num);
int game_any_active_goto(game_context_t *ctx);

void game_reset_round_state(game_context_t *ctx);
void game_begin_wait_pick(game_context_t *ctx);
int game_start_next_unstarted(game_context_t *ctx);
void game_force_stop_now(game_context_t *ctx);
void game_advance_round_or_finish(game_context_t *ctx);
int game_round_done(game_context_t *ctx);
void game_consume_events(game_context_t *ctx);
void game_sync_slot_completion_from_position(game_context_t *ctx, game_role_t role);
void game_try_resolve_neutral_cases(game_context_t *ctx);
int game_handle_pick(game_context_t *ctx, const train_command_t *cmd);

void game_setup_print_prompt(game_context_t *ctx);
void game_setup_log_role_line(game_role_t role, int train_num, const char *suffix);
int game_launch_match(game_context_t *ctx, const int trains[GAME_ROLES], int seed_override);
int game_start_interactive_setup(game_context_t *ctx);
int game_handle_setup_input(game_context_t *ctx, const train_command_t *cmd);

#endif /* GAME_MANAGER_INTERNAL_H */
