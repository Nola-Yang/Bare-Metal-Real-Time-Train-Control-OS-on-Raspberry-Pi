#ifndef _game_manager_h_
#define _game_manager_h_ 1

#include <stdint.h>
#include "runtime_protocol.h"
#include "train_tracking/track_node.h"

#define GAME_ROUNDS 2

typedef struct {
    int active;
    const char *state_name;
    uint32_t seed;
    int round_num;
    const char *priority_name;
    int human_train;
    int ai_train;
    int neutral_train;
    int human_score_half;
    int ai_score_half;
    int reveal_targets;
    const char *human_target_name;
    const char *ai_target_name;
    const char *neutral_target_name;
    const char *hint_text;
    const char *result_text;
} game_ui_summary_t;

void game_init(void);
int game_handle_command(const train_command_t *cmd);
void game_on_tick(uint64_t now_us);
int game_is_active(void);
int game_is_setup_active(void);
void game_get_ui_summary(game_ui_summary_t *out, uint64_t now_us);
int game_deadlock_mode_active(void);
int game_deadlock_victim_rank(int train_num);
track_node *game_deadlock_preferred_yield_target(int train_num);
int game_deadlock_should_resume_after_yield(int train_num);
void game_note_neutral_deadlock_yield(int train_num, track_node *blocked_target,
                                      track_node *yield_target);
int game_deadlock_handle_no_solution(const int *cycle_trains, int cycle_count,
                                     int victim_train, track_node *blocked_target);

#endif /* _game_manager_h_ */
