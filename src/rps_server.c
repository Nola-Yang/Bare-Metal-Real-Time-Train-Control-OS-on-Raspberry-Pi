#include "rps_server.h"
#include "nameserver.h"
#include "syscall.h"
#include "uart.h"

#define MAX_PLAYERS 16
#define MAX_GAMES   8


// Player state 
typedef struct {
    int tid;
    int ind;
    int in_game;
    int partner_tid;
    int partner_ind;
    int choice;        
    int has_played;    // Has made a play this round
    int opponent_quit_pending;
} Player;

static Player Players[MAX_PLAYERS];
static int Waiting_Tid = -1;  // Tid of player waiting for opponent
static int Waiting_Ind = -1;


static void clear_waiting() {
    Waiting_Ind = -1;
    Waiting_Tid = -1;
}

const char *rps_choice_to_str(int choice) {
    switch (choice) {
        case RPS_ROCK: return "Rock";
        case RPS_PAPER: return "Paper";
        case RPS_SCISSORS: return "Scissors";
        default: return "Unknown";
    }
}

const char *rps_result_to_str(int result) {
    switch (result) {
        case RPS_RESULT_WIN: return "Win";
        case RPS_RESULT_LOSE: return "Lose";
        case RPS_RESULT_TIE: return "Tie";
        case RPS_RESULT_OPPONENT_QUIT: return "Opponent Quit";
        default: return "Unknown";
    }
}

static int find_player_ind(int tid) {
    for (int i = 0; i < MAX_PLAYERS; i++) {
        if (Players[i].tid == tid) return i;
    }
    return -1;
}

static int add_player(int tid) {
    Player *player;

    for (int i = 0; i < MAX_PLAYERS; i++) {
        player = &(Players[i]);
        if (player->tid > 0) continue;

        player->tid = tid;
        player->ind = i;
        player->in_game = 0;
        player->partner_tid = -1;
        player->partner_ind = -1;
        player->has_played = 0;
        player->opponent_quit_pending = 0;

        return i;
    }
    return -1;
}

static void remove_player_by_ind(int ind) {
    if (ind < 0) return;

    Player *player = &(Players[ind]);
    player->tid = 0;
    player->in_game = 0;
}

// Determine winner: returns result for player1
// 0=tie, 1=player1 wins, 2=player2 wins
static int determine_winner(int choice1, int choice2) {
    if (choice1 == choice2) return RPS_RESULT_TIE;

    if ((choice1 == RPS_ROCK && choice2 == RPS_SCISSORS) ||
        (choice1 == RPS_PAPER && choice2 == RPS_ROCK) ||
        (choice1 == RPS_SCISSORS && choice2 == RPS_PAPER)) {
        return RPS_RESULT_WIN;
    }
    return RPS_RESULT_LOSE;
}

void rps_server_task(void) {
    for (int i = 0; i < MAX_PLAYERS; i++) {
        Players[i].tid = 0;
    }

    RegisterAs(RPS_SERVER_NAME);
    uart_printf(CONSOLE, "RPS Server: Started and registered\r\n");

    int sender_tid;
    RpsRequest req;
    RpsResponse resp;

    while (1) {
        int len = Receive(&sender_tid, (char *)&req, sizeof(RpsRequest));
        if (len < 0) continue;

        resp.status = RPS_OK;
        resp.result = 0;
        resp.opponent_choice = -1;

        switch (req.type) {
            case RPS_SIGNUP:
                uart_printf(CONSOLE, "RPS Server: Player %d signing up\r\n", sender_tid);
                int sender_ind = find_player_ind(sender_tid);

                if (sender_ind < 0) {
                    sender_ind = add_player(sender_tid);

                    if (sender_ind < 0) {
                        resp.status = RPS_ERROR;
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                        break;
                    }
                }

                if (Waiting_Tid < 0) {
                    Waiting_Tid = sender_tid;
                    Waiting_Ind = sender_ind;
                    uart_printf(CONSOLE, "RPS Server: Player %d waiting for opponent\r\n", sender_tid);
                } else {
                    // Match with waiting player
                    int p1_ind = Waiting_Ind;
                    int p2_ind = sender_ind;

                    if (p1_ind >= 0 && p2_ind >= 0) {
                        Player *player1 = &(Players[p1_ind]);
                        Player *player2 = &(Players[p2_ind]);

                        player1->in_game = 1;
                        player1->partner_tid = sender_tid;
                        player1->partner_ind = p2_ind;

                        player2->in_game = 1;
                        player2->partner_tid = Waiting_Tid;
                        player2->partner_ind = p1_ind;

                        uart_printf(CONSOLE, "RPS Server: Matched players %d and %d\r\n", Waiting_Tid, sender_tid);

                        // Reply to both players
                        resp.status = RPS_GAME_START;
                        Reply(Waiting_Tid, (const char *)&resp, sizeof(RpsResponse));
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                    }
                    
                    clear_waiting();
                }
                break;

            case RPS_PLAY:
                {
                    int player_ind = find_player_ind(sender_tid);
                    Player *player;

                    if (player_ind < 0) {
                        uart_printf(CONSOLE, "RPS Server: Player %d not in game\r\n", sender_tid);
                        resp.status = RPS_ERROR;
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                        break;
                    }

                    player = &(Players[player_ind]);

                    if (player->opponent_quit_pending) {
                        resp.status = RPS_OK;
                        resp.result = RPS_RESULT_OPPONENT_QUIT;
                        resp.opponent_choice = -1;
                        player->in_game = 0;
                        player->partner_tid = -1;
                        player->opponent_quit_pending = 0;
                        uart_printf(CONSOLE, "RPS Server: Player %d's opponent had quit\r\n", sender_tid);
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                        break;
                    }

                    if (!player->in_game) {
                        uart_printf(CONSOLE, "RPS Server: Player %d not in game\r\n", sender_tid);
                        resp.status = RPS_ERROR;
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                        break;
                    }

                    int partner_ind = player->partner_ind;

                    if (partner_ind < 0) {
                        resp.status = RPS_OK;
                        resp.result = RPS_RESULT_OPPONENT_QUIT;
                        resp.opponent_choice = -1;

                        player->in_game = 0;
                        player->partner_tid = -1;
                        uart_printf(CONSOLE, "RPS Server: Player %d's opponent had quit\r\n", sender_tid);
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                        break;
                    }
                    
                    Player *partner = &(Players[partner_ind]);
                    player->choice = req.choice;
                    player->has_played = 1;

                    uart_printf(CONSOLE, "RPS Server: Player %d plays %s\r\n", sender_tid, rps_choice_to_str(req.choice));

                    if (!(partner->has_played)) break;

                    int result = determine_winner(player->choice, partner->choice);

                    resp.status = RPS_OK;
                    resp.result = result;
                    resp.opponent_choice = Players[partner_ind].choice;
                    Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));

                    RpsResponse partner_resp;
                    partner_resp.status = RPS_OK;

                    if (result == RPS_RESULT_WIN) {
                        partner_resp.result = RPS_RESULT_LOSE;
                    } else if (result == RPS_RESULT_LOSE) {
                        partner_resp.result = RPS_RESULT_WIN;
                    } else {
                        partner_resp.result = RPS_RESULT_TIE;
                    }

                    partner_resp.opponent_choice = player->choice;
                    Reply(Players[player_ind].partner_tid, (const char *)&partner_resp, sizeof(RpsResponse));

                    uart_printf(CONSOLE, "RPS Server: Round result - P%d %s vs P%d %s\r\n",
                                sender_tid, rps_choice_to_str(Players[player_ind].choice),
                                Players[player_ind].partner_tid, rps_choice_to_str(Players[partner_ind].choice));

                    player->has_played = 0;
                    player->choice = -1;
                    partner->has_played = 0;
                    partner->choice = -1;
                }
                break;

            case RPS_QUIT:
                {
                    uart_printf(CONSOLE, "RPS Server: Player %d quitting\r\n", sender_tid);

                    int player_ind = find_player_ind(sender_tid);
                    Player *player;

                    if (player_ind >= 0 && Players[player_ind].in_game) {
                        player = &(Players[player_ind]);
                        int partner_ind = player->partner_ind;

                        if (partner_ind >= 0) {
                            Player *partner = &(Players[partner_ind]);

                            if (partner->has_played) {
                                RpsResponse partner_resp;
                                partner_resp.status = RPS_OK;
                                partner_resp.result = RPS_RESULT_OPPONENT_QUIT;
                                partner_resp.opponent_choice = -1;
                                partner->opponent_quit_pending = 1;

                                Reply(player->partner_tid, (const char *)&partner_resp, sizeof(RpsResponse));

                                partner->has_played = 0;
                                partner->choice = -1;
                                partner->in_game = 0;
                                partner->partner_tid = -1;
                                partner->opponent_quit_pending = 0;
                            } else {
                                partner->opponent_quit_pending = 1;
                                partner->in_game = 0;
                                partner->partner_tid = -1;
                            }
                        }
                    }

                    // Clear waiting if this player was waiting
                    if (Waiting_Tid == sender_tid) {
                        clear_waiting();
                    }

                    remove_player_by_ind(player_ind);
                    resp.status = RPS_OK;
                    Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                }
                break;

            default:
                resp.status = RPS_ERROR;
                Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                break;
        }
    }
}
