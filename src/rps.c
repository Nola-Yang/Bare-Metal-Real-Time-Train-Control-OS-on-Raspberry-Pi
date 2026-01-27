#include "rps.h"
#include "nameserver.h"
#include "syscall.h"
#include "uart.h"

#define MAX_PLAYERS 16
#define MAX_GAMES   8

// Player state 
typedef struct {
    int tid;
    int in_game;
    int partner_tid;
    int choice;        
    int has_played;    // Has made a play this round
    int opponent_quit_pending;
} Player;

// Game state
typedef struct {
    int player1_tid;
    int player2_tid;
    int active;
} Game;

static Player players[MAX_PLAYERS];
static Game games[MAX_GAMES];
static int waiting_tid = -1;  // Tid of player waiting for opponent

static const char *choice_to_string(int choice) {
    switch (choice) {
        case RPS_ROCK: return "Rock";
        case RPS_PAPER: return "Paper";
        case RPS_SCISSORS: return "Scissors";
        default: return "Unknown";
    }
}

static const char *result_to_string(int result) {
    switch (result) {
        case RPS_RESULT_WIN: return "Win";
        case RPS_RESULT_LOSE: return "Lose";
        case RPS_RESULT_TIE: return "Tie";
        case RPS_RESULT_OPPONENT_QUIT: return "Opponent Quit";
        default: return "Unknown";
    }
}

static int find_player(int tid) {
    for (int i = 0; i < MAX_PLAYERS; i++) {
        if (players[i].tid == tid) return i;
    }
    return -1;
}

static int add_player(int tid) {
    for (int i = 0; i < MAX_PLAYERS; i++) {
        if (players[i].tid == 0) {
            players[i].tid = tid;
            players[i].in_game = 0;
            players[i].partner_tid = -1;
            players[i].choice = -1;
            players[i].has_played = 0;
            players[i].opponent_quit_pending = 0;
            return i;
        }
    }
    return -1;
}

static void remove_player(int tid) {
    int idx = find_player(tid);
    if (idx >= 0) {
        players[idx].tid = 0;
        players[idx].in_game = 0;
    }
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
        players[i].tid = 0;
    }
    for (int i = 0; i < MAX_GAMES; i++) {
        games[i].active = 0;
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

                if (find_player(sender_tid) < 0) {
                    if (add_player(sender_tid) < 0) {
                        resp.status = RPS_ERROR;
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                        break;
                    }
                }

                if (waiting_tid < 0) {
                    waiting_tid = sender_tid;
                    uart_printf(CONSOLE, "RPS Server: Player %d waiting for opponent\r\n", sender_tid);
                } else {
                    // Match with waiting player
                    int p1 = find_player(waiting_tid);
                    int p2 = find_player(sender_tid);

                    if (p1 >= 0 && p2 >= 0) {
                        players[p1].in_game = 1;
                        players[p1].partner_tid = sender_tid;
                        players[p2].in_game = 1;
                        players[p2].partner_tid = waiting_tid;

                        uart_printf(CONSOLE, "RPS Server: Matched players %d and %d\r\n",
                                    waiting_tid, sender_tid);

                        // Reply to both players
                        resp.status = RPS_GAME_START;
                        Reply(waiting_tid, (const char *)&resp, sizeof(RpsResponse));
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                    }
                    waiting_tid = -1;
                }
                break;

            case RPS_PLAY:
                {
                    int p = find_player(sender_tid);
                    if (p < 0 || !players[p].in_game) {
                        uart_printf(CONSOLE, "RPS Server: Player %d not in game\r\n", sender_tid);
                        resp.status = RPS_ERROR;
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                        break;
                    }

                    int partner = find_player(players[p].partner_tid);
                    if (players[p].opponent_quit_pending || partner < 0) {
                        resp.status = RPS_OK;
                        resp.result = RPS_RESULT_OPPONENT_QUIT;
                        resp.opponent_choice = -1;
                        players[p].in_game = 0;
                        players[p].partner_tid = -1;
                        players[p].opponent_quit_pending = 0;
                        uart_printf(CONSOLE, "RPS Server: Player %d's opponent had quit\r\n", sender_tid);
                        Reply(sender_tid, (const char *)&resp, sizeof(RpsResponse));
                        break;
                    }

                    players[p].choice = req.choice;
                    players[p].has_played = 1;

                    uart_printf(CONSOLE, "RPS Server: Player %d plays %s\r\n",
                                sender_tid, choice_to_string(req.choice));

                    if (players[partner].has_played) {
                        int result = determine_winner(players[p].choice, players[partner].choice);

                        resp.status = RPS_OK;
                        resp.result = result;
                        resp.opponent_choice = players[partner].choice;
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
                        partner_resp.opponent_choice = players[p].choice;
                        Reply(players[p].partner_tid, (const char *)&partner_resp, sizeof(RpsResponse));

                        uart_printf(CONSOLE, "RPS Server: Round result - P%d %s vs P%d %s\r\n",
                                    sender_tid, choice_to_string(players[p].choice),
                                    players[p].partner_tid, choice_to_string(players[partner].choice));

                        players[p].has_played = 0;
                        players[p].choice = -1;
                        players[partner].has_played = 0;
                        players[partner].choice = -1;
                    }
                }
                break;

            case RPS_QUIT:
                {
                    uart_printf(CONSOLE, "RPS Server: Player %d quitting\r\n", sender_tid);

                    int p = find_player(sender_tid);
                    if (p >= 0 && players[p].in_game) {
                        int partner = find_player(players[p].partner_tid);
                        if (partner >= 0) {
                            if (players[partner].has_played) {
                                RpsResponse partner_resp;
                                partner_resp.status = RPS_OK;
                                partner_resp.result = RPS_RESULT_OPPONENT_QUIT;
                                partner_resp.opponent_choice = -1;
                                Reply(players[p].partner_tid, (const char *)&partner_resp,
                                      sizeof(RpsResponse));

                                players[partner].has_played = 0;
                                players[partner].choice = -1;
                                players[partner].in_game = 0;
                                players[partner].partner_tid = -1;
                                players[partner].opponent_quit_pending = 0;
                            } else {
                                players[partner].opponent_quit_pending = 1;
                                players[partner].in_game = 0;
                                players[partner].partner_tid = -1;
                            }
                        }
                    }

                    // Clear waiting if this player was waiting
                    if (waiting_tid == sender_tid) {
                        waiting_tid = -1;
                    }

                    remove_player(sender_tid);
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


void rps_client_task(void) {
    int my_tid = MyTid();

    int server_tid = WhoIs(RPS_SERVER_NAME);
    if (server_tid < 0) {
        uart_printf(CONSOLE, "Client %d: Failed to find RPS server\r\n", my_tid);
        Exit();
    }

    uart_printf(CONSOLE, "Client %d: Found RPS server at tid %d\r\n", my_tid, server_tid);

    RpsRequest req;
    RpsResponse resp;

    req.type = RPS_SIGNUP;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    if (resp.status == RPS_ERROR) {
        uart_printf(CONSOLE, "Client %d: Signup failed\r\n", my_tid);
        Exit();
    }

    uart_printf(CONSOLE, "Client %d: Signed up, game starting\r\n", my_tid);

    // Play a few rounds
    int choices[] = {RPS_ROCK, RPS_PAPER, RPS_SCISSORS, RPS_ROCK, RPS_SCISSORS};
    int num_rounds = 5;

    for (int round = 0; round < num_rounds; round++) {
        req.type = RPS_PLAY;
        req.choice = choices[round % 5];

        uart_printf(CONSOLE, "Client %d: Round %d - Playing %s\r\n",
                    my_tid, round + 1, choice_to_string(req.choice));

        Send(server_tid, (const char *)&req, sizeof(RpsRequest),
             (char *)&resp, sizeof(RpsResponse));

        if (resp.result == RPS_RESULT_OPPONENT_QUIT) {
            uart_printf(CONSOLE, "Client %d: Opponent quit!\r\n", my_tid);
            break;
        }

        uart_printf(CONSOLE, "Client %d: Round %d - Result: %s (opponent: %s)\r\n",
                    my_tid, round + 1, result_to_string(resp.result),
                    choice_to_string(resp.opponent_choice));
    }

    req.type = RPS_QUIT;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_printf(CONSOLE, "Client %d: Quit game, exiting\r\n", my_tid);
    Exit();
}

void rps_client_early_quit(void) {
    int my_tid = MyTid();

    int server_tid = WhoIs(RPS_SERVER_NAME);
    if (server_tid < 0) {
        uart_printf(CONSOLE, "EarlyQuit %d: Failed to find RPS server\r\n", my_tid);
        Exit();
    }

    uart_printf(CONSOLE, "EarlyQuit %d: Found RPS server\r\n", my_tid);

    RpsRequest req;
    RpsResponse resp;

    req.type = RPS_SIGNUP;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    if (resp.status == RPS_ERROR) {
        uart_printf(CONSOLE, "EarlyQuit %d: Signup failed\r\n", my_tid);
        Exit();
    }

    uart_printf(CONSOLE, "EarlyQuit %d: Signed up\r\n", my_tid);

    for (int round = 0; round < 2; round++) {
        req.type = RPS_PLAY;
        req.choice = RPS_ROCK;

        uart_printf(CONSOLE, "EarlyQuit %d: Round %d - Playing Rock\r\n", my_tid, round + 1);

        Send(server_tid, (const char *)&req, sizeof(RpsRequest),
             (char *)&resp, sizeof(RpsResponse));

        if (resp.result == RPS_RESULT_OPPONENT_QUIT) {
            uart_printf(CONSOLE, "EarlyQuit %d: Opponent quit first!\r\n", my_tid);
            break;
        }

        uart_printf(CONSOLE, "EarlyQuit %d: Round %d - Result: %s\r\n",
                    my_tid, round + 1, result_to_string(resp.result));
    }

    // Quit early
    req.type = RPS_QUIT;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_printf(CONSOLE, "EarlyQuit %d: Quit early, exiting\r\n", my_tid);
    Exit();
}

void rps_client_long_player(void) {
    int my_tid = MyTid();

    int server_tid = WhoIs(RPS_SERVER_NAME);
    if (server_tid < 0) {
        uart_printf(CONSOLE, "LongPlayer %d: Failed to find RPS server\r\n", my_tid);
        Exit();
    }

    uart_printf(CONSOLE, "LongPlayer %d: Found RPS server\r\n", my_tid);

    RpsRequest req;
    RpsResponse resp;

    // Sign up
    req.type = RPS_SIGNUP;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    if (resp.status == RPS_ERROR) {
        uart_printf(CONSOLE, "LongPlayer %d: Signup failed\r\n", my_tid);
        Exit();
    }

    uart_printf(CONSOLE, "LongPlayer %d: Signed up\r\n", my_tid);

    // Try to play 10 rounds
    int choices[] = {RPS_PAPER, RPS_SCISSORS, RPS_ROCK};
    for (int round = 0; round < 10; round++) {
        req.type = RPS_PLAY;
        req.choice = choices[round % 3];

        uart_printf(CONSOLE, "LongPlayer %d: Round %d - Playing %s\r\n",
                    my_tid, round + 1, choice_to_string(req.choice));

        Send(server_tid, (const char *)&req, sizeof(RpsRequest),
             (char *)&resp, sizeof(RpsResponse));

        if (resp.result == RPS_RESULT_OPPONENT_QUIT) {
            uart_printf(CONSOLE, "LongPlayer %d: Opponent quit!\r\n", my_tid);
            break;
        }

        uart_printf(CONSOLE, "LongPlayer %d: Round %d - Result: %s\r\n",
                    my_tid, round + 1, result_to_string(resp.result));
    }

    // Quit
    req.type = RPS_QUIT;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_printf(CONSOLE, "LongPlayer %d: Quit game, exiting\r\n", my_tid);
    Exit();
}

void rps_client_immediate_quit(void) {
    int my_tid = MyTid();

    int server_tid = WhoIs(RPS_SERVER_NAME);
    if (server_tid < 0) {
        uart_printf(CONSOLE, "ImmediateQuit %d: Failed to find RPS server\r\n", my_tid);
        Exit();
    }

    uart_printf(CONSOLE, "ImmediateQuit %d: Found RPS server\r\n", my_tid);

    RpsRequest req;
    RpsResponse resp;

    // Sign up
    req.type = RPS_SIGNUP;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_printf(CONSOLE, "ImmediateQuit %d: Signed up, status=%d\r\n", my_tid, resp.status);

    // Immediately quit without playing
    req.type = RPS_QUIT;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_printf(CONSOLE, "ImmediateQuit %d: Quit immediately, exiting\r\n", my_tid);
    Exit();
}
