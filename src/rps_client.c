#include "rps_client.h"
#include "syscall.h"
#include "nameserver.h"
#include "rps_server.h"
#include "uart.h"


// get_server_tid: Helper to get the tid of the RPS server
static int get_server_tid(int tid) {
    int server_tid = WhoIs(RPS_SERVER_NAME);
    if (server_tid < 0) {
        uart_printf(CONSOLE, "Client %d: Failed to find RPS server\r\n", tid);
        Exit();
    }

    return server_tid;
}

void rps_client_standard_player(uint32_t rounds, int8_t* moves) {
    int my_tid = MyTid();
    int server_tid = get_server_tid(my_tid);

    uart_debug_printf(CONSOLE, "Client %d: Found RPS server at tid %d\r\n", my_tid, server_tid);

    RpsRequest req;
    RpsResponse resp;

    req.type = RPS_SIGNUP;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    if (resp.status == RPS_ERROR) {
        uart_printf(CONSOLE, "Client %d: Signup failed\r\n", my_tid);
        Exit();
    }

    uart_debug_printf(CONSOLE, "Client %d: Signed up, game starting\r\n", my_tid);

    // Play a few rounds
    for (uint32_t round = 0; round < rounds; round++) {
        req.type = RPS_PLAY;
        req.choice = moves[round];

        uart_debug_printf(CONSOLE, "Client %d: Round %d - Playing %s\r\n",
                    my_tid, round + 1, rps_choice_to_str(req.choice));

        Send(server_tid, (const char *)&req, sizeof(RpsRequest),
             (char *)&resp, sizeof(RpsResponse));

        if (resp.result == RPS_RESULT_OPPONENT_QUIT) {
            uart_printf(CONSOLE, "Client %d: Opponent quit!\r\n", my_tid);
            break;
        }

        uart_debug_printf(CONSOLE, "Client %d: Round %d - Result: %s (opponent: %s)\r\n",
                    my_tid, round + 1, rps_result_to_str(resp.result),
                    rps_choice_to_str(resp.opponent_choice));
    }

    req.type = RPS_QUIT;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_debug_printf(CONSOLE, "Client %d: Quit game, exiting\r\n", my_tid);
    Exit();
}

void rps_client_quick_rock() {
    int8_t moves[1] = {RPS_ROCK};
    rps_client_standard_player(1, moves);
}

void rps_client_quick_paper() {
    int8_t moves[1] = {RPS_PAPER};
    rps_client_standard_player(1, moves);
}

void rps_client_quick_scissor() {
    int8_t moves[1] = {RPS_SCISSORS};
    rps_client_standard_player(1, moves);
}

void rps_client_3round_rock() {
    int8_t moves[3] = {RPS_ROCK, RPS_ROCK, RPS_ROCK};
    rps_client_standard_player(3, moves);
}

void rps_client_3round_scissor() {
    int8_t moves[3] = {RPS_SCISSORS, RPS_SCISSORS, RPS_SCISSORS};
    rps_client_standard_player(3, moves);
}

void rps_client_5round_paper() {
    int8_t moves[5] = {RPS_PAPER, RPS_PAPER, RPS_PAPER, RPS_PAPER, RPS_PAPER};
    rps_client_standard_player(5, moves);
}

void rps_client_immediate_quit(void) {
    int my_tid = MyTid();
    int server_tid = get_server_tid(my_tid);

    uart_debug_printf(CONSOLE, "ImmediateQuit %d: Found RPS server\r\n", my_tid);

    RpsRequest req;
    RpsResponse resp;

    // Sign up
    req.type = RPS_SIGNUP;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_debug_printf(CONSOLE, "ImmediateQuit %d: Signed up, status=%d\r\n", my_tid, resp.status);

    // Immediately quit without playing
    req.type = RPS_QUIT;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_debug_printf(CONSOLE, "ImmediateQuit %d: Quit immediately, exiting\r\n", my_tid);
    Exit();
}

void rps_client_force_play() {
    int my_tid = MyTid();
    int server_tid = get_server_tid(my_tid);

    uart_debug_printf(CONSOLE, "Force Player %d: Found RPS server\r\n", my_tid);

    RpsRequest req;
    RpsResponse resp;

    req.type = RPS_PLAY;
    req.choice = RPS_SCISSORS;

    uart_printf(CONSOLE, "Client %d - Playing %s Without Signup!\r\n", my_tid, rps_choice_to_str(req.choice));

    Send(server_tid, (const char *)&req, sizeof(RpsRequest), (char *)&resp, sizeof(RpsResponse));

    if (resp.status == RPS_ERROR) {
        uart_printf(CONSOLE, "Client %d: Realized forgot to signup\r\n", my_tid);
    }

    Exit();
}

void rps_client_force_quit() {
    int my_tid = MyTid();
    int server_tid = get_server_tid(my_tid);

    uart_debug_printf(CONSOLE, "Force Player %d: Found RPS server\r\n", my_tid);

    RpsRequest req;
    RpsResponse resp;
    req.type = RPS_QUIT;

    uart_printf(CONSOLE, "Client %d - Quitting Without Signup!\r\n", my_tid);
    Send(server_tid, (const char *)&req, sizeof(RpsRequest), (char *)&resp, sizeof(RpsResponse));

    Exit();
}