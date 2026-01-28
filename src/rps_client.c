#include "rps_client.h"
#include "syscall.h"
#include "nameserver.h"
#include "rps_server.h"
#include "uart.h"


void rps_client_standard_player(uint32_t rounds, int8_t* moves) {
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
    for (uint32_t round = 0; round < rounds; round++) {
        req.type = RPS_PLAY;
        req.choice = moves[round];

        uart_printf(CONSOLE, "Client %d: Round %d - Playing %s\r\n",
                    my_tid, round + 1, rps_choice_to_str(req.choice));

        Send(server_tid, (const char *)&req, sizeof(RpsRequest),
             (char *)&resp, sizeof(RpsResponse));

        if (resp.result == RPS_RESULT_OPPONENT_QUIT) {
            uart_printf(CONSOLE, "Client %d: Opponent quit!\r\n", my_tid);
            break;
        }

        uart_printf(CONSOLE, "Client %d: Round %d - Result: %s (opponent: %s)\r\n",
                    my_tid, round + 1, rps_result_to_str(resp.result),
                    rps_choice_to_str(resp.opponent_choice));
    }

    req.type = RPS_QUIT;
    Send(server_tid, (const char *)&req, sizeof(RpsRequest),
         (char *)&resp, sizeof(RpsResponse));

    uart_printf(CONSOLE, "Client %d: Quit game, exiting\r\n", my_tid);
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
                    my_tid, round + 1, rps_result_to_str(resp.result));
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
                    my_tid, round + 1, rps_choice_to_str(req.choice));

        Send(server_tid, (const char *)&req, sizeof(RpsRequest),
             (char *)&resp, sizeof(RpsResponse));

        if (resp.result == RPS_RESULT_OPPONENT_QUIT) {
            uart_printf(CONSOLE, "LongPlayer %d: Opponent quit!\r\n", my_tid);
            break;
        }

        uart_printf(CONSOLE, "LongPlayer %d: Round %d - Result: %s\r\n",
                    my_tid, round + 1, rps_result_to_str(resp.result));
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