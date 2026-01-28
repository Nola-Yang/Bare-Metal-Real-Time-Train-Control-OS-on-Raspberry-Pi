#include "rps_test.h"
#include "syscall.h"
#include "rps_server.h"
#include "rps_client.h"
#include "uart.h"
#include "task_scheduler.h"


typedef void (*Task_Func)();


static void setup_test_suite() {
    int tid = Create(RPS_SERVER_PRIORITY, rps_server_task);
    if (tid < 0) {
        uart_printf(CONSOLE, "ERROR: Failed to create RPS Server\r\n");
        Exit();
    }

    uart_printf(CONSOLE, "Created RPS Server, tid=%d\r\n", tid);
}

static Task_Func get_quick_func(int8_t rps_move) {
    switch (rps_move){
        case RPS_ROCK:
            return rps_client_quick_rock;
        case RPS_PAPER:
            return rps_client_quick_paper;
        case RPS_SCISSORS:
            return rps_client_quick_scissor;
        default:
            return NULL;
    }
}

// ============= Test Cases ==========================

static void test_playwithoutsignup_nomoveplayed() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: Play Without Signup --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_force_play);
    uart_debug_printf(CONSOLE, "Created Force Player, tid=%d\r\n", tid);
}

static void test_quitwithoutsignup_noplayerremoved() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: Quit Without Signup --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_force_quit);
    uart_debug_printf(CONSOLE, "Created Force Quitter, tid=%d\r\n", tid);
}

static void test_gamestartedimmediatequit_nogameplayed() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: Immediate Quitters --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_immediate_quit);
    uart_debug_printf(CONSOLE, "Created Immediate Quitter 1, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_immediate_quit);
    uart_debug_printf(CONSOLE, "Created Immediate Quitter 2, tid=%d\r\n", tid);
}

static void test_playerdoublesignup_playerisalreadyingame() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: Double Signup --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_double_signup_quick_paper);
    uart_debug_printf(CONSOLE, "Created Double Signup Player, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_quick_scissor);
    uart_debug_printf(CONSOLE, "Created Standard Player, tid=%d\r\n", tid);
}

static void test_playsagain_2gamesplayed() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: Player Plays Again --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_play_again);
    uart_debug_printf(CONSOLE, "Created Replay Player, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_quick_scissor);
    uart_debug_printf(CONSOLE, "Created Standard Player, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_quick_rock);
    uart_debug_printf(CONSOLE, "Created Standard Player, tid=%d\r\n", tid);
}

static void test_1stplayerquitmidway_gameends() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: 1st Player Early Quit --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_immediate_quit);
    uart_debug_printf(CONSOLE, "Created Immediate Quitter, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_3round_rock);
    uart_debug_printf(CONSOLE, "Created Standard Player, tid=%d\r\n", tid);
}

static void test_2ndplayerquitmidway_gameends() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: 2nd Player Early Quit --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_3round_rock);
    uart_debug_printf(CONSOLE, "Created Standard Player, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_immediate_quit);
    uart_debug_printf(CONSOLE, "Created ImmediateQuitter, tid=%d\r\n", tid);
}

static void test_multiroundgames_playuntilaplayerquits() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: Play for multiple rounds --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_3round_rock);
    uart_debug_printf(CONSOLE, "Created 3 Round Player, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_5round_paper);
    uart_debug_printf(CONSOLE, "Created 5 Round Player, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_3round_scissor);
    uart_debug_printf(CONSOLE, "Created 3 Round Player, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_quick_scissor);
    uart_debug_printf(CONSOLE, "Created 1 Round Player, tid=%d\r\n", tid);
}

static void test_1roundallpossiblemoves_winorlose() {
    uart_printf(CONSOLE, "\r\n-------- Test: All Standard RPS Combinations --------\r\n");

    int32_t tid = 0;

    Task_Func p1_func;
    Task_Func p2_func;

    for (uint8_t p1_move = RPS_ROCK; p1_move <= RPS_SCISSORS; ++p1_move) {
        p1_func = get_quick_func(p1_move);

        for (uint8_t p2_move = RPS_ROCK; p2_move <= RPS_SCISSORS; ++p2_move) {
            p2_func = get_quick_func(p2_move);

            uart_printf(CONSOLE, "~~~~~ %s vs %s ~~~~~\r\n", rps_choice_to_str(p1_move), rps_choice_to_str(p2_move));

            tid = Create(RPS_CLIENT_PRIORITY, p1_func);
            uart_debug_printf(CONSOLE, "Created Standard Player 1, tid=%d\r\n", tid);

            tid = Create(RPS_CLIENT_PRIORITY, p2_func);
            uart_debug_printf(CONSOLE, "Created Standard Player 2, tid=%d\r\n", tid);

            uart_puts(CONSOLE, "\r\n");
        }
    }
}

// ===================================================


void rps_test_run() {
    setup_test_suite();

    test_playwithoutsignup_nomoveplayed();
    test_quitwithoutsignup_noplayerremoved();
    test_gamestartedimmediatequit_nogameplayed();
    test_playerdoublesignup_playerisalreadyingame();
    test_playsagain_2gamesplayed();
    test_1stplayerquitmidway_gameends();
    test_2ndplayerquitmidway_gameends();
    test_multiroundgames_playuntilaplayerquits();

    #ifdef FULLTEST
    test_1roundallpossiblemoves_winorlose();
    #endif
}