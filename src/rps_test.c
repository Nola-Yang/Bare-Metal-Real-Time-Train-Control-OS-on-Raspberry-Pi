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

static void test_immediatequit_nogameplayed() {
    int32_t tid = 0;
    uart_printf(CONSOLE, "\r\n-------- Test: Immediate Quitters --------\r\n");

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_immediate_quit);
    uart_printf(CONSOLE, "Created ImmediateQuitter 1, tid=%d\r\n", tid);

    tid = Create(RPS_CLIENT_PRIORITY, rps_client_immediate_quit);
    uart_printf(CONSOLE, "Created ImmediateQuitter 2, tid=%d\r\n", tid);
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
            uart_printf(CONSOLE, "Created RPS Player 1, tid=%d\r\n", tid);

            tid = Create(RPS_CLIENT_PRIORITY, p2_func);
            uart_printf(CONSOLE, "Created RPS Player 2, tid=%d\r\n", tid);

            uart_puts(CONSOLE, "\r\n");
        }
    }
}



void rps_test_run() {
    setup_test_suite();

    test_immediatequit_nogameplayed();
    //test_1roundallpossiblemoves_winorlose();
}