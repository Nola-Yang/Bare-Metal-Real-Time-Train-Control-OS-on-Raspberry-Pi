#ifndef RPS_CLIENT_H
#define RPS_CLIENT_H

#include <stdint.h>


// Standard client: plays multiple rounds
void rps_client_standard_player(uint32_t rounds, int8_t* moves);

// rps_client_quick_rock: Plays only 1 round with Rock
void rps_client_quick_rock();

// rps_client_quick_paper: Plays only 1 round with Paper
void rps_client_quick_paper();

// rps_client_quick_scissor: Plays only 1 round with Scissor
void rps_client_quick_scissor();

// Early quitter: plays 2 rounds then quits (tests opponent quit scenario)
void rps_client_early_quit(void);

// Long player: plays many rounds (paired with early quitter to test opponent quit)
void rps_client_long_player(void);

// Immediate quitter: signs up and immediately quits (edge case test)
void rps_client_immediate_quit(void);

#endif