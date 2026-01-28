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

// rps_client_3round_rock: Plays 3 rounds of consecutive Rock
void rps_client_3round_rock();

// rps_client_3round_scissor: Plays 3 rounds of consecutive Scissor
void rps_client_3round_scissor();

// rps_client_5round_paper: Plays 5 rounds of consecutive Paper
void rps_client_5round_paper();

// Immediate quitter: signs up and immediately quits (edge case test)
void rps_client_immediate_quit(void);

// Force player: trys to play a move even if they have not signed up yet
void rps_client_force_play();

// Force quitter: trys to quit even if they have not signed up yet
void rps_client_force_quit();

#endif