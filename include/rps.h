#ifndef RPS_H
#define RPS_H

#define RPS_SERVER_NAME "rps_server"

//request types
#define RPS_SIGNUP  1
#define RPS_PLAY    2
#define RPS_QUIT    3

//play choices
#define RPS_ROCK     0
#define RPS_PAPER    1
#define RPS_SCISSORS 2

// game results
#define RPS_RESULT_WIN      1
#define RPS_RESULT_LOSE     2
#define RPS_RESULT_TIE      3
#define RPS_RESULT_OPPONENT_QUIT 4

// response status
#define RPS_OK              0
#define RPS_ERROR           -1
#define RPS_WAITING         1
#define RPS_GAME_START      2

typedef struct {
    int type;
    int choice;  
} RpsRequest;

typedef struct {
    int status;
    int result;      
    int opponent_choice;  
} RpsResponse;

void rps_server_task(void);

// Standard client: plays multiple rounds
void rps_client_task(void);

// Early quitter: plays 2 rounds then quits (tests opponent quit scenario)
void rps_client_early_quit(void);

// Long player: plays many rounds (paired with early quitter to test opponent quit)
void rps_client_long_player(void);

// Immediate quitter: signs up and immediately quits (edge case test)
void rps_client_immediate_quit(void);

#endif /* RPS_H */