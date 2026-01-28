#ifndef RPS_SERVER_H
#define RPS_SERVER_H

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


// rps_server_task: Runs the RPS server
void rps_server_task(void);

// rps_choice_to_str: Converts an RPS game choice to its string representation
const char *rps_choice_to_str(int choice);

// rps_result_to_str: Converts the result of a RPS game to its string representation
const char *rps_result_to_str(int result);

#endif