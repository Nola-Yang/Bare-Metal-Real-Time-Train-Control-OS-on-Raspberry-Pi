#ifndef RPS_MESSAGE_H
#define RPS_MESSAGE_H


// RpsRequest: Client request message to the RPS server
typedef struct {
    int type;
    int choice;  
} RpsRequest;


// RpsResponse: 
typedef struct {
    int status;
    int result;      
    int opponent_choice;  
} RpsResponse;


#endif