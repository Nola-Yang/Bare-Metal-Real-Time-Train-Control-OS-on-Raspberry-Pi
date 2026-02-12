#ifndef _terminal_server_h_
#define _terminal_server_h_ 1

#include <stdint.h>

#define TERMINAL_SERVER_NAME "TerminalServer"
#define TERM_CHANNEL_CONSOLE 0

#define TERM_MAX_STR_LEN 128

// Message types
#define TERM_MSG_GETC       0
#define TERM_MSG_PUTC       1
#define TERM_MSG_PUTS       2   // Send string 
#define TERM_MSG_RX_NOTIFY  3   // From RX notifier: data available
#define TERM_MSG_TX_NOTIFY  4   // From TX notifier: TX ready

typedef struct {
    int type;
    char ch;
    char str[TERM_MAX_STR_LEN];  
    int len;
} TermRequest_t;

typedef struct {
    int status;
    char ch;
} TermReply_t;

// creates RX/TX notifiers
void terminal_server_task(void);

// User API

// Get a character from the terminal 
// Returns: character on success, negative on error
int Getc(int tid, int channel);

// Put a character to the terminal
// Returns: 0 on success, negative on error
int Putc(int tid, int channel, char ch);

// Put a string to the terminal (data is copied for security)
// Returns: number of characters queued on success, negative on error
int Puts(int tid, int channel, const char *str, int len);

#endif /* _terminal_server_h_ */
