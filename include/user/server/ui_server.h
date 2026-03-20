#ifndef _ui_server_h_
#define _ui_server_h_ 1

#include "server/terminal_server.h"

#define UI_SERVER_NAME "UIServer"

#define UI_SERVER_MSG_ACQUIRE      0
#define UI_SERVER_MSG_RELEASE      1
#define UI_SERVER_MSG_RAW_PUTC     2
#define UI_SERVER_MSG_RAW_PUTS     3
#define UI_SERVER_MSG_CMD_PUTC     4
#define UI_SERVER_MSG_CMD_PUTS     5
#define UI_SERVER_MSG_CMD_BACKSPACE 6
#define UI_SERVER_MSG_CMD_ENTER    7
#define UI_SERVER_MSG_CMD_PROMPT   8
#define UI_SERVER_MSG_CMD_PREPARE  9

#define UI_SERVER_BATCH_RAW     0
#define UI_SERVER_BATCH_COMMAND 1

typedef struct {
    int type;
    int mode;
    int len;
    char ch;
    char str[TERM_MAX_STR_LEN];
} UIServerRequest_t;

typedef struct {
    int status;
} UIServerReply_t;

void ui_server_task(void);

int UIServerPutc(int tid, char ch);
int UIServerPuts(int tid, const char *str, int len);
int UIServerCmdPutc(int tid, char ch);
int UIServerCmdPuts(int tid, const char *str, int len);
int UIServerCmdBackspace(int tid);
int UIServerCmdEnter(int tid);
int UIServerCmdPrompt(int tid);
int UIServerPrepareCmd(int tid);

#endif /* _ui_server_h_ */
