#ifndef _demo_server_h_
#define _demo_server_h_ 1

#include <stdint.h>
#include "runtime_protocol.h"

#define DEMO_SERVER_NAME "DemoServer"

void demo_server_task(void);
int DemoServerInit(int tid);
int DemoServerHandleCommand(int tid, const train_command_t *cmd);
int DemoServerOnTick(int tid, uint64_t now_us);

#endif /* _demo_server_h_ */
