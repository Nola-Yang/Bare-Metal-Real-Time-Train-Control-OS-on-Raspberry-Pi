#ifndef _deadlock_server_h_
#define _deadlock_server_h_ 1

#include <stdint.h>

#define DEADLOCK_SERVER_NAME "DeadlockServer"

void deadlock_server_task(void);
int DeadlockServerInit(int tid);
int DeadlockServerOnTick(int tid, uint64_t now_us);

#endif /* _deadlock_server_h_ */
