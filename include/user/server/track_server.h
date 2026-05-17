#ifndef _track_server_h_
#define _track_server_h_ 1

#define TRACK_SERVER_NAME "TrackServer"

void track_server_task(void);
int TrackServerInit(int tid, int can_server_tid);

#endif /* _track_server_h_ */
