#ifndef _position_server_h_
#define _position_server_h_ 1

#include <stdint.h>

#define POSITION_SERVER_NAME "PositionServer"

void position_server_task(void);
int PositionServerInit(int tid);
int PositionServerOnSensor(int tid, uint16_t sensor_id, uint64_t time_us);
int PositionServerOnFastTick(int tid, uint64_t now_us);
int PositionServerOnReplanTick(int tid, uint64_t now_us);
int PositionServerOnSwitchSettleTick(int tid, uint64_t now_us);
int PositionServerSpeedChange(int tid, int train_num, int user_speed);
int PositionServerReverse(int tid, int train_num);
int PositionServerGoto(int tid, int train_num, int target_idx, int speed_level, int32_t offset_mm);
int PositionServerStartFindPos(int tid, int train_num);
int PositionServerMarkRoutesDirty(int tid);
int PositionServerReleaseTrain(int tid, int train_num);

#endif /* _position_server_h_ */
