#ifndef GYRO_TURN_H
#define GYRO_TURN_H
#include "../../../../shared/gyro_turn_protocol.h"
typedef enum { GYRO_IDLE, GYRO_RUNNING, GYRO_DONE, GYRO_ERROR } GyroTurnState;
void GyroTurn_Init(void);
void GyroTurn_SafeBoot(void);
bool GyroTurn_Start(const GyroCommand *command);
void GyroTurn_Tick(void);
void GyroTurn_Stop(void);
GyroTurnState GyroTurn_State(void);
bool GyroTurn_Busy(void);
const char *GyroTurn_Reason(void);
#endif
