#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

/* 顺序动作接口；调用返回时预计运动时间已结束，不查询步进实际到位状态。 */
typedef void (*MotionService)(void);
void Motion_Init(MotionService service);
void Motion_Wait(uint32_t ms);
uint32_t Motion_EstimateMs(double revolutions, uint16_t rpm, double acceleration);
bool Motion_Reference(void);
bool Motion_Move(char direction, float mm, uint16_t rpm);
bool Motion_Turn(char direction, float degrees, uint16_t rpm);
bool Motion_Servo(char channel, uint16_t target, uint16_t dps);
/* r 使用 0.1 mm，z 使用 mm，base 使用度；三轴同时启动，等待预计最长时间。 */
bool Motion_MoveAll(int32_t r, int32_t z, uint16_t base, uint16_t rpm);
/* 当前伸缩逻辑位置，单位 0.1 mm，用于收臂时先保持伸缩抬升。 */
int32_t Motion_Radial(void);
void Motion_Stop(void);
#endif
