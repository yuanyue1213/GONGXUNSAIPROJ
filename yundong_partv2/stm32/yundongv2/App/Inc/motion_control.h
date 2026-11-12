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
float Motion_Yaw(void); /* 新鲜IMU航向，单位度；无数据时沿用现有等待策略。 */
bool Motion_Move(char direction, float mm, uint16_t rpm);
bool Motion_MoveAt(char direction,float mm,uint16_t rpm,uint8_t acceleration); /* 指定Emm加速度档位。 */
#if defined(FORWARD_TEST)
/* 独立测试：纯位置模式直行，按预计时间等待并停车，不使用IMU。仅支持F/B。 */
bool Motion_MoveStraight(char direction,float mm,uint16_t rpm);
#endif
bool Motion_Turn(char direction, float degrees, uint16_t rpm);
bool Motion_TurnFine(float degrees,uint16_t rpm); /* 相对当前IMU航向，顺时针为正。 */
bool Motion_RestoreYaw(float yaw,uint16_t rpm); /* 回到记录的展开航向，不修改路线累计目标。 */
bool Motion_TurnPulses(int32_t pulses,uint16_t rpm); /* 左转为正，车轮相对脉冲。 */
bool Motion_Servo(char channel, uint16_t target, uint16_t dps);
/* r 使用 0.1 mm，z 使用 mm，base 使用度；三轴同时启动，等待预计最长时间。 */
bool Motion_MoveAll(int32_t r, int32_t z, uint16_t base, uint16_t rpm);
/* 当前伸缩逻辑位置，单位 0.1 mm，用于收臂时先保持伸缩抬升。 */
int32_t Motion_Radial(void);
void Motion_Stop(void);
#endif
