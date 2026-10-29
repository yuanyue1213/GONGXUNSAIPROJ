/* 与硬件无关的单通道舵机轨迹状态：记录起点、目标和时间，计算当前输出角。
 * 完成表示软件轨迹到期，不代表通过位置传感器确认了舵机实际到位。 */
#ifndef SERVO_TRAJECTORY_H
#define SERVO_TRAJECTORY_H
#include <stdint.h>
#include <stdbool.h>
/* Pure trajectory state; hardware access and interrupt exclusion live in servo_control. */
typedef struct {
    int32_t start,target;
    uint32_t started,duration;
    uint16_t angle;
    bool known,active;
} ServoTrajectory;
void ServoTrajectory_Set(ServoTrajectory *job,uint16_t angle);
void ServoTrajectory_Start(ServoTrajectory *job,uint16_t target,uint32_t duration,uint32_t now);
uint16_t ServoTrajectory_Update(ServoTrajectory *job,uint32_t now);
#endif
