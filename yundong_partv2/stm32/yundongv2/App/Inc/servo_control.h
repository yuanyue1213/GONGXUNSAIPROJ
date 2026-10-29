/* 舵机硬件层：TIM1/TIM3 输出 PWM，TIM6 每 20 ms 推进 G/T/B 三路独立轨迹。
 * 前台与中断共享的轨迹状态通过短临界区保护；中断内不得进行电机串口等待。 */
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* G: PC8 clamp, T: PA8 turntable, B: PC6 base. */
/* PWM 外设由 CubeMX 配置为 1 MHz 计数、20000 周期；此处绑定并启动轨迹时钟。 */
bool ServoControl_Init(TIM_HandleTypeDef *tim1,TIM_HandleTypeDef *tim3);
bool ServoControl_SetAngle(char channel, uint16_t angle);
/* Reference-style independent servo jobs, updated by TIM6 every 20 ms. */
bool ServoControl_MoveTo(char channel,uint16_t target,uint32_t duration_ms);
bool ServoControl_GetAngle(char channel,uint16_t *angle);
void ServoControl_Cancel(char channel);
void ServoControl_TimerIRQ(void);

#endif
