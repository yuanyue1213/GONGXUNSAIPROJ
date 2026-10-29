/* 底盘 USART1：只保留使能、同步位置、同步轮速及立即停止。 */
#ifndef ZDT_MOTOR_H
#define ZDT_MOTOR_H
#include "stm32h7xx_hal.h"
#include "stepper_limits.h"
void ZDT_Motor_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef ZDT_Motor_Enable(uint8_t id);
/* 数组顺序：左前、右前、右后、左后。四条命令确认后统一 FF 触发。 */
HAL_StatusTypeDef ZDT_Motor_MoveWheelSynchronous(const int32_t pulses[4],uint16_t rpm,uint8_t acceleration);
HAL_StatusTypeDef ZDT_Motor_SetWheelSpeedsSynchronous(const int16_t rpm[4],uint8_t acceleration);
HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id);
#endif
