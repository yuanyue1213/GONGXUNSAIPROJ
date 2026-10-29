/* USART2：升降 ID1、伸缩 ID2，仅保留全流程使用的相对运动与停止。 */
#ifndef LIFT_MOTOR_H
#define LIFT_MOTOR_H
#include "stm32h7xx_hal.h"
void LiftMotor_Init(UART_HandleTypeDef *uart);
/* 电机轴转角单位 0.1 度；升降 U/D，伸缩 E/C。 */
HAL_StatusTypeDef LiftMotor_MoveAngle(char direction,uint32_t angle,uint16_t rpm,uint32_t pulses_per_rev);
HAL_StatusTypeDef ArmMotor_MoveAngle(char direction,uint32_t angle,uint16_t rpm,uint32_t pulses_per_rev);
HAL_StatusTypeDef LiftMotor_Stop(void);
HAL_StatusTypeDef ArmMotor_StopForeAft(void);
/* 返回已识别的固件 E/X，未识别时为 ?；用于预计动作时间。 */
char LiftMotor_FirmwareCode(uint8_t id);
#endif
