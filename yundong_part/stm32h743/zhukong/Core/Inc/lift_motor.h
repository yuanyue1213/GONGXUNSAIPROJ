#ifndef LIFT_MOTOR_H
#define LIFT_MOTOR_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>

void LiftMotor_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef LiftMotor_Move(char direction);
HAL_StatusTypeDef LiftMotor_Stop(void);
/* USART2 ID1 calibration: motor angle in 0.1 degrees, U up / D down. */
HAL_StatusTypeDef LiftMotor_MoveAngle(char direction, uint32_t angle_tenths,
                                     uint16_t speed_rpm, uint32_t pulses_per_rev);
/* USART2 上的 ID 2：机械臂前后电机。 */
HAL_StatusTypeDef ArmMotor_StopForeAft(void);
/* 前后电机轴转角，不是舵机角度；angle_tenths 单位 0.1°。 */
HAL_StatusTypeDef ArmMotor_MoveAngle(char direction, uint32_t angle_tenths,
                                    uint16_t speed_rpm, uint32_t pulses_per_rev);
bool LiftMotor_UartReady(void);
bool LiftMotor_Probe(uint8_t id);
char LiftMotor_FirmwareCode(uint8_t id);
const char *LiftMotor_LastError(uint8_t id);
bool LiftMotor_ReadStatus(uint8_t id, uint8_t *status);
/* Signed real motor position in 0.1 degrees, converted for Emm/X firmware. */
bool ArmMotor_ReadPosition(uint8_t id, int64_t *angle_tenths);
/* FD mode 01: signed absolute target in the driver's coordinate system. */
HAL_StatusTypeDef ArmMotor_MoveAbsolute(uint8_t id, int64_t angle_tenths,
                                       uint16_t rpm, uint32_t pulses_per_rev);

#endif
