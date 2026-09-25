#ifndef LIFT_MOTOR_H
#define LIFT_MOTOR_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>

void LiftMotor_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef LiftMotor_Move(char direction);
HAL_StatusTypeDef LiftMotor_Stop(void);
/* USART2 上的 ID 2：机械臂前后电机。 */
HAL_StatusTypeDef ArmMotor_MoveForeAft(char direction);
HAL_StatusTypeDef ArmMotor_StopForeAft(void);
bool LiftMotor_UartReady(void);
bool LiftMotor_Probe(uint8_t id);
char LiftMotor_FirmwareCode(uint8_t id);
const char *LiftMotor_LastError(uint8_t id);
bool LiftMotor_ReadStatus(uint8_t id, uint8_t *status);

#endif
