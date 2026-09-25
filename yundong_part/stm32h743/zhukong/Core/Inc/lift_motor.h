#ifndef LIFT_MOTOR_H
#define LIFT_MOTOR_H

#include "stm32h7xx_hal.h"

void LiftMotor_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef LiftMotor_Move(char direction);
HAL_StatusTypeDef LiftMotor_Stop(void);

#endif
