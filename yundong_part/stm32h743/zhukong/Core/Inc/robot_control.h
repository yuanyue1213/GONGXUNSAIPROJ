/**
 ******************************************************************************
 * @file    robot_control.h
 * @brief   ESP32 手机遥控命令的 USART3 接收与四轮底盘控制。
 ******************************************************************************
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>

/** 初始化 USART3 命令处理，并记录四轮电机串口初始化结果。 */
void RobotControl_Init(UART_HandleTypeDef *command_uart, bool wheel_uart_ready);

/** 在主循环中持续调用，轮询接收并执行 ESP32 转发的命令。 */
void RobotControl_Process(void);

/** 在主循环中持续调用，处理通信超时停车。 */
void RobotControl_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONTROL_H */
