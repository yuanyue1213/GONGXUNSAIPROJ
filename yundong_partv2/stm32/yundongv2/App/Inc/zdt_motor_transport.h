/* 底盘串口传输接口：隔离中断接收缓冲和电机协议，读取操作有超时。 */
#ifndef ZDT_MOTOR_TRANSPORT_H
#define ZDT_MOTOR_TRANSPORT_H
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* 中断持续接收，前台事务读取；发送时保持接收开启。 */
void ZDT_Transport_Start(UART_HandleTypeDef *uart);
void ZDT_Transport_PrepareTx(void);
HAL_StatusTypeDef ZDT_Transport_Receive(UART_HandleTypeDef *uart,uint8_t *byte,uint32_t timeout);
bool ZDT_Motor_RxCallback(UART_HandleTypeDef *uart);
bool ZDT_Motor_ErrorCallback(UART_HandleTypeDef *uart);

#endif
