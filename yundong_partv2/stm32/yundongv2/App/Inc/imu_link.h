/* JY901S 串口链路：接收中断收集字节，前台解析姿态数据并推进设备配置状态。
 * 控制层应通过接口读取样本及其有效性，避免把过期姿态用于运动闭环。 */
#ifndef IMU_LINK_H
#define IMU_LINK_H
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include "robot_config.h"
typedef struct { float yaw,rate; uint32_t angle_tick,rate_tick; } ImuSample;
void ImuLink_Init(UART_HandleTypeDef *uart);
/* Invoke once at boot, before accepting any IMU-dependent motion. */
void ImuLink_StartConfiguration(void);
/* Board port: abort RX, change baud, clear old RX data; does not re-arm RX. */
HAL_StatusTypeDef ImuLink_SetBaud(UART_HandleTypeDef *uart,uint32_t baud);
void ImuLink_Process(void);
bool ImuLink_RxCallback(UART_HandleTypeDef *uart);
bool ImuLink_ErrorCallback(UART_HandleTypeDef *uart);
bool ImuLink_Read(ImuSample *sample);
#endif
