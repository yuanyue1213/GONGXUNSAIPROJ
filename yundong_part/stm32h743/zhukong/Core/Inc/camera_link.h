#ifndef CAMERA_LINK_H
#define CAMERA_LINK_H
#include "stm32h7xx_hal.h"
#include "../../../../shared/camera_position_protocol.h"
void CameraLink_Init(UART_HandleTypeDef *uart);
bool CameraLink_Ready(void);
void CameraLink_Process(void);
bool CameraLink_TakeCenter(CameraCenter *center);
void CameraLink_Discard(void);
bool CameraLink_RxCallback(UART_HandleTypeDef *uart);
bool CameraLink_ErrorCallback(UART_HandleTypeDef *uart);
#endif
