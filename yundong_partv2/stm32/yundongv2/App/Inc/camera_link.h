/* 相机链路负责收发及顺序等待；业务层只传任务种类并取得识别结果。 */
#ifndef CAMERA_LINK_H
#define CAMERA_LINK_H
#include "stm32h7xx_hal.h"
#include "task_protocol.h"
typedef void (*CameraService)(void);
void CameraLink_Init(UART_HandleTypeDef *uart,CameraService service);
bool CameraLink_Wait(TaskVisionKind kind,unsigned target,TaskVision *result);
/* 粗调专用：允许返回经过识别窗口后收到的 NONE；无串口回复时仍等待。 */
bool CameraLink_Observe(TaskVisionKind kind,unsigned target,TaskVision *result);
void CameraLink_Idle(void);
void CameraLink_Process(void);
bool CameraLink_RxCallback(UART_HandleTypeDef *uart);
bool CameraLink_ErrorCallback(UART_HandleTypeDef *uart);
#endif
