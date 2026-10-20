#ifndef IMU_LINK_H
#define IMU_LINK_H
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#define IMU_UART_BAUD_RATE 9600U
typedef struct { float yaw,rate; uint32_t angle_tick,rate_tick,sequence,epoch; } ImuSample;
typedef struct {
    uint32_t baud,rx_bytes,valid_frames,register_replies,uart_errors,bad_checksums;
    uint32_t angle_frames,rate_frames,last_valid_baud;
    uint16_t output,frequency,baud_code;
} ImuDiagnostics;
void ImuLink_Diagnostics(ImuDiagnostics *diagnostics);
void ImuLink_Init(UART_HandleTypeDef *uart);
/* Invoke once at boot, before accepting any IMU-dependent motion. */
void ImuLink_StartConfiguration(void);
const char *ImuLink_Status(void);
/* Board port: abort RX, change baud, clear old RX data; does not re-arm RX. */
HAL_StatusTypeDef ImuLink_SetBaud(UART_HandleTypeDef *uart,uint32_t baud);
void ImuLink_Process(void);
bool ImuLink_RxCallback(UART_HandleTypeDef *uart);
bool ImuLink_ErrorCallback(UART_HandleTypeDef *uart);
bool ImuLink_Read(ImuSample *sample);
#endif
