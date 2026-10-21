#ifndef ZDT_MOTOR_TRANSPORT_H
#define ZDT_MOTOR_TRANSPORT_H
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Main-loop transaction API. Hardware target binds to the continuous RX ring;
 * standalone driver tests may bind to the weak blocking transport. */
void ZDT_Transport_Start(UART_HandleTypeDef *uart);
void ZDT_Transport_PrepareTx(void);
HAL_StatusTypeDef ZDT_Transport_Receive(UART_HandleTypeDef *uart,uint8_t *byte,uint32_t timeout);
uint32_t ZDT_Transport_Error(UART_HandleTypeDef *uart);
bool ZDT_Motor_RxCallback(UART_HandleTypeDef *uart);
bool ZDT_Motor_ErrorCallback(UART_HandleTypeDef *uart);

typedef struct {
    uint32_t bytes,uart_errors,overflows,arm_failures,last_uart_error;
} ZDT_RxDiagnostics;
extern volatile ZDT_RxDiagnostics g_zdt_rx_diagnostics;
#endif
