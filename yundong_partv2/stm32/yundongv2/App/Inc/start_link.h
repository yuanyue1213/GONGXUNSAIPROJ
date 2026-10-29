#ifndef START_LINK_H
#define START_LINK_H
#include "stm32h7xx_hal.h"
void StartLink_Init(UART_HandleTypeDef *uart);
/* Service 可在动作等待期间调用；Tick 仅由主循环调用，避免递归执行流程。 */
void StartLink_Service(void);
void StartLink_Tick(void);
void StartLink_Publish(const char *line);
#endif
