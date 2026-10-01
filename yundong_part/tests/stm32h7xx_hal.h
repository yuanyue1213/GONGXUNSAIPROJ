#ifndef HOST_HAL_H
#define HOST_HAL_H
#include <stdint.h>
#include <stddef.h>
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_RX 1U
typedef struct { int instance; uint32_t RxState; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define HAL_MAX_DELAY UINT32_MAX
#define USART3_IRQn 0
#define UART4_IRQn 1
#define __HAL_UART_CLEAR_OREFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_NEFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_FEFLAG(x) ((void)(x))
static inline void HAL_NVIC_SetPriority(int irq, int priority, int sub) { (void)irq; (void)priority; (void)sub; }
static inline void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t state) { (void)state; }
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
