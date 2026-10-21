#include "zdt_motor_transport.h"

#define RX_SIZE 512U
#define RX_OVERFLOW_ERROR 0x80000000UL
static UART_HandleTypeDef *s_uart;
static uint8_t s_byte;
static volatile uint8_t s_rx[RX_SIZE];
static volatile uint16_t s_head,s_tail;
static volatile uint32_t s_error;
static volatile bool s_restart;
volatile ZDT_RxDiagnostics g_zdt_rx_diagnostics;

static bool arm(void)
{
    if(s_uart==NULL) return false;
    HAL_StatusTypeDef result=HAL_UART_Receive_IT(s_uart,&s_byte,1U);
    s_restart=result!=HAL_OK&&result!=HAL_BUSY;
    if(s_restart) ++g_zdt_rx_diagnostics.arm_failures;
    return !s_restart;
}
void ZDT_Transport_Start(UART_HandleTypeDef *uart)
{
    s_uart=uart; s_head=s_tail=0; s_error=0; s_restart=false;
    g_zdt_rx_diagnostics=(ZDT_RxDiagnostics){0};
    if(uart!=NULL) {
        HAL_NVIC_SetPriority(USART1_IRQn,4U,0U);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        (void)arm();
    }
}
void ZDT_Transport_PrepareTx(void)
{
    /* Only discard bytes predating a new request, never a current response.
     * Motor replies can arrive while blocking TX is returning: RX stays armed. */
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    s_tail=s_head; s_error=0;
    __set_PRIMASK(irq);
    if(s_restart) (void)arm();
}
HAL_StatusTypeDef ZDT_Transport_Receive(UART_HandleTypeDef *uart,uint8_t *byte,uint32_t timeout)
{
    if(uart==NULL||uart!=s_uart||byte==NULL) return HAL_ERROR;
    if(s_restart&&!arm()) return HAL_ERROR;
    uint32_t started=HAL_GetTick();
    do {
        /* A damaged/overflowed frame must not become a plausible valid one. */
        if(s_error) return HAL_ERROR;
        if(s_tail!=s_head) {
            *byte=s_rx[s_tail];
            s_tail=(uint16_t)((s_tail+1U)%RX_SIZE);
            return HAL_OK;
        }
    } while((uint32_t)(HAL_GetTick()-started)<timeout);
    return HAL_TIMEOUT;
}
uint32_t ZDT_Transport_Error(UART_HandleTypeDef *uart)
{ return uart==s_uart?s_error:0U; }
bool ZDT_Motor_RxCallback(UART_HandleTypeDef *uart)
{
    if(uart==NULL||uart!=s_uart) return false;
    ++g_zdt_rx_diagnostics.bytes;
    uint16_t next=(uint16_t)((s_head+1U)%RX_SIZE);
    if(next==s_tail) { ++g_zdt_rx_diagnostics.overflows; s_error|=RX_OVERFLOW_ERROR; }
    else { s_rx[s_head]=s_byte; s_head=next; }
    (void)arm(); return true;
}
bool ZDT_Motor_ErrorCallback(UART_HandleTypeDef *uart)
{
    if(uart==NULL||uart!=s_uart) return false;
    ++g_zdt_rx_diagnostics.uart_errors;
    g_zdt_rx_diagnostics.last_uart_error=uart->ErrorCode;
    s_error|=uart->ErrorCode?uart->ErrorCode:1U;
    __HAL_UART_CLEAR_OREFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    (void)arm(); return true;
}
