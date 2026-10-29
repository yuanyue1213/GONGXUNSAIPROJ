#include "robot_config.h"
/* 底盘 USART1 接收传输层：中断写入环形缓冲，前台事务按超时读取字节。
 * 发送期间仍保持接收；溢出或串口错误使当前帧失效，避免残缺数据被误认为有效应答。 */
#include "zdt_motor_transport.h"

static UART_HandleTypeDef *s_uart;
static uint8_t s_byte;
static volatile uint8_t s_rx[MOTOR_RX_SIZE];
static volatile uint16_t s_head,s_tail;
static volatile bool s_fault;
static volatile bool s_restart;

static bool arm(void)
{
    if(s_uart==NULL) return false;
    HAL_StatusTypeDef result=HAL_UART_Receive_IT(s_uart,&s_byte,1U);
    s_restart=result!=HAL_OK&&result!=HAL_BUSY;
    return !s_restart;
}
void ZDT_Transport_Start(UART_HandleTypeDef *uart)
{
    s_uart=uart; s_head=s_tail=0; s_fault=false; s_restart=false;
    if(uart!=NULL) {
        HAL_NVIC_SetPriority(USART1_IRQn,4U,0U);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        (void)arm();
    }
}
/* 仅清理新请求之前的旧数据；发送期间的接收中断保持开启，以接住快速返回的应答。 */
void ZDT_Transport_PrepareTx(void)
{
    /* Only discard bytes predating a new request, never a current response.
     * Motor replies can arrive while blocking TX is returning: RX stays armed. */
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    s_tail=s_head; s_fault=false;
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
        if(s_fault) return HAL_ERROR;
        if(s_tail!=s_head) {
            *byte=s_rx[s_tail];
            s_tail=(uint16_t)((s_tail+1U)%MOTOR_RX_SIZE);
            return HAL_OK;
        }
    } while((uint32_t)(HAL_GetTick()-started)<timeout);
    return HAL_TIMEOUT;
}
/* 中断仅把字节压入缓冲并重新挂接接收；具体协议校验在前台完成。 */
bool ZDT_Motor_RxCallback(UART_HandleTypeDef *uart)
{
    if(uart==NULL||uart!=s_uart) return false;
    uint16_t next=(uint16_t)((s_head+1U)%MOTOR_RX_SIZE);
    if(next==s_tail) s_fault=true;
    else { s_rx[s_head]=s_byte; s_head=next; }
    (void)arm(); return true;
}
bool ZDT_Motor_ErrorCallback(UART_HandleTypeDef *uart)
{
    if(uart==NULL||uart!=s_uart) return false;
    s_fault=true;
    __HAL_UART_CLEAR_OREFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    (void)arm(); return true;
}
