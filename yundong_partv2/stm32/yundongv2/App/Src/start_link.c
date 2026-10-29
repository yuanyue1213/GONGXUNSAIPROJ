/* UART5 启动入口：中断收字节，前台组帧，只接受 START。
 * 等待动作时仍可回复 BUSY；网络断开不会中止 STM32 的顺序流程。 */
#include "start_link.h"
#include "robot_config.h"
#include "start_protocol.h"
#include "mission_runner.h"
#include "camera_link.h"
#include "imu_link.h"
#include "zdt_motor_transport.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *command_uart;
static uint8_t byte;
static volatile uint8_t rx[START_RX_SIZE];
static volatile uint16_t head,tail;
static volatile bool fault;
static char frame[32];
static unsigned length;
static bool drop;
static uint32_t pending,last_start,active;
static void receive(void)
{
    if(command_uart) (void)HAL_UART_Receive_IT(command_uart,&byte,1U);
}
void StartLink_Init(UART_HandleTypeDef *uart)
{
    command_uart=uart; head=tail=0; fault=false; length=0; drop=false;
    pending=last_start=active=0; receive();
}
void StartLink_Publish(const char *line)
{
    if(command_uart) (void)HAL_UART_Transmit(command_uart,(uint8_t *)line,(uint16_t)strlen(line),UART_TX_TIMEOUT_MS);
}
static void status(uint32_t id,const char *state,const char *reason)
{
    char line[96];
    snprintf(line,sizeof(line),"AUTO,%lu,%s,0,0,0,0,%s\n",(unsigned long)id,state,reason);
    StartLink_Publish(line);
}
static void command(void)
{
    uint32_t id;
    if(!StartProtocol_Parse(frame,&id)) return;
    if(MissionRunner_Running()||pending) {
        status(active?active:pending,"BUSY","RUNNING");
        return;
    }
    if(id<=last_start) {
        status(id,"ERROR","DUPLICATE_START");
        return;
    }
    last_start=id;
    pending=id;
}
void StartLink_Service(void)
{
    ImuLink_Process();
    CameraLink_Process();
    if(!command_uart) return;
    if(command_uart->RxState==HAL_UART_STATE_READY) receive();
    if(fault) {
        uint32_t irq=__get_PRIMASK();
        __disable_irq();
        tail=head;
        fault=false;
        __set_PRIMASK(irq);
        length=0;
        drop=true;
    }
    /* 每轮最多读一缓冲，持续来帧时也让动作等待能返回检查时间。 */
    unsigned budget=START_RX_SIZE;
    while(budget--&&tail!=head) {
        char c=(char)rx[tail];
        tail=(uint16_t)((tail+1U)%START_RX_SIZE);
        if(c=='\n') {
            if(!drop) {
                frame[length]=0;
                command();
            }
            length=0;
            drop=false;
        } else if(!drop) {
            if(length<sizeof(frame)-1U) frame[length++]=c;
            else drop=true;
        }
    }
}
void StartLink_Tick(void)
{
    if(!pending) return;
    active=pending;
    pending=0;
    MissionRunner_Run(active);
    active=0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if(ZDT_Motor_RxCallback(uart)||ImuLink_RxCallback(uart)||CameraLink_RxCallback(uart)) return;
    if(uart!=command_uart) return;
    uint16_t next=(uint16_t)((head+1U)%START_RX_SIZE);
    if(next==tail) fault=true;
    else { rx[head]=byte; head=next; }
    receive();
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if(ZDT_Motor_ErrorCallback(uart)||ImuLink_ErrorCallback(uart)||CameraLink_ErrorCallback(uart)) return;
    if(uart!=command_uart) return;
    fault=true;
    __HAL_UART_CLEAR_OREFLAG(uart); __HAL_UART_CLEAR_NEFLAG(uart); __HAL_UART_CLEAR_FEFLAG(uart);
    receive();
}
