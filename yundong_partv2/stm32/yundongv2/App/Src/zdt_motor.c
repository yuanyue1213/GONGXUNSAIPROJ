#include "robot_config.h"
/* 当前底盘只使用 Emm 的 FD/F6 同步命令。ACK 表示接收成功，不表示到位。 */
#include "zdt_motor.h"
#include "zdt_motor_transport.h"
#include <string.h>
#define FRAME_TAIL 0x6BU
static UART_HandleTypeDef *s_uart;
/* 左前、右前、右后、左后：软件正值的实际电机方向，0=CW，1=CCW。 */
static const uint8_t forward[4]=CHASSIS_FORWARD_POLARITY;
void ZDT_Motor_Init(UART_HandleTypeDef *uart)
{
    s_uart=uart; ZDT_Transport_Start(uart);
}
static HAL_StatusTypeDef send_raw(const uint8_t *frame,uint16_t length)
{
    if(!s_uart) return HAL_ERROR;
    ZDT_Transport_PrepareTx();
    return HAL_UART_Transmit(s_uart,(uint8_t *)frame,length,MOTOR_ACK_TIMEOUT_MS);
}
static HAL_StatusTypeDef send_command(const uint8_t *frame,uint16_t length)
{
    HAL_StatusTypeDef result=send_raw(frame,length);
    if(result!=HAL_OK) return result;
    uint8_t response[4]; unsigned count=0,scanned=0;
    uint32_t started=HAL_GetTick();
    while(scanned++<20U) {
        uint32_t elapsed=HAL_GetTick()-started;
        if(elapsed>=MOTOR_ACK_TIMEOUT_MS||ZDT_Transport_Receive(s_uart,&response[count],MOTOR_ACK_TIMEOUT_MS-elapsed)!=HAL_OK) break;
        if(++count<4U) continue;
        if(response[0]==frame[0]&&response[3]==FRAME_TAIL) {
            if(response[1]==frame[1]) return response[2]==0x02U?HAL_OK:HAL_ERROR;
            if(response[1]==0&&response[2]==0xEE) return HAL_ERROR;
        }
        memmove(response,response+1,3); count=3;
    }
    return HAL_TIMEOUT;
}
HAL_StatusTypeDef ZDT_Motor_Enable(uint8_t id)
{
    if(id<1U||id>4U) return HAL_ERROR;
    uint8_t frame[]={id,0xF3,0xAB,1,0,FRAME_TAIL};
    return send_command(frame,sizeof(frame));
}
HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id)
{
    if(id<1U||id>4U) return HAL_ERROR;
    uint8_t frame[]={id,0xFE,0x98,0,FRAME_TAIL};
    return send_command(frame,sizeof(frame));
}
static HAL_StatusTypeDef trigger(void)
{
    const uint8_t frame[]={0,0xFF,0x66,FRAME_TAIL};
    return send_raw(frame,sizeof(frame));
}
HAL_StatusTypeDef ZDT_Motor_MoveWheelSynchronous(const int32_t pulses[4],uint16_t rpm,uint8_t acceleration)
{
    if(!pulses||!rpm||rpm>STEPPER_MAX_SPEED_RPM) return HAL_ERROR;
    for(unsigned i=0;i<4U;++i) {
        int64_t value=pulses[i];
        uint32_t magnitude=(uint32_t)(value<0?-value:value);
        uint8_t dir=value>=0?forward[i]:(uint8_t)(1U-forward[i]);
        uint8_t frame[]={(uint8_t)(i+1U),0xFD,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acceleration,
            (uint8_t)(magnitude>>24),(uint8_t)(magnitude>>16),(uint8_t)(magnitude>>8),(uint8_t)magnitude,
            2,1,FRAME_TAIL};
        if(send_command(frame,sizeof(frame))!=HAL_OK) return HAL_ERROR;
    }
    return trigger();
}
HAL_StatusTypeDef ZDT_Motor_SetWheelSpeedsSynchronous(const int16_t rpm[4],uint8_t acceleration)
{
    if(!rpm) return HAL_ERROR;
    /* 先校验整组参数，不能发到一半才发现后续轮速无效。 */
    for(unsigned i=0;i<4U;++i) {
        int32_t value=rpm[i];
        if((uint32_t)(value<0?-value:value)>STEPPER_MAX_SPEED_RPM) return HAL_ERROR;
    }
    for(unsigned i=0;i<4U;++i) {
        int32_t value=rpm[i]; uint16_t magnitude=(uint16_t)(value<0?-value:value);
        uint8_t dir=value>=0?forward[i]:(uint8_t)(1U-forward[i]);
        uint8_t frame[]={(uint8_t)(i+1U),0xF6,dir,(uint8_t)(magnitude>>8),(uint8_t)magnitude,acceleration,1,FRAME_TAIL};
        if(send_command(frame,sizeof(frame))!=HAL_OK) return HAL_ERROR;
    }
    return trigger();
}
