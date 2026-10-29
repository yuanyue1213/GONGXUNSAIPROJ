#include "robot_config.h"
/* 机械臂相对位置驱动：Emm 使用微步脉冲，X 使用 0.1 度，不能共用同一报文。
 * 仅确认命令接收，不查询位置或到位状态，不自动重发运动命令。 */
#include "lift_motor.h"
#include "stepper_limits.h"
#include <stdbool.h>
#include <string.h>
#define FRAME_TAIL 0x6BU
static UART_HandleTypeDef *s_uart;
static uint8_t s_firmware[2]; /* 0=未知，1=Emm，2=X */
static bool s_enabled[2];

/* 首次运动读取配置并缓存固件类型；滑动扫描跳过回显及残留字节。 */
static uint8_t firmware(uint8_t id)
{
    if(!s_uart||id<1U||id>2U) return 0;
    if(s_firmware[id-1U]) return s_firmware[id-1U];
    uint8_t query[]={id,0x42,0x6C,FRAME_TAIL},response[37];
    if(HAL_UART_Transmit(s_uart,query,sizeof(query),MOTOR_ACK_TIMEOUT_MS)!=HAL_OK) return 0;
    unsigned count=0,scanned=0;
    uint32_t started=HAL_GetTick();
    while(scanned++<128U) {
        uint32_t elapsed=HAL_GetTick()-started;
        if(elapsed>=70U||HAL_UART_Receive(s_uart,&response[count],1U,70U-elapsed)!=HAL_OK) break;
        ++count;
        while(count) {
            if(response[0]!=id||(count>=2U&&response[1]!=0x42)) {
                memmove(response,response+1,--count); continue;
            }
            if(count<3U) break;
            unsigned length=response[2];
            if(length!=0x21U&&length!=0x25U) { memmove(response,response+1,--count); continue; }
            if(count<length) break;
            if(response[length-1U]==FRAME_TAIL&&response[3]==(length==0x21U?0x15U:0x18U)) {
                s_firmware[id-1U]=length==0x21U?1U:2U;
                return s_firmware[id-1U];
            }
            memmove(response,response+1,--count);
        }
    }
    return 0;
}
static HAL_StatusTypeDef send_command(const uint8_t *frame,uint16_t length)
{
    if(!s_uart) return HAL_ERROR;
    HAL_StatusTypeDef result=HAL_UART_Transmit(s_uart,(uint8_t *)frame,length,MOTOR_ACK_TIMEOUT_MS);
    if(result!=HAL_OK) return result;
    uint8_t response[4]; unsigned count=0,scanned=0;
    uint32_t started=HAL_GetTick();
    while(scanned++<20U) {
        uint32_t elapsed=HAL_GetTick()-started;
        if(elapsed>=MOTOR_ACK_TIMEOUT_MS||HAL_UART_Receive(s_uart,&response[count],1U,MOTOR_ACK_TIMEOUT_MS-elapsed)!=HAL_OK) break;
        if(++count<4U) continue;
        if(response[0]==frame[0]&&response[1]==frame[1]&&response[3]==FRAME_TAIL)
            return response[2]==0x02U?HAL_OK:HAL_ERROR;
        memmove(response,response+1,3); count=3;
    }
    return HAL_TIMEOUT;
}
void LiftMotor_Init(UART_HandleTypeDef *uart)
{
    s_uart=uart;
    memset(s_firmware,0,sizeof(s_firmware)); memset(s_enabled,0,sizeof(s_enabled));
}
static HAL_StatusTypeDef move_angle(uint8_t id,uint8_t direction,uint32_t angle,uint16_t rpm,uint32_t ppr)
{
    if(!angle||angle>36000U||!rpm||rpm>STEPPER_MAX_SPEED_RPM||ppr<200U||ppr>51200U) return HAL_ERROR;
    uint8_t type=firmware(id);
    if(!type) return HAL_ERROR;
    uint32_t pulses=(uint32_t)(((uint64_t)angle*ppr+1800U)/3600U);
    if(type==1U&&!pulses) return HAL_ERROR;
    uint8_t enable[]={id,0xF3,0xAB,1,0,FRAME_TAIL};
    if(send_command(enable,sizeof(enable))!=HAL_OK) return HAL_ERROR;
    s_enabled[id-1U]=true;
    uint8_t emm[]={id,0xFD,direction,(uint8_t)(rpm>>8),(uint8_t)rpm,(uint8_t)MISSION_ARM_EMM_ACCEL,
        (uint8_t)(pulses>>24),(uint8_t)(pulses>>16),(uint8_t)(pulses>>8),(uint8_t)pulses,2,0,FRAME_TAIL};
    uint16_t speed=rpm*10U;
    uint8_t x[]={id,0xFD,direction,(uint8_t)(ARM_X_ACCEL_RPM_S>>8),(uint8_t)ARM_X_ACCEL_RPM_S,(uint8_t)(ARM_X_ACCEL_RPM_S>>8),(uint8_t)ARM_X_ACCEL_RPM_S,(uint8_t)(speed>>8),(uint8_t)speed,
        (uint8_t)(angle>>24),(uint8_t)(angle>>16),(uint8_t)(angle>>8),(uint8_t)angle,2,0,FRAME_TAIL};
    return type==2U?send_command(x,sizeof(x)):send_command(emm,sizeof(emm));
}
HAL_StatusTypeDef LiftMotor_MoveAngle(char direction,uint32_t angle,uint16_t rpm,uint32_t ppr)
{
    if(direction!='U'&&direction!='D') return HAL_ERROR;
    return move_angle(1U,direction=='U'?1U:0U,angle,rpm,ppr);
}
HAL_StatusTypeDef ArmMotor_MoveAngle(char direction,uint32_t angle,uint16_t rpm,uint32_t ppr)
{
    if(direction!='E'&&direction!='C') return HAL_ERROR;
    return move_angle(2U,direction=='E'?0U:1U,angle,rpm,ppr);
}
static HAL_StatusTypeDef stop(uint8_t id)
{
    if(!s_enabled[id-1U]) return HAL_OK;
    uint8_t frame[]={id,0xFE,0x98,0,FRAME_TAIL};
    return send_command(frame,sizeof(frame));
}
HAL_StatusTypeDef LiftMotor_Stop(void) { return stop(1U); }
HAL_StatusTypeDef ArmMotor_StopForeAft(void) { return stop(2U); }
char LiftMotor_FirmwareCode(uint8_t id)
{
    return id>=1U&&id<=2U?"?EX"[s_firmware[id-1U]]:'?';
}
