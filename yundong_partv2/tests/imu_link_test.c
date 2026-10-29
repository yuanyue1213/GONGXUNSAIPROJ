#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "imu_link.h"
static UART_HandleTypeDef uart;
static uint8_t *rx;
static uint32_t tick;
static uint16_t output,rate;
static unsigned queries,writes,baud_changes;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ assert(u==&uart&&n==1); rx=p; return HAL_OK; }
static void packet(uint8_t kind,const uint16_t data[4])
{
    uint8_t f[11]={0x55,kind};
    for(unsigned i=0;i<4;++i) { f[2+i*2]=(uint8_t)data[i]; f[3+i*2]=(uint8_t)(data[i]>>8); }
    for(unsigned i=0;i<10;++i) f[10]=(uint8_t)(f[10]+f[i]);
    for(unsigned i=0;i<11;++i) { *rx=f[i]; assert(ImuLink_RxCallback(&uart)); }
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t timeout)
{
    assert(u==&uart&&n==5&&timeout==20&&p[0]==0xFF&&p[1]==0xAA);
    uint16_t value=(uint16_t)(p[3]|((uint16_t)p[4]<<8));
    if(p[2]==0x27) { ++queries; const uint16_t data[4]={output,rate,2,0}; packet(0x5F,data); }
    else { ++writes; if(p[2]==2) output=value; if(p[2]==3) rate=value; }
    return HAL_OK;
}
HAL_StatusTypeDef ImuLink_SetBaud(UART_HandleTypeDef *u,uint32_t baud)
{ assert(u==&uart&&baud==9600); ++baud_changes; return HAL_OK; }
static void samples(uint16_t raw_yaw,unsigned count)
{
    const uint16_t angles[4]={0,0,raw_yaw,0},rates[4]={0,0,0,0};
    for(unsigned i=0;i<count;++i) { packet(0x52,rates); packet(0x53,angles); }
    ImuLink_Process();
}
int main(void)
{
    for(unsigned changed=0;changed<2;++changed) {
        tick=queries=writes=baud_changes=0; output=changed?0:0x0C; rate=changed?0:6;
        ImuLink_Init(&uart); ImuLink_StartConfiguration();
        ImuSample s; assert(!ImuLink_Read(&s));
        for(unsigned i=0;i<70;++i) {
            tick+=100; ImuLink_Process();
            if(output==0x0C&&rate==6) samples(0x4000,1);
        }
        samples(0x4000,10); assert(ImuLink_Read(&s)&&s.yaw==-90&&s.rate==0);
        assert(queries>=2&&baud_changes==1&&output==0x0C&&rate==6);
        assert(changed?writes>0:writes==0);
        tick+=201; assert(!ImuLink_Read(&s));
        samples(0x7F00,12); assert(ImuLink_Read(&s));
        float before=s.yaw; tick+=50;
        samples(0x8100,1); assert(ImuLink_Read(&s)&&s.yaw<before&&before-s.yaw<5);
        before=s.yaw; assert(before< -180);
        tick+=201; assert(!ImuLink_Read(&s));
        samples(0x8200,12); /* 过期后的首帧清空旧计数，再积累两类有效样本。 */
        assert(ImuLink_Read(&s)&&s.yaw< -180&&fabsf(s.yaw-before)<5);
        before=s.yaw;
        assert(ImuLink_ErrorCallback(&uart)); assert(!ImuLink_Read(&s)); ImuLink_Process();
        samples(0x8300,10);
        assert(ImuLink_Read(&s)&&s.yaw< -180&&fabsf(s.yaw-before)<5);
    }
    puts("PASS: live IMU startup query/config/save, angle/rate freshness, yaw unwrap and UART error handling without diagnostic interfaces");
}
