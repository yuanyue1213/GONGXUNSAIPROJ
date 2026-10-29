#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "zdt_motor.h"
#include "zdt_motor_transport.h"

static UART_HandleTypeDef uart;
static uint8_t *rx;
static uint32_t tick;
static uint8_t frames[16][16];
static unsigned lengths[16],count;
static unsigned fail_id,mode;
uint32_t HAL_GetTick(void) { return tick++; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ assert(u==&uart&&n==1); rx=p; return HAL_OK; }
static void emit(uint8_t b) { *rx=b; assert(ZDT_Motor_RxCallback(&uart)); }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *f,uint16_t n,uint32_t timeout)
{
    assert(u==&uart&&timeout==20&&count<16&&n<=16);
    memcpy(frames[count],f,n); lengths[count++]=n;
    if(f[0]==fail_id) return HAL_ERROR;
    if(!f[0]) return HAL_OK;
    if(mode==3) return HAL_OK;
    if(mode==4) { for(unsigned i=0;i<520;++i) emit(0xFF); return HAL_OK; }
    emit(0xAA); /* 跳过不匹配字节，且 ACK 在 TX 返回前抵达。 */
    emit(f[0]); emit(f[1]); emit(mode==2?0xE2:2); emit(0x6B);
    return HAL_OK;
}
static void reset(void) { count=mode=0; fail_id=99; ZDT_Motor_Init(&uart); }
int main(void)
{
    reset(); assert(ZDT_Motor_Enable(1)==HAL_OK&&frames[0][3]==1);
    reset(); int32_t p[4]={3200,-3200,1600,-1600};
    assert(ZDT_Motor_MoveWheelSynchronous(p,1500,10)==HAL_OK&&count==5);
    const uint8_t directions[]={1,1,0,0};
    for(unsigned i=0;i<4;++i) assert(lengths[i]==13&&frames[i][0]==i+1&&frames[i][1]==0xFD&&frames[i][2]==directions[i]&&frames[i][3]==5&&frames[i][4]==0xDC&&frames[i][10]==2&&frames[i][11]==1);
    assert(frames[0][8]==12&&frames[0][9]==128&&memcmp(frames[4],(uint8_t[]){0,0xFF,0x66,0x6B},4)==0);
    reset(); int16_t speed[4]={60,-60,0,1500};
    assert(ZDT_Motor_SetWheelSpeedsSynchronous(speed,240)==HAL_OK&&count==5);
    assert(frames[0][1]==0xF6&&frames[2][3]==0&&frames[2][4]==0&&frames[3][3]==5&&frames[3][4]==0xDC);
    reset(); speed[3]=1501; assert(ZDT_Motor_SetWheelSpeedsSynchronous(speed,240)==HAL_ERROR&&count==0);
    assert(ZDT_Motor_MoveWheelSynchronous(p,1501,10)==HAL_ERROR&&count==0);
    reset(); fail_id=2; assert(ZDT_Motor_MoveWheelSynchronous(p,300,10)!=HAL_OK&&count==2);
    reset(); mode=2; assert(ZDT_Motor_Stop(1)!=HAL_OK&&count==1);
    reset(); mode=3; assert(ZDT_Motor_Enable(1)!=HAL_OK);
    reset(); mode=4; assert(ZDT_Motor_Enable(1)!=HAL_OK); /* 接收溢出拒绝整帧。 */
    reset(); tick=UINT32_MAX-5; assert(ZDT_Motor_Stop(4)==HAL_OK&&frames[0][3]==0);
    puts("PASS: live wheel UART transport, early ACK, synchronous FD/F6 encoding, 1500 RPM, no trigger after failure and overflow rejection");
}
