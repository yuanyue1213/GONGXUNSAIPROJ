#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motion_control.h"
#include "zdt_motor.h"
#include "zdt_motor_transport.h"
#include "imu_link.h"

static UART_HandleTypeDef uart;
static uint8_t *rx;
static uint8_t frames[16][16];
static unsigned lengths[16],count;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick++; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ assert(u==&uart&&n==1); rx=p; return HAL_OK; }
static void emit(uint8_t b) { *rx=b; assert(ZDT_Motor_RxCallback(&uart)); }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *f,uint16_t n,uint32_t timeout)
{
    (void)timeout; assert(u==&uart&&count<16&&n<=16);
    memcpy(frames[count],f,n); lengths[count++]=n;
    if(f[0]) { emit(f[0]); emit(f[1]); emit(2); emit(0x6B); }
    return HAL_OK;
}
bool ImuLink_Read(ImuSample *s) { (void)s; assert(!"Straight move must not call IMU"); return false; }
int main(void)
{
    ZDT_Motor_Init(&uart); Motion_Init(NULL);
    assert(Motion_Move('F',1800,1500));
    assert(count==9);
    const uint8_t dir[]={1,0,0,1};
    const char *names[]={"left-front","right-front","right-rear","left-rear"};
    for(unsigned i=0;i<4;++i) {
        uint8_t *f=frames[4+i];
        assert(lengths[4+i]==13&&f[0]==i+1&&f[1]==0xFD&&f[2]==dir[i]);
        assert(f[3]==5&&f[4]==0xDC&&f[5]==10);
        assert(f[6]==0&&f[7]==0&&f[8]==0x58&&f[9]==0x7D&&f[10]==2&&f[11]==1&&f[12]==0x6B);
        printf("ID%u %-12s:",i+1,names[i]);
        for(unsigned j=0;j<13;++j) printf(" %02X",f[j]);
        putchar('\n');
    }
    assert(lengths[8]==4&&!memcmp(frames[8],(uint8_t[]){0,0xFF,0x66,0x6B},4));
    puts("START: 00 FF 66 6B (one broadcast trigger after four position ACKs)");
    Motion_Stop(); assert(count==13);
    for(unsigned i=0;i<4;++i)
        assert(lengths[9+i]==5&&!memcmp(frames[9+i],(uint8_t[]){i+1,0xFE,0x98,0,0x6B},5));
    puts("STOP: individual immediate-stop frames for ID1, ID2, ID3, ID4");
    puts("PASS: actual Motion_Move + wheel mixer + UART encoder/receiver; all four wheels 22653 pulses, 1500 RPM, accel 10, identical position/sync flags");
}
