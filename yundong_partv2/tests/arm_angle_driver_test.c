#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "lift_motor.h"

static UART_HandleTypeDef uart;
static uint32_t tick;
static uint8_t response[80],last[16],lengths[2];
static unsigned received,total,moves,queries[2],tx_count,last_length;
static uint8_t reject,fail_tx,timeout_ack;
static bool bad_config,echo;
uint32_t HAL_GetTick(void) { return tick++; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *f,uint16_t n,uint32_t timeout)
{
    (void)timeout; assert(u==&uart&&f[0]>=1&&f[0]<=2); ++tx_count; received=total=0;
    if(f[1]==fail_tx) return HAL_ERROR;
    if(f[1]==0x42) {
        ++queries[f[0]-1]; unsigned size=lengths[f[0]-1];
        if(echo) { memcpy(response,f,n); total=n; }
        uint8_t *p=response+total; memset(p,0,size);
        p[0]=f[0]; p[1]=0x42; p[2]=(uint8_t)size;
        p[3]=bad_config?0:size==0x21?0x15:0x18; p[size-1]=0x6B; total+=size;
    } else {
        assert(f[1]==0xF3||f[1]==0xFD||f[1]==0xFE);
        if(f[1]==0xFD) { ++moves; memcpy(last,f,n); last_length=n; }
        if(f[1]!=timeout_ack) {
            response[0]=f[0]; response[1]=f[1]; response[2]=f[1]==reject?0xE2:2; response[3]=0x6B; total=4;
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t timeout)
{
    assert(u==&uart&&n==1&&timeout<=70);
    if(received==total) { tick+=timeout; return HAL_TIMEOUT; }
    *p=response[received++]; return HAL_OK;
}
static void reset(unsigned size)
{
    lengths[0]=lengths[1]=(uint8_t)size; moves=queries[0]=queries[1]=tx_count=0;
    reject=fail_tx=timeout_ack=0; bad_config=echo=false; tick=0; LiftMotor_Init(&uart);
}
int main(void)
{
    for(unsigned size=0x21;size<=0x25;size+=4) {
        reset(size); echo=true;
        assert(LiftMotor_Stop()==HAL_OK&&tx_count==0);
        assert(LiftMotor_MoveAngle('U',900,1500,3200)==HAL_OK&&last[0]==1&&last[2]==1);
        assert(LiftMotor_FirmwareCode(1)==(size==0x21?'E':'X'));
        if(size==0x21) assert(last_length==13&&last[3]==5&&last[4]==0xDC&&last[8]==3&&last[9]==0x20&&last[10]==2);
        else assert(last_length==16&&last[7]==0x3A&&last[8]==0x98&&last[11]==3&&last[12]==0x84&&last[13]==2);
        assert(LiftMotor_MoveAngle('D',900,300,3200)==HAL_OK&&last[2]==0&&queries[0]==1);
        assert(ArmMotor_MoveAngle('E',900,300,3200)==HAL_OK&&last[0]==2&&last[2]==0);
        assert(ArmMotor_MoveAngle('C',900,1,3200)==HAL_OK&&last[2]==1&&queries[1]==1);
        assert(LiftMotor_Stop()==HAL_OK&&ArmMotor_StopForeAft()==HAL_OK);
        unsigned before=tx_count;
        assert(ArmMotor_MoveAngle('E',900,1501,3200)==HAL_ERROR);
        assert(ArmMotor_MoveAngle('X',900,300,3200)==HAL_ERROR);
        assert(LiftMotor_MoveAngle('U',0,300,3200)==HAL_ERROR);
        assert(LiftMotor_MoveAngle('U',900,300,0)==HAL_ERROR&&tx_count==before);
    }
    reset(0x21); lengths[1]=0x25;
    assert(LiftMotor_MoveAngle('U',900,300,3200)==HAL_OK&&ArmMotor_MoveAngle('E',900,300,3200)==HAL_OK);
    assert(LiftMotor_FirmwareCode(1)=='E'&&LiftMotor_FirmwareCode(2)=='X');
    reset(0x21); reject=0xF3;
    assert(LiftMotor_MoveAngle('U',900,300,3200)!=HAL_OK&&moves==0);
    reset(0x25); timeout_ack=0xFD;
    assert(ArmMotor_MoveAngle('E',900,300,3200)!=HAL_OK&&moves==1); /* 相对运动绝不重发。 */
    reset(0x21); fail_tx=0x42;
    assert(LiftMotor_MoveAngle('U',900,300,3200)!=HAL_OK&&moves==0);
    reset(0x25); bad_config=true;
    assert(ArmMotor_MoveAngle('E',900,300,3200)!=HAL_OK&&moves==0);
    reset(0x21); tick=UINT32_MAX-8;
    assert(LiftMotor_MoveAngle('U',900,300,3200)==HAL_OK);
    puts("PASS: minimal arm driver, ID1/ID2 Emm/X relative frames, cached detection, direction, speed limits, command failures and no motion retry");
}
