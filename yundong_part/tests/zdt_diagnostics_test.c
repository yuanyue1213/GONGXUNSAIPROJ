#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "zdt_motor.h"

static UART_HandleTypeDef uart={1,HAL_UART_STATE_READY,0};
static uint8_t wire[48],sent[8];
static unsigned wire_length,offset,sent_length;
static uint32_t tick;
static HAL_StatusTypeDef tx_result,empty_result;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port,uint8_t *data,uint16_t length,uint32_t timeout)
{
    assert(port==&uart&&length<=sizeof(sent)&&timeout==20U);
    memcpy(sent,data,length); sent_length=length; offset=0;
    return tx_result;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *port,uint8_t *data,uint16_t length,uint32_t timeout)
{
    assert(port==&uart&&length==1U);
    if(offset<wire_length) { *data=wire[offset++]; return HAL_OK; }
    tick+=timeout;
    uart.ErrorCode=empty_result==HAL_ERROR?8U:0U;
    return empty_result;
}
static void input(const uint8_t *data,unsigned length)
{
    if(length) memcpy(wire,data,length);
    wire_length=length; tx_result=HAL_OK; empty_result=HAL_TIMEOUT; uart.ErrorCode=0;
}
static ZDT_RawTransaction stop_record(void) { return g_zdt_diagnostics[1].stop; }
int main(void)
{
    ZDT_Motor_Init(&uart,true,20U);
    const uint8_t accepted[]={2,0xFE,2,0x6B};
    const uint8_t stop_tx[]={2,0xFE,0x98,0,0x6B};
    input(accepted,sizeof(accepted));
    assert(ZDT_Motor_Stop(2,false)==HAL_OK);
    ZDT_RawTransaction s=stop_record();
    assert(s.attempts==1&&s.matched==1&&s.result==HAL_OK&&s.tx_result==HAL_OK&&s.rx_result==HAL_OK);
    assert(s.tx_length==5&&s.rx_length==4&&memcmp(s.tx,stop_tx,5)==0&&memcmp(s.rx,accepted,4)==0);
    assert(sent_length==5&&memcmp(sent,stop_tx,5)==0);

    const uint8_t rejected[]={2,0xFE,0xE2,0x6B};
    input(rejected,4); assert(ZDT_Motor_Stop(2,false)==HAL_ERROR);
    s=stop_record(); assert(s.attempts==2&&!s.matched&&s.rx_length==4&&memcmp(s.rx,rejected,4)==0);
    assert(s.rx_result==HAL_OK&&s.result==HAL_ERROR&&s.finished_tick==s.started_tick);
    const uint8_t malformed[]={2,0,0xEE,0x6B};
    input(malformed,4); assert(ZDT_Motor_Stop(2,false)==HAL_ERROR);
    s=stop_record(); assert(s.rx_length==4&&memcmp(s.rx,malformed,4)==0);
    const uint8_t other_error_then_ack[]={3,0xFE,0xE2,0x6B,2,0xFE,2,0x6B};
    input(other_error_then_ack,sizeof(other_error_then_ack));
    assert(ZDT_Motor_Stop(2,false)==HAL_OK); // Foreign rejection must not reject this motor.
    s=stop_record(); assert(s.matched&&s.rx_length==8&&memcmp(s.rx,other_error_then_ack,8)==0);
    const uint8_t noise_partial[]={0xFF,3,0xFE,2,0x6B,2,0xFE};
    input(noise_partial,sizeof(noise_partial)); assert(ZDT_Motor_Stop(2,false)==HAL_TIMEOUT);
    s=stop_record(); assert(s.rx_length==sizeof(noise_partial)&&memcmp(s.rx,noise_partial,sizeof(noise_partial))==0);

    uint32_t stop_attempts=s.attempts;
    uint8_t flags;
    const uint8_t status[]={2,0x3A,9,0x6B};
    input(status,4); assert(ZDT_Motor_ReadStatus(2,&flags)==HAL_OK&&flags==9);
    ZDT_RawTransaction q=g_zdt_diagnostics[1].status;
    assert(q.attempts==1&&q.tx_length==3&&q.rx_length==4&&q.status_flags==9&&q.matched);
    assert(memcmp(q.rx,status,4)==0&&stop_record().attempts==stop_attempts);
    assert(memcmp(stop_record().rx,noise_partial,sizeof(noise_partial))==0);

    input(NULL,0); assert(ZDT_Motor_Stop(2,false)==HAL_TIMEOUT);
    s=stop_record(); assert(s.rx_length==0&&s.rx_result==HAL_TIMEOUT&&!s.matched);
    input(NULL,0); empty_result=HAL_ERROR; assert(ZDT_Motor_Stop(2,false)==HAL_TIMEOUT);
    s=stop_record(); assert(s.rx_result==HAL_ERROR&&s.uart_error==8U);
    input(NULL,0); tx_result=HAL_ERROR; assert(ZDT_Motor_Stop(2,false)==HAL_ERROR);
    s=stop_record(); assert(s.tx_result==HAL_ERROR&&s.rx_result==HAL_BUSY&&s.rx_length==0);
    assert((g_zdt_diagnostics[1].publish_sequence&1U)==0U);
    assert(g_zdt_diagnostics[0].stop.attempts==0&&g_zdt_diagnostics[2].stop.attempts==0);

    input(NULL,0); tx_result=HAL_ERROR; assert(ZDT_Motor_ReadStatus(2,&flags)==HAL_ERROR);
    q=g_zdt_diagnostics[1].status; assert(q.tx_result==HAL_ERROR&&q.rx_length==0&&q.rx_result==HAL_BUSY);
    ZDT_Motor_Init(&uart,false,20U); input(NULL,0);
    assert(ZDT_Motor_Stop(2,false)==HAL_OK&&offset==0);
    s=stop_record(); assert(s.attempts==1&&s.rx_length==0&&s.rx_result==HAL_BUSY&&!s.matched);
    puts("PASS: raw STOP/status capture, explicit E2/EE errors, foreign reply isolation, partial bytes, no reply, TX/RX errors, isolated snapshots");
}
