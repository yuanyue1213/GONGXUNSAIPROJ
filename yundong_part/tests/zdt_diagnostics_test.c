#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "zdt_motor.h"

static UART_HandleTypeDef uart={1,HAL_UART_STATE_READY,0};
static uint8_t wire[160],sent[8];
static unsigned wire_length,offset,sent_length;
static uint32_t tick;
static uint32_t byte_delay;
static unsigned transmissions,faults;
static bool drop_first_position,drop_first_status;
void ZDT_Motor_Fault(uint8_t id,const char *operation)
{ (void)id; (void)operation; ++faults; }
static HAL_StatusTypeDef tx_result,empty_result;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port,uint8_t *data,uint16_t length,uint32_t timeout)
{
    assert(port==&uart&&length<=sizeof(sent)&&timeout==20U);
    memcpy(sent,data,length); sent_length=length; offset=0;
    ++transmissions;
    if(drop_first_position&&data[1]==0x36&&transmissions==1U) offset=wire_length;
    if(drop_first_status&&data[1]==0x3A&&transmissions==1U) offset=wire_length;
    return tx_result;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *port,uint8_t *data,uint16_t length,uint32_t timeout)
{
    assert(port==&uart&&length==1U);
    if(byte_delay>=timeout&&byte_delay) { tick+=timeout; return HAL_TIMEOUT; }
    if(offset<wire_length) { tick+=byte_delay; *data=wire[offset++]; return HAL_OK; }
    tick+=timeout;
    uart.ErrorCode=empty_result==HAL_ERROR?8U:0U;
    return empty_result;
}
static void input(const uint8_t *data,unsigned length)
{
    if(length) memcpy(wire,data,length);
    wire_length=length; tx_result=HAL_OK; empty_result=HAL_TIMEOUT; uart.ErrorCode=0; byte_delay=0;
    transmissions=0; drop_first_position=drop_first_status=false;
}
static ZDT_RawTransaction stop_record(void) { return g_zdt_diagnostics[1].stop; }
int main(void)
{
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
    signal(SIGABRT,exit);
#endif
    ZDT_Motor_Init(&uart,true,20U);
    /* A valid position reply after 40 stale bytes must fit the 20 ms read
     * budget at 115200 baud, rather than hitting the old 32-byte scan cap. */
    uint8_t delayed_position[48]; memset(delayed_position,0xFF,40);
    const uint8_t position_reply[]={2,0x36,0,0,1,0,0,0x6B};
    memcpy(delayed_position+40,position_reply,8);
    int64_t counts=0;
    input(delayed_position,sizeof(delayed_position));
    assert(ZDT_Motor_ReadWheelPosition(2,&counts)==HAL_OK&&counts==65536);
    ZDT_RawTransaction p=g_zdt_diagnostics[1].position;
    assert(p.matched&&p.rx_length==48&&p.tx_length==3&&p.result==HAL_OK);
    assert(memcmp(p.rx,delayed_position,48)==0&&sent[0]==2&&sent[1]==0x36&&sent[2]==0x6B);
    assert(g_zdt_diagnostics[1].stop.attempts==0&&g_zdt_diagnostics[1].status.attempts==0);
    /* Fixed wire vector, independent of wheel-sign normalization. */
    const uint8_t negative_position[]={1,0x36,1,0xFF,0xFF,0xFF,0xFF,0x6B};
    input(negative_position,8);
    assert(ZDT_Motor_ReadWheelPosition(1,&counts)==HAL_OK&&counts==4294967295LL);
    input(position_reply,7); counts=123;
    assert(ZDT_Motor_ReadWheelPosition(2,&counts)==HAL_TIMEOUT&&counts==123);
    p=g_zdt_diagnostics[1].position; assert(p.rx_length==7&&!p.matched);
    input(NULL,0); empty_result=HAL_ERROR;
    assert(ZDT_Motor_ReadWheelPosition(2,&counts)==HAL_ERROR&&counts==123);
    p=g_zdt_diagnostics[1].position; assert(p.rx_result==HAL_ERROR&&p.uart_error==8U);
    input(NULL,0); tx_result=HAL_BUSY;
    assert(ZDT_Motor_ReadWheelPosition(2,&counts)==HAL_BUSY);
    p=g_zdt_diagnostics[1].position; assert(p.tx_result==HAL_BUSY&&p.rx_result==HAL_BUSY&&p.rx_length==0);
    /* The longer scan is still bounded by the original 20 ms deadline. */
    input(delayed_position,sizeof(delayed_position)); byte_delay=1;
    uint32_t started=tick;
    assert(ZDT_Motor_ReadWheelPosition(2,&counts)==HAL_TIMEOUT&&tick-started==40U&&transmissions==2U);
    assert(!g_zdt_diagnostics[1].position.matched);
    input(position_reply,8); drop_first_position=true; faults=0; started=tick;
    assert(ZDT_Motor_ReadWheelPosition(2,&counts)==HAL_OK&&counts==65536);
    assert(transmissions==2U&&tick-started==20U&&faults==0U);
    input(NULL,0); faults=0;
    assert(ZDT_Motor_ReadWheelPosition(2,&counts)==HAL_TIMEOUT&&transmissions==2U&&faults==1U);
    input(NULL,0); tx_result=HAL_ERROR;
    assert(ZDT_Motor_ReadWheelPosition(2,&counts)==HAL_ERROR&&transmissions==1U);
    assert(ZDT_Motor_ReadWheelPosition(0,&counts)==HAL_ERROR);
    assert(ZDT_Motor_ReadWheelPosition(2,NULL)==HAL_ERROR);
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
    assert(transmissions==1U); // Real protection flags (9) must never be retried.
    const uint8_t healthy_status[]={2,0x3A,1,0x6B};
    input(healthy_status,4); drop_first_status=true; faults=0; started=tick;
    assert(ZDT_Motor_ReadStatus(2,&flags)==HAL_OK&&flags==1);
    assert(transmissions==2U&&tick-started==20U&&faults==0U);
    input(NULL,0); faults=0; started=tick; flags=0xA5;
    assert(ZDT_Motor_ReadStatus(2,&flags)==HAL_TIMEOUT&&flags==0xA5);
    assert(transmissions==2U&&tick-started==40U&&faults==1U);
    input(NULL,0); tx_result=HAL_ERROR;
    assert(ZDT_Motor_ReadStatus(2,&flags)==HAL_ERROR&&transmissions==1U);

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
