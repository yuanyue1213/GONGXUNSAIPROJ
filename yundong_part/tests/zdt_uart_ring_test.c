#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "zdt_motor.h"
#include "zdt_motor_transport.h"

static UART_HandleTypeDef uart={1,HAL_UART_STATE_READY,0};
static uint8_t *armed_byte;
static uint32_t tick;
static unsigned mode,transmissions,fail_arm;
uint32_t HAL_GetTick(void) { return tick++; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *byte,uint16_t length)
{
    assert(u==&uart&&length==1);
    if(fail_arm) { --fail_arm; return HAL_ERROR; }
    if(u->RxState==HAL_UART_STATE_BUSY_RX) return HAL_BUSY;
    armed_byte=byte; u->RxState=HAL_UART_STATE_BUSY_RX; u->ErrorCode=0;
    return HAL_OK;
}
static void incoming(uint8_t byte)
{
    assert(uart.RxState==HAL_UART_STATE_BUSY_RX);
    *armed_byte=byte; uart.RxState=HAL_UART_STATE_READY;
    assert(ZDT_Motor_RxCallback(&uart));
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *b,uint16_t n,uint32_t ms)
{
    (void)u;(void)b;(void)n;(void)ms;
    assert(!"Hardware transport must not call polling HAL_UART_Receive");
    return HAL_ERROR;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *b,uint16_t n,uint32_t ms)
{
    assert(u==&uart&&n>=3&&ms==20); ++transmissions;
    if(mode==1U) return HAL_OK; // no reply
    if(mode==2U&&transmissions==1U) { // error abort and re-arm, then retry
        uart.ErrorCode=8; uart.RxState=HAL_UART_STATE_READY;
        assert(ZDT_Motor_ErrorCallback(&uart)); return HAL_OK;
    }
    if(mode==3U) { for(unsigned i=0;i<520;++i) incoming(0xFF); return HAL_OK; }
    /* Entire reply arrives before TX returns, including noise/foreign frames. */
    incoming(0xFF); incoming(4); incoming(0x3A); incoming(1); incoming(0x6B);
    incoming(b[0]); incoming(b[1]);
    if(b[1]==0x36) {
        const uint8_t tail[]={0,0,1,0,0,0x6B};
        for(unsigned i=0;i<sizeof(tail);++i) incoming(tail[i]);
    } else { incoming(mode==4U?9:2); incoming(0x6B); }
    return HAL_OK;
}
static void reset(unsigned next_mode)
{
    uart.RxState=HAL_UART_STATE_READY; uart.ErrorCode=0;
    tick=0; transmissions=0; mode=next_mode; fail_arm=0;
    ZDT_Motor_Init(&uart,true,40U);
}
int main(void)
{
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR); signal(SIGABRT,exit);
#endif
    reset(0);
    /* Simulate 100 bytes received while main loop handles another peripheral.
     * They are buffered, then discarded only before sending the new request. */
    for(unsigned i=0;i<100;++i) incoming(0xAA);
    uint8_t flags=0; int64_t position=0;
    assert(ZDT_Motor_ReadStatus(3,&flags)==HAL_OK&&flags==2);
    assert(g_zdt_rx_diagnostics.bytes>100&&g_zdt_rx_diagnostics.overflows==0);
    assert(ZDT_Motor_ReadWheelPosition(2,&position)==HAL_OK&&position==65536);
    assert(ZDT_Motor_Stop(2,false)==HAL_OK);
    assert(g_zdt_diagnostics[2].status.matched);
    reset(2); assert(ZDT_Motor_ReadStatus(3,&flags)==HAL_OK&&transmissions==2);
    assert(g_zdt_rx_diagnostics.uart_errors==1&&g_zdt_rx_diagnostics.last_uart_error==8);
    reset(3); assert(ZDT_Motor_ReadStatus(3,&flags)==HAL_ERROR&&transmissions==2);
    assert(g_zdt_rx_diagnostics.overflows>0&&g_zdt_diagnostics[2].status.uart_error==0x80000000UL);
    reset(1); flags=0xA5;
    assert(ZDT_Motor_ReadStatus(3,&flags)==HAL_TIMEOUT&&flags==0xA5&&transmissions==2);
    reset(4); assert(ZDT_Motor_ReadStatus(3,&flags)==HAL_OK&&flags==9&&transmissions==1);
    UART_HandleTypeDef other={2,HAL_UART_STATE_READY,0};
    assert(!ZDT_Motor_RxCallback(&other)&&!ZDT_Motor_ErrorCallback(&other));
    reset(0); fail_arm=1;
    incoming(0); // callback re-arm fails; the next request must recover
    assert(ZDT_Motor_ReadStatus(2,&flags)==HAL_OK&&g_zdt_rx_diagnostics.arm_failures==1);
    puts("PASS: continuous motor RX, early replies, stale bytes, UART recovery, overflow, timeout and fault status");
}
