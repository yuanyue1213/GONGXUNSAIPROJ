#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "imu_link.h"

static UART_HandleTypeDef uart={7,HAL_UART_STATE_READY};
static uint32_t now,host_baud,device_baud;
static uint32_t device_ready_at;
static uint16_t output,rate,baud_code;
static uint8_t *rx_byte,reply[11];
static unsigned saves,writes,queries,baud_changes;
static bool pending,unlocked,connected,bad_checksum,ignore_writes,no_stream,fail_tx,fail_baud;
static bool ignore_rate;
static uint8_t reply_kind;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef ImuLink_SetBaud(UART_HandleTypeDef *port,uint32_t baud)
{
    assert(port==&uart); if(fail_baud) return HAL_ERROR;
    host_baud=baud; ++baud_changes; uart.RxState=HAL_UART_STATE_READY;
    pending=false; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *port,uint8_t *data,uint16_t length)
{
    assert(port==&uart&&length==1);
    if(port->RxState==HAL_UART_STATE_BUSY_RX) return HAL_BUSY;
    rx_byte=data; port->RxState=HAL_UART_STATE_BUSY_RX; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port,uint8_t *data,uint16_t length,uint32_t timeout)
{
    assert(port==&uart&&length==5&&timeout==20&&data[0]==0xFF&&data[1]==0xAA);
    if(fail_tx) return HAL_ERROR;
    if(!connected||host_baud!=device_baud||now<device_ready_at) return HAL_OK;
    uint16_t value=(uint16_t)data[3]|((uint16_t)data[4]<<8);
    if(data[2]==0x27) {
        static const uint8_t query[]={0xFF,0xAA,0x27,0x02,0x00};
        assert(memcmp(data,query,sizeof(query))==0);
        assert(value==2); ++queries;
        uint16_t registers[]={output,rate,baud_code,0};
        memset(reply,0,sizeof(reply)); reply[0]=0x55; reply[1]=reply_kind;
        for(unsigned i=0;i<4;++i) { reply[2+i*2]=(uint8_t)registers[i]; reply[3+i*2]=(uint8_t)(registers[i]>>8); }
        for(unsigned i=0;i<10;++i) reply[10]=(uint8_t)(reply[10]+reply[i]);
        if(bad_checksum) ++reply[10];
        pending=true;
    } else if(data[2]==0x69) {
        assert(value==0xB588); unlocked=true;
    } else {
        assert(unlocked);
        if(data[2]==0x00) { assert(value==0); ++saves; unlocked=false; }
        else {
            ++writes;
            assert(data[2]==2||data[2]==3); // No baud, calibration or reset writes.
            assert((data[2]==2&&value==0x0C)||(data[2]==3&&value==6));
            unlocked=false; // Require a new unlock for each write, as in the official SDK helpers.
            if(!ignore_writes) {
                if(data[2]==2) output=value;
                if(data[2]==3&&!ignore_rate) rate=value;
            }
        }
    }
    return HAL_OK;
}
static void bytes(const uint8_t *data,unsigned length)
{
    for(unsigned i=0;i<length;++i) {
        assert(uart.RxState==HAL_UART_STATE_BUSY_RX);
        *rx_byte=data[i]; uart.RxState=HAL_UART_STATE_READY; assert(ImuLink_RxCallback(&uart));
    }
}
static void stream(uint8_t kind)
{
    uint8_t packet[11]={0x55,kind,0,0,0,0,0,0,0,0,0};
    for(unsigned i=0;i<10;++i) packet[10]=(uint8_t)(packet[10]+packet[i]);
    bytes(packet,sizeof(packet));
}
static void step(void)
{
    now+=10;
    if(pending) { pending=false; bytes(reply,sizeof(reply)); }
    unsigned period=rate==7?50:100;
    if(connected&&!no_stream&&host_baud==device_baud&&now>=device_ready_at&&now%period==0) {
        if(output&4U) stream(0x52);
        if(output&8U) stream(0x53);
    }
    ImuLink_Process();
}
static void run(unsigned milliseconds) { for(unsigned i=0;i<milliseconds/10;++i) step(); }
static void boot(void)
{
    now=0; host_baud=9600; pending=unlocked=false; saves=writes=queries=baud_changes=0;
    uart.RxState=HAL_UART_STATE_READY; ImuLink_Init(&uart); ImuLink_StartConfiguration();
}
static void reset_device(uint32_t baud,uint16_t code,uint16_t mask,uint16_t frequency)
{
    device_baud=baud; baud_code=code; output=mask; rate=frequency;
    connected=true; bad_checksum=ignore_writes=no_stream=fail_tx=fail_baud=ignore_rate=false; reply_kind=0x5F;
    device_ready_at=0;
    boot();
}
static void test_wire_frames(void)
{
    /* Fixed wire bytes, independently calculated from the documented scales
     * and checksum; do not generate these with the parser's own formulas. */
    static const uint8_t gyro_positive[]={0x55,0x52,0,0,0,0,0,0x40,0,0,0xE7};
    static const uint8_t gyro_negative[]={0x55,0x52,0,0,0,0,0,0xC0,0,0,0x67};
    static const uint8_t angle_positive[]={0x55,0x53,0,0,0,0,0,0x40,0,0,0xE8};
    static const uint8_t angle_negative[]={0x55,0x53,0,0,0,0,0,0xC0,0,0,0x68};
    static const uint8_t noise[]={0x00,0xFF,0xAA};
    ImuSample sample;
    ImuDiagnostics before,after;
    reset_device(9600,2,12,6); run(2500);
    assert(ImuLink_Read(&sample));
    ImuLink_Diagnostics(&before);
    bytes(noise,sizeof(noise));
    bytes(angle_positive,5); ImuLink_Process();
    assert(ImuLink_Read(&sample)&&sample.yaw==0.0f); // Incomplete frame must wait.
    bytes(angle_positive+5,6); bytes(gyro_positive,sizeof(gyro_positive)); ImuLink_Process();
    assert(ImuLink_Read(&sample)&&sample.yaw==90.0f&&sample.rate==1000.0f);
    bytes(angle_negative,sizeof(angle_negative)); bytes(gyro_negative,sizeof(gyro_negative)); ImuLink_Process();
    assert(ImuLink_Read(&sample)&&sample.yaw==-90.0f&&sample.rate==-1000.0f);
    ImuLink_Diagnostics(&after);
    assert(after.rx_bytes-before.rx_bytes==sizeof(noise)+44U);
    assert(after.valid_frames-before.valid_frames==4U);

    uint8_t corrupt[11]; memcpy(corrupt,angle_positive,sizeof(corrupt));
    corrupt[4]=0x55; // Embedded header exercises resynchronization after bad SUM.
    bytes(corrupt,sizeof(corrupt)); bytes(angle_positive,sizeof(angle_positive)); ImuLink_Process();
    assert(ImuLink_Read(&sample)&&sample.yaw==90.0f);
    ImuLink_Diagnostics(&after); assert(after.bad_checksums>before.bad_checksums);

    UART_HandleTypeDef other={4,HAL_UART_STATE_READY};
    ImuLink_Diagnostics(&before);
    assert(!ImuLink_RxCallback(&other)&&!ImuLink_ErrorCallback(&other));
    ImuLink_Diagnostics(&after); assert(after.rx_bytes==before.rx_bytes&&after.uart_errors==before.uart_errors);
    uart.RxState=HAL_UART_STATE_READY; // HAL ends a blocking RX error before callback.
    assert(ImuLink_ErrorCallback(&uart)); ImuLink_Process();
    assert(!ImuLink_Read(&sample)); // Error invalidates old samples.
    run(1100); assert(ImuLink_Read(&sample));
    ImuLink_Diagnostics(&after); assert(after.uart_errors==before.uart_errors+1U);
}
int main(void)
{
    test_wire_frames();
    ImuSample sample;
    reset_device(9600,2,2,6);
    assert(!ImuLink_Read(&sample)&&strcmp(ImuLink_Status(),"INIT_SEARCH")==0);
    run(4000);
    assert(ImuLink_Read(&sample)&&strcmp(ImuLink_Status(),"READY")==0);
    assert(host_baud==9600&&device_baud==9600&&output==12&&rate==6&&writes==2&&saves==1);
    unsigned before=baud_changes; ImuLink_StartConfiguration(); run(300);
    assert(baud_changes==before&&saves==1); // No repeated initialization during operation.
    boot(); run(2500); assert(ImuLink_Read(&sample)&&saves==0&&writes==0); // Correct config skips EEPROM writes.
    now+=201; assert(!ImuLink_Read(&sample)); // Usual motion freshness rule still applies.

    reset_device(115200,6,12,8); run(5000);
    assert(!ImuLink_Read(&sample)&&host_baud==9600&&device_baud==115200&&writes==0&&saves==0);
    reset_device(9600,2,0,13); run(4000);
    assert(ImuLink_Read(&sample)&&saves==1&&writes==2); // Read registers even if output is disabled.
    reset_device(9600,2,12,6); run(2500);
    assert(ImuLink_Read(&sample)&&saves==0&&writes==0);
    reset_device(9600,2,12,7); run(3000);
    assert(ImuLink_Read(&sample)&&saves==1&&rate==6);

    reset_device(9600,2,12,6); device_ready_at=3200; run(5000);
    assert(ImuLink_Read(&sample)&&baud_changes==2U&&writes==0); // Late startup, bounded retry at the SAME baud.
    ImuDiagnostics d; ImuLink_Diagnostics(&d);
    assert(d.rx_bytes>0&&d.valid_frames>0&&d.register_replies>=2&&d.last_valid_baud==9600);
    reset_device(9600,2,12,6); connected=false; run(5000);
    assert(strcmp(ImuLink_Status(),"INIT_NO_REPLY")==0&&!ImuLink_Read(&sample)&&saves==0&&writes==0);
    ImuLink_Diagnostics(&d); assert(d.rx_bytes==0&&d.register_replies==0);
    reset_device(9600,2,12,6); bad_checksum=true; run(5000);
    assert(strcmp(ImuLink_Status(),"INIT_NO_REPLY")==0&&!ImuLink_Read(&sample)&&writes==0);
    ImuLink_Diagnostics(&d); assert(d.rx_bytes>0&&d.bad_checksums>0&&d.register_replies==0);
    reset_device(9600,2,12,6); reply_kind=0x71; run(5000);
    assert(strcmp(ImuLink_Status(),"INIT_NO_REPLY")==0&&!ImuLink_Read(&sample)&&writes==0);
    ImuLink_Diagnostics(&d); assert(d.register_replies==0); // No unsupported 71 register responses.
    reset_device(9600,2,2,6); ignore_writes=true; run(5000);
    assert(strcmp(ImuLink_Status(),"INIT_PARAM_MISMATCH")==0&&!ImuLink_Read(&sample)&&saves==0);
    reset_device(9600,2,2,7); ignore_rate=true; run(4000);
    assert(strcmp(ImuLink_Status(),"INIT_PARAM_MISMATCH")==0&&!ImuLink_Read(&sample)&&saves==0);
    ImuLink_Diagnostics(&d); assert(d.frequency==7&&d.baud_code==2);
    reset_device(9600,2,2,6); no_stream=true; run(5000);
    assert(strcmp(ImuLink_Status(),"INIT_STREAM_TIMEOUT")==0&&!ImuLink_Read(&sample));
    reset_device(9600,2,12,6); fail_tx=true; run(2000);
    assert(strcmp(ImuLink_Status(),"INIT_TX_ERROR")==0&&!ImuLink_Read(&sample));
    reset_device(9600,2,12,6); fail_baud=true; run(2000);
    assert(strcmp(ImuLink_Status(),"INIT_UART_ERROR")==0&&!ImuLink_Read(&sample));
    puts("PASS: fixed 9600/10Hz, no baud writes or scanning, bounded retry, register verification, config/save, fail-closed");
}
