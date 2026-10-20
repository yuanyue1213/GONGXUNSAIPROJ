#include "imu_link.h"
#include <string.h>
#include <math.h>
/* User mounting: X forward, Y right, Z down; controller uses clockwise positive.
 * Euler yaw and body GZ can have different signs when mounted upside down.
 * Verify each independently with a hand rotation before enabling motor turns. */
#define IMU_YAW_SIGN 1.0f
#define IMU_RATE_SIGN 1.0f
#define IMU_QUEUE 512U
static UART_HandleTypeDef *s_uart;
static uint8_t s_byte,s_data[IMU_QUEUE],s_frame[11];
static uint32_t s_ticks[IMU_QUEUE];
static volatile uint16_t s_head,s_tail;
static volatile bool s_fault,s_restart;
static unsigned s_length,s_angles,s_rates;
static float s_raw_yaw;
static ImuSample s_sample;
static volatile ImuDiagnostics s_diag;
static const char *s_setup_error;
typedef enum { SETUP_OFF, SETUP_WAIT, SETUP_SCAN, SETUP_UNLOCK, SETUP_OUTPUT,
    SETUP_RATE_UNLOCK, SETUP_RATE, SETUP_VERIFY, SETUP_SAVE_UNLOCK,
    SETUP_SAVE, SETUP_SETTLE, SETUP_STREAM, SETUP_READY, SETUP_FAILED } SetupState;
static struct {
    SetupState state;
    uint32_t since,query_tick,query_sequence;
    unsigned replies,scan_pass;
    bool pending,changed;
    uint16_t registers[4];
    uint32_t sequence;
} s_setup;
static void setup_tick(void);
static void arm(void)
{
    if(!s_uart) return;
    HAL_StatusTypeDef result=HAL_UART_Receive_IT(s_uart,&s_byte,1U);
    s_restart=result!=HAL_OK&&result!=HAL_BUSY;
}
void ImuLink_Init(UART_HandleTypeDef *uart)
{
    s_uart=uart; s_head=s_tail=0; s_fault=s_restart=false;
    s_length=s_angles=s_rates=0; memset(&s_sample,0,sizeof(s_sample)); s_raw_yaw=0;
    memset(&s_setup,0,sizeof(s_setup));
    s_diag=(ImuDiagnostics){.baud=IMU_UART_BAUD_RATE}; s_setup_error="INIT_FAILED";
    arm();
}
void ImuLink_StartConfiguration(void)
{
    if(!s_uart||s_setup.state!=SETUP_OFF) return;
    s_setup.state=SETUP_WAIT; s_setup.since=HAL_GetTick();
}
bool ImuLink_RxCallback(UART_HandleTypeDef *uart)
{
    if(!s_uart||uart!=s_uart) return false;
    ++s_diag.rx_bytes;
    uint16_t next=(uint16_t)((s_head+1U)%IMU_QUEUE);
    if(next==s_tail) s_fault=true;
    else { s_data[s_head]=s_byte; s_ticks[s_head]=HAL_GetTick(); s_head=next; }
    arm(); return true;
}
bool ImuLink_ErrorCallback(UART_HandleTypeDef *uart)
{
    if(!s_uart||uart!=s_uart) return false;
    ++s_diag.uart_errors;
    s_fault=true;
    __HAL_UART_CLEAR_OREFLAG(uart); __HAL_UART_CLEAR_NEFLAG(uart); __HAL_UART_CLEAR_FEFLAG(uart);
    arm(); return true;
}
static int16_t signed16(const uint8_t *p) { return (int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
void ImuLink_Process(void)
{
    if(s_fault) {
        uint32_t irq=__get_PRIMASK(); __disable_irq(); s_tail=s_head; s_fault=false; __set_PRIMASK(irq);
        s_length=s_angles=s_rates=0;
        s_setup.pending=false;
        ++s_sample.epoch;
    }
    if(s_restart) arm();
    while(s_tail!=s_head) {
        uint8_t b=s_data[s_tail]; uint32_t tick=s_ticks[s_tail]; s_tail=(uint16_t)((s_tail+1U)%IMU_QUEUE);
        if(s_length==0U&&b!=0x55U) continue;
        s_frame[s_length++]=b;
        if(s_length<11U) continue;
        uint8_t sum=0; for(unsigned i=0;i<10U;++i) sum=(uint8_t)(sum+s_frame[i]);
        if(sum!=s_frame[10]) {
            ++s_diag.bad_checksums;
            unsigned start=1; while(start<11U&&s_frame[start]!=0x55U) ++start;
            s_length=11U-start; memmove(s_frame,s_frame+start,s_length); continue;
        }
        /* Official standard UART register reply is 55 5F + four little-endian
         * registers + sum. No unverified alternate reply types are accepted. */
        ++s_diag.valid_frames; s_diag.last_valid_baud=s_diag.baud;
        if(s_frame[1]==0x5FU&&s_setup.pending&&
            (uint32_t)(tick-s_setup.query_tick)<=250U) {
            for(unsigned i=0;i<4;++i) s_setup.registers[i]=(uint16_t)s_frame[2+i*2]|((uint16_t)s_frame[3+i*2]<<8);
            ++s_setup.sequence; s_setup.pending=false;
            ++s_diag.register_replies;
            s_diag.output=s_setup.registers[0]; s_diag.frequency=s_setup.registers[1]; s_diag.baud_code=s_setup.registers[2];
        } else if(s_frame[1]==0x52U) {
            ++s_diag.rate_frames;
            s_sample.rate=IMU_RATE_SIGN*signed16(&s_frame[6])*(2000.0f/32768.0f);
            s_sample.rate_tick=tick; if(s_rates<10U) ++s_rates;
        } else if(s_frame[1]==0x53U) {
            ++s_diag.angle_frames;
            float yaw=IMU_YAW_SIGN*signed16(&s_frame[6])*(180.0f/32768.0f);
            if(s_angles&&(uint32_t)(tick-s_sample.angle_tick)>200U) {
                s_angles=s_rates=0; ++s_sample.epoch;
            }
            if(s_angles==0U) s_sample.yaw=yaw;
            else {
                float delta=yaw-s_raw_yaw;
                if(delta>180) delta-=360;
                if(delta< -180) delta+=360;
                s_sample.yaw+=delta;
            }
            s_raw_yaw=yaw; s_sample.angle_tick=tick; ++s_sample.sequence;
            if(s_angles<10U) ++s_angles;
        }
        s_length=0;
    }
    setup_tick();
}
bool ImuLink_Read(ImuSample *sample)
{
    uint32_t now=HAL_GetTick();
    if(s_setup.state!=SETUP_OFF&&s_setup.state!=SETUP_READY) return false;
    if(s_angles<10U||s_rates<10U||(uint32_t)(now-s_sample.angle_tick)>200U||
        (uint32_t)(now-s_sample.rate_tick)>200U||s_fault) return false;
    *sample=s_sample; return true;
}
static bool fresh_stream(void)
{
    uint32_t now=HAL_GetTick();
    return s_angles>=10U&&s_rates>=10U&&!s_fault&&
        (uint32_t)(now-s_sample.angle_tick)<=200U&&(uint32_t)(now-s_sample.rate_tick)<=200U;
}
const char *ImuLink_Status(void)
{
    if(s_setup.state==SETUP_FAILED) return s_setup_error;
    if(s_setup.state==SETUP_WAIT||s_setup.state==SETUP_SCAN) return "INIT_SEARCH";
    if(s_setup.state>=SETUP_UNLOCK&&s_setup.state<=SETUP_RATE) return "INIT_CONFIG";
    if(s_setup.state>=SETUP_VERIFY&&s_setup.state<=SETUP_STREAM) return "INIT_VERIFY";
    return fresh_stream()?"READY":"NO_DATA";
}
void ImuLink_Diagnostics(ImuDiagnostics *diagnostics)
{
    uint32_t irq=__get_PRIMASK(); __disable_irq(); *diagnostics=s_diag; __set_PRIMASK(irq);
}
static void setup_state(SetupState state)
{
    s_setup.state=state; s_setup.since=HAL_GetTick();
}
static void setup_fail(const char *reason) { s_setup_error=reason; setup_state(SETUP_FAILED); }
static bool setup_write(uint8_t reg,uint16_t value)
{
    uint8_t frame[]={0xFF,0xAA,reg,(uint8_t)value,(uint8_t)(value>>8)};
    if(HAL_UART_Transmit(s_uart,frame,sizeof(frame),20U)==HAL_OK) return true;
    setup_fail("INIT_TX_ERROR"); return false;
}
static bool setup_baud(uint32_t baud)
{
    if(ImuLink_SetBaud(s_uart,baud)!=HAL_OK) { setup_fail("INIT_UART_ERROR"); return false; }
    s_diag.baud=baud;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    s_tail=s_head=0; s_fault=s_restart=false; s_length=s_angles=s_rates=0;
    s_setup.pending=false; ++s_sample.epoch; __set_PRIMASK(irq);
    arm(); return true;
}
static bool setup_query(void)
{
    s_setup.query_sequence=s_setup.sequence;
    s_setup.query_tick=HAL_GetTick(); s_setup.pending=true;
    return setup_write(0x27,0x02); // RSW, RRATE, BAUD, AXOFFSET (read only)
}
static bool setup_response(void) { return s_setup.sequence!=s_setup.query_sequence; }
static bool setup_parameters(void)
{
    return s_setup.registers[0]==0x0CU&&s_setup.registers[1]==0x06U&&s_setup.registers[2]==0x02U;
}
static void setup_scan(void)
{
    if(!setup_baud(IMU_UART_BAUD_RATE)) return;
    s_setup.replies=0; setup_state(SETUP_SCAN); (void)setup_query();
}
static void setup_tick(void)
{
    uint32_t now=HAL_GetTick(),elapsed=(uint32_t)(now-s_setup.since);
    switch(s_setup.state) {
    case SETUP_OFF: case SETUP_READY: case SETUP_FAILED: return;
    case SETUP_WAIT:
        /* The manufacturer's WT901 startup specification is 1000 ms. Allow
         * margin before probing, and make one bounded second scan if late. */
        if(elapsed>=1500U) setup_scan();
        return;
    case SETUP_SCAN:
        if(setup_response()) {
            s_setup.query_sequence=s_setup.sequence;
            if(s_setup.registers[2]==0x02U) ++s_setup.replies;
            else s_setup.replies=0;
            if(s_setup.replies>=2U) {
                s_setup.changed=!setup_parameters();
                if(!s_setup.changed) { setup_state(SETUP_STREAM); return; }
                if(setup_write(0x69,0xB588)) setup_state(SETUP_UNLOCK);
                return;
            }
        }
        if(elapsed>=1500U) {
            if(++s_setup.scan_pass<2U) setup_scan();
            else setup_fail("INIT_NO_REPLY");
        } else if((uint32_t)(now-s_setup.query_tick)>=150U) (void)setup_query();
        return;
    case SETUP_UNLOCK:
        if(elapsed>=50U&&setup_write(0x02,0x0C)) setup_state(SETUP_OUTPUT);
        return;
    case SETUP_OUTPUT:
        if(elapsed>=50U&&setup_write(0x69,0xB588)) setup_state(SETUP_RATE_UNLOCK);
        return;
    case SETUP_RATE_UNLOCK:
        if(elapsed>=50U&&setup_write(0x03,0x06)) setup_state(SETUP_RATE);
        return;
    case SETUP_RATE:
        if(elapsed>=100U) {
            s_setup.replies=0; setup_state(SETUP_VERIFY); (void)setup_query();
        }
        return;
    case SETUP_VERIFY:
        if(setup_response()) {
            s_setup.query_sequence=s_setup.sequence;
            if(!setup_parameters()) { setup_fail("INIT_PARAM_MISMATCH"); return; }
            if(++s_setup.replies>=2U) {
                if(setup_write(0x69,0xB588)) setup_state(SETUP_SAVE_UNLOCK);
                return;
            }
        }
        if(elapsed>=1500U) setup_fail("INIT_VERIFY_TIMEOUT");
        else if((uint32_t)(now-s_setup.query_tick)>=150U) (void)setup_query();
        return;
    case SETUP_SAVE_UNLOCK:
        if(elapsed>=50U&&setup_write(0x00,0x00)) setup_state(SETUP_SAVE);
        return;
    case SETUP_SAVE:
        if(elapsed>=200U) { setup_state(SETUP_SETTLE); (void)setup_query(); }
        return;
    case SETUP_SETTLE:
        if(setup_response()) {
            if(setup_parameters()) setup_state(SETUP_STREAM);
            else setup_fail("INIT_SAVE_MISMATCH");
        } else if(elapsed>=1000U) setup_fail("INIT_SAVE_TIMEOUT");
        return;
    case SETUP_STREAM:
        if(fresh_stream()) setup_state(SETUP_READY);
        else if(elapsed>=1500U) setup_fail("INIT_STREAM_TIMEOUT");
        return;
    }
}
