/**
 ******************************************************************************
 * @file    zdt_motor.c
 * @brief   张大头 X42S Emm 固件闭环步进电机 UART 驱动实现。
 ******************************************************************************
 */

#include "zdt_motor.h"
#include "zdt_motor_transport.h"
#include <string.h>

#define ZDT_ACK_LENGTH              4U
#define ZDT_TX_TIMEOUT_MS           20U
#define ZDT_MAX_ACK_SCAN_BYTES      16U
#define ZDT_POSITION_SCAN_BYTES     128U
static UART_HandleTypeDef *s_motor_uart;
static bool s_wait_for_ack;
static uint32_t s_response_timeout_ms;
volatile ZDT_MotorDiagnostics g_zdt_diagnostics[ZDT_MOTOR_WHEEL_COUNT];
/* The hardware build links zdt_motor_uart.c to override these test fallbacks. */
__attribute__((weak)) void ZDT_Transport_Start(UART_HandleTypeDef *uart) { (void)uart; }
__attribute__((weak)) void ZDT_Transport_PrepareTx(void) {}
__attribute__((weak)) HAL_StatusTypeDef ZDT_Transport_Receive(UART_HandleTypeDef *uart,uint8_t *byte,uint32_t timeout)
{ return HAL_UART_Receive(uart,byte,1U,timeout); }
__attribute__((weak)) uint32_t ZDT_Transport_Error(UART_HandleTypeDef *uart)
{ return uart?uart->ErrorCode:0U; }
static void diagnostic_begin(ZDT_RawTransaction *record,const uint8_t *tx,uint16_t length,uint32_t attempts)
{
    memset(record,0,sizeof(*record)); record->attempts=attempts+1U;
    record->started_tick=HAL_GetTick(); record->tx_length=length;
    record->rx_result=HAL_BUSY; // No receive call yet; distinct from a timeout.
    memcpy(record->tx,tx,length<sizeof(record->tx)?length:sizeof(record->tx));
}
static void diagnostic_byte(ZDT_RawTransaction *record,uint8_t byte)
{
    if(record&&record->rx_length<ZDT_DIAGNOSTIC_RX_BYTES) record->rx[record->rx_length++]=byte;
}
/* kind: 0=status, 1=stop, 2=actual position. */
static void diagnostic_publish(uint8_t id,unsigned kind,ZDT_RawTransaction *record,HAL_StatusTypeDef result)
{
    record->result=result; record->finished_tick=HAL_GetTick();
    record->uart_error=ZDT_Transport_Error(s_motor_uart);
    volatile ZDT_MotorDiagnostics *slot=&g_zdt_diagnostics[id-1U];
    ++slot->publish_sequence;
    if(kind==2U) slot->position=*record;
    else if(kind==1U) slot->stop=*record;
    else slot->status=*record;
    ++slot->publish_sequence;
}

typedef struct
{
    uint8_t id;
    ZDT_Direction forward_direction;
} ZDT_WheelConfig;

/* 软件正号的安装映射：左侧 CCW、右侧 CW。
 * 按实车反向反馈标定后，车体前进 F 在上层使用正脉冲。 */
static const ZDT_WheelConfig s_wheel_config[ZDT_MOTOR_WHEEL_COUNT] =
{
    [ZDT_WHEEL_INDEX_LEFT_UP] = {ZDT_WHEEL_LEFT_UP, ZDT_DIRECTION_CCW},
    [ZDT_WHEEL_INDEX_RIGHT_UP] = {ZDT_WHEEL_RIGHT_UP, ZDT_DIRECTION_CW},
    [ZDT_WHEEL_INDEX_RIGHT_DOWN] = {ZDT_WHEEL_RIGHT_DOWN, ZDT_DIRECTION_CW},
    [ZDT_WHEEL_INDEX_LEFT_DOWN] = {ZDT_WHEEL_LEFT_DOWN, ZDT_DIRECTION_CCW}
};

static HAL_StatusTypeDef ZDT_Motor_WaitAck(uint8_t id, uint8_t function,ZDT_RawTransaction *record);
__attribute__((weak)) void ZDT_Motor_Fault(uint8_t id, const char *operation)
{ (void)id; (void)operation; }
static HAL_StatusTypeDef read_wheel_position(uint8_t id, int64_t *counts)
{
    if(id<1U||id>4U||counts==NULL) return HAL_ERROR;
    const uint8_t request[]={id,0x36U,ZDT_MOTOR_FRAME_TAIL};
    ZDT_RawTransaction record;
    diagnostic_begin(&record,request,sizeof(request),g_zdt_diagnostics[id-1U].position.attempts);
    record.tx_result=ZDT_Motor_SendRaw(request,sizeof(request));
    if(record.tx_result!=HAL_OK) {
        diagnostic_publish(id,2U,&record,(HAL_StatusTypeDef)record.tx_result);
        return (HAL_StatusTypeDef)record.tx_result;
    }
    HAL_StatusTypeDef result=HAL_TIMEOUT;
    uint8_t response[8]; unsigned length=0,scanned=0; uint32_t start=HAL_GetTick();
    while((uint32_t)(HAL_GetTick()-start)<s_response_timeout_ms&&scanned++<ZDT_POSITION_SCAN_BYTES) {
        uint32_t elapsed=HAL_GetTick()-start;
        if(elapsed>=s_response_timeout_ms) break;
        record.rx_result=ZDT_Transport_Receive(s_motor_uart,&response[length],s_response_timeout_ms-elapsed);
        if(record.rx_result!=HAL_OK) { result=(HAL_StatusTypeDef)record.rx_result; break; }
        diagnostic_byte(&record,response[length]);
        if(++length<8U) continue;
        if(response[0]==id&&response[1]==0x36U&&response[2]<=1U&&response[7]==ZDT_MOTOR_FRAME_TAIL) {
            uint32_t raw=((uint32_t)response[3]<<24)|((uint32_t)response[4]<<16)|
                ((uint32_t)response[5]<<8)|response[6];
            int64_t signed_counts=response[2]?-(int64_t)raw:(int64_t)raw;
            /* Protocol positive is CW; normalized positive is CCW on left wheels. */
            *counts=s_wheel_config[id-1U].forward_direction==ZDT_DIRECTION_CW?signed_counts:-signed_counts;
            record.matched=1U;
            diagnostic_publish(id,2U,&record,HAL_OK);
            return HAL_OK;
        }
        for(unsigned i=0;i<7U;++i) response[i]=response[i+1U];
        length=7U;
    }
    diagnostic_publish(id,2U,&record,result);
    return result;
}

HAL_StatusTypeDef ZDT_Motor_ReadWheelPosition(uint8_t id, int64_t *counts)
{
    HAL_StatusTypeDef result=read_wheel_position(id,counts);
    /* Only this read-only query is retried, at most once on RX failure.
     * Each attempt keeps its configured deadline. No stale value is used and
     * no motion/relative-position command is replayed. TX failures return now. */
    if(result!=HAL_OK&&id>=1U&&id<=4U&&counts!=NULL&&
       g_zdt_diagnostics[id-1U].position.tx_result==HAL_OK)
        result=read_wheel_position(id,counts);
    if(result!=HAL_OK) ZDT_Motor_Fault(id,"POSITION_READ");
    return result;
}

static HAL_StatusTypeDef ZDT_Motor_SendControl(const uint8_t *frame,
                                               uint16_t length,
                                               uint8_t id, uint8_t function);

void ZDT_Motor_Init(UART_HandleTypeDef *huart, bool wait_for_ack,
                    uint32_t response_timeout_ms)
{
    s_motor_uart = huart;
    s_wait_for_ack = wait_for_ack;
    s_response_timeout_ms = response_timeout_ms;
    for(unsigned i=0;i<ZDT_MOTOR_WHEEL_COUNT;++i)
        g_zdt_diagnostics[i]=(ZDT_MotorDiagnostics){0};
    ZDT_Transport_Start(huart);
}

HAL_StatusTypeDef ZDT_Motor_SendRaw(const uint8_t *frame, uint16_t length)
{
    if ((s_motor_uart == NULL) || (frame == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    ZDT_Transport_PrepareTx();
    return HAL_UART_Transmit(s_motor_uart, (uint8_t *)frame, length,
                             ZDT_TX_TIMEOUT_MS);
}

/* F3 AB 使能命令，最后的 00 表示立即执行；所有帧均使用固定尾字节 6B。 */
HAL_StatusTypeDef ZDT_Motor_Enable(uint8_t id, bool enable)
{
    const uint8_t frame[] =
    {
        id, 0xF3U, 0xABU, enable ? 0x01U : 0x00U, 0x00U,
        ZDT_MOTOR_FRAME_TAIL
    };

    return ZDT_Motor_SendControl(frame, sizeof(frame), id, 0xF3U);
}

/* FE 98 停止帧。sync=false 时立即停止；true 时缓存等待同步触发。 */
HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id, bool sync)
{
    const uint8_t frame[] =
    {
        id, 0xFEU, 0x98U, sync ? 0x01U : 0x00U,
        ZDT_MOTOR_FRAME_TAIL
    };

    return ZDT_Motor_SendControl(frame, sizeof(frame), id, 0xFEU);
}

/* 定距入口：四个带符号的微步脉冲目标打包为 AA 多电机位置命令。
 * 脉冲绝对值决定转角，正负号决定方向；速度字段为 RPM，不是脉冲频率。
 * 此函数只确认接收命令，不等待到位；到位由上层轮询 3A 判断。 */
HAL_StatusTypeDef ZDT_Motor_MoveWheelPulses(
    const int32_t wheel_pulses[ZDT_MOTOR_WHEEL_COUNT],
    uint16_t speed_rpm, uint8_t acceleration)
{
    const uint16_t speeds[ZDT_MOTOR_WHEEL_COUNT] = {speed_rpm,speed_rpm,speed_rpm,speed_rpm};
    return ZDT_Motor_MoveWheelProfile(wheel_pulses, speeds, acceleration);
}
HAL_StatusTypeDef ZDT_Motor_MoveWheelSynchronous(const int32_t pulses[4],
    uint16_t rpm,uint8_t acceleration)
{
    if(!pulses||!rpm||rpm>ZDT_MOTOR_MAX_SPEED_RPM) return HAL_ERROR;
    for(unsigned i=0;i<4U;++i) {
        int64_t value=pulses[i];
        uint32_t magnitude=(uint32_t)(value<0?-value:value);
        ZDT_Direction forward=s_wheel_config[i].forward_direction;
        uint8_t dir=value>=0?(uint8_t)forward:(uint8_t)(forward==ZDT_DIRECTION_CW?ZDT_DIRECTION_CCW:ZDT_DIRECTION_CW);
        uint8_t frame[]={s_wheel_config[i].id,0xFD,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acceleration,
            (uint8_t)(magnitude>>24),(uint8_t)(magnitude>>16),(uint8_t)(magnitude>>8),(uint8_t)magnitude,
            0x02,0x01,ZDT_MOTOR_FRAME_TAIL}; /* relative to actual position, wait for sync */
        if(ZDT_Motor_SendControl(frame,sizeof(frame),frame[0],0xFD)!=HAL_OK) return HAL_ERROR;
    }
    const uint8_t trigger[]={0,0xFF,0x66,ZDT_MOTOR_FRAME_TAIL};
    HAL_StatusTypeDef result=ZDT_Motor_SendRaw(trigger,sizeof(trigger));
    if(result!=HAL_OK) ZDT_Motor_Fault(0U,"COMMAND");
    return result;
}
HAL_StatusTypeDef ZDT_Motor_MoveWheelProfile(
    const int32_t wheel_pulses[ZDT_MOTOR_WHEEL_COUNT],
    const uint16_t speed_rpm[ZDT_MOTOR_WHEEL_COUNT], uint8_t acceleration)
{
    uint8_t frame[57U]; /* 4 字节 AA 帧头 + 四个 13 字节 FD 子帧 + 1 字节帧尾。 */
    uint16_t index = 4U;
    uint32_t wheel;
    HAL_StatusTypeDef result;

    if (wheel_pulses == NULL || speed_rpm == NULL) return HAL_ERROR;
    for (wheel = 0U; wheel < ZDT_MOTOR_WHEEL_COUNT; ++wheel)
        if (speed_rpm[wheel] == 0U || speed_rpm[wheel] > ZDT_MOTOR_MAX_SPEED_RPM) return HAL_ERROR;
    frame[0] = 0x00U;
    frame[1] = 0xAAU;
    frame[2] = 0x00U;
    frame[3] = sizeof(frame);
    for (wheel = 0U; wheel < ZDT_MOTOR_WHEEL_COUNT; ++wheel)
    {
        int32_t value = wheel_pulses[wheel];
        /* 先扩展到 64 位再取负，避免 INT32_MIN 的绝对值溢出。 */
        uint32_t magnitude = (value < 0) ? (uint32_t)(-(int64_t)value) :
                                         (uint32_t)value;
        ZDT_Direction forward = s_wheel_config[wheel].forward_direction;
        frame[index++] = s_wheel_config[wheel].id;
        frame[index++] = 0xFDU;
        frame[index++] = (value >= 0) ? (uint8_t)forward :
                        (uint8_t)(forward == ZDT_DIRECTION_CW ?
                                  ZDT_DIRECTION_CCW : ZDT_DIRECTION_CW);
        frame[index++] = (uint8_t)(speed_rpm[wheel] >> 8);
        frame[index++] = (uint8_t)speed_rpm[wheel];
        frame[index++] = acceleration;
        /* 32 位脉冲数采用高字节在前；例如 9889 -> 00 00 26 A1。 */
        frame[index++] = (uint8_t)(magnitude >> 24);
        frame[index++] = (uint8_t)(magnitude >> 16);
        frame[index++] = (uint8_t)(magnitude >> 8);
        frame[index++] = (uint8_t)magnitude;
        frame[index++] = 0x02U; /* 相对当前实际位置，停止后新任务以当前位置为起点。 */
        frame[index++] = 0x00U; /* AA 内不主动回到位帧，避免共享总线回包冲突。 */
        frame[index++] = ZDT_MOTOR_FRAME_TAIL;
    }
    frame[index] = ZDT_MOTOR_FRAME_TAIL;
    result = ZDT_Motor_SendRaw(frame, sizeof(frame));
    /* AA 位置命令只等待 ID1 的 FD 接收确认，不代表四轮均到位。 */
    if(result==HAL_OK&&s_wait_for_ack) result=ZDT_Motor_WaitAck(1U,0xFDU,NULL);
    if(result!=HAL_OK) ZDT_Motor_Fault(0U,"POSITION_COMMAND");
    return result;
}

HAL_StatusTypeDef ZDT_Motor_SetWheelSpeeds(const int16_t rpm[4], uint8_t acceleration)
{
    if(rpm==NULL) return HAL_ERROR;
    uint8_t frame[37]={0,0xAA,0,37}; unsigned index=4;
    for(unsigned i=0;i<4U;++i) {
        int32_t value=rpm[i]; uint32_t magnitude=(uint32_t)(value<0?-value:value);
        if(magnitude>ZDT_MOTOR_MAX_SPEED_RPM) return HAL_ERROR;
        ZDT_Direction forward=s_wheel_config[i].forward_direction;
        frame[index++]=s_wheel_config[i].id; frame[index++]=0xF6;
        frame[index++]=value>=0?(uint8_t)forward:(uint8_t)(forward==ZDT_DIRECTION_CW?ZDT_DIRECTION_CCW:ZDT_DIRECTION_CW);
        frame[index++]=(uint8_t)(magnitude>>8); frame[index++]=(uint8_t)magnitude;
        frame[index++]=acceleration; frame[index++]=0; frame[index++]=ZDT_MOTOR_FRAME_TAIL;
    }
    frame[index]=ZDT_MOTOR_FRAME_TAIL;
    HAL_StatusTypeDef status=ZDT_Motor_SendRaw(frame,sizeof(frame));
    if(status==HAL_OK&&s_wait_for_ack) status=ZDT_Motor_WaitAck(1U,0xF6,NULL);
    if(status!=HAL_OK) ZDT_Motor_Fault(0U,"SPEED_COMMAND");
    return status;
}

HAL_StatusTypeDef ZDT_Motor_SetWheelSpeedsSynchronous(const int16_t rpm[4],uint8_t acceleration)
{
    if(!rpm) return HAL_ERROR;
    for(unsigned i=0;i<4U;++i) {
        int32_t value=rpm[i];
        if((uint32_t)(value<0?-value:value)>ZDT_MOTOR_MAX_SPEED_RPM) return HAL_ERROR;
    }
    for(unsigned i=0;i<4U;++i) {
        int32_t value=rpm[i]; uint16_t magnitude=(uint16_t)(value<0?-value:value);
        ZDT_Direction forward=s_wheel_config[i].forward_direction;
        uint8_t dir=value>=0?(uint8_t)forward:(uint8_t)(forward==ZDT_DIRECTION_CW?ZDT_DIRECTION_CCW:ZDT_DIRECTION_CW);
        uint8_t frame[]={s_wheel_config[i].id,0xF6,dir,(uint8_t)(magnitude>>8),(uint8_t)magnitude,
            acceleration,0x01,ZDT_MOTOR_FRAME_TAIL};
        if(ZDT_Motor_SendControl(frame,sizeof(frame),frame[0],0xF6)!=HAL_OK) return HAL_ERROR;
    }
    const uint8_t trigger[]={0,0xFF,0x66,ZDT_MOTOR_FRAME_TAIL};
    HAL_StatusTypeDef result=ZDT_Motor_SendRaw(trigger,sizeof(trigger));
    if(result!=HAL_OK) ZDT_Motor_Fault(0U,"SPEED_COMMAND");
    return result;
}

/* 发送 ID 3A 6B，匹配 ID 3A flags 6B；滑动窗口跳过不匹配字节。
 * flags 位含义由 robot_control.c 判断；总接收时间受配置的超时限制。 */
static HAL_StatusTypeDef read_status(uint8_t id, uint8_t *flags)
{
    const uint8_t request[] = {id, 0x3AU, ZDT_MOTOR_FRAME_TAIL};
    uint8_t response[4U];
    uint8_t length = 0U;
    uint32_t start;
    if ((flags == NULL) || (id < 1U) || (id > 4U)) return HAL_ERROR;
    ZDT_RawTransaction record;
    diagnostic_begin(&record,request,sizeof(request),g_zdt_diagnostics[id-1U].status.attempts);
    record.tx_result=ZDT_Motor_SendRaw(request,sizeof(request));
    if(record.tx_result!=HAL_OK) { diagnostic_publish(id,false,&record,HAL_ERROR); return HAL_ERROR; }
    HAL_StatusTypeDef result=HAL_TIMEOUT;
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < s_response_timeout_ms)
    {
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed >= s_response_timeout_ms) break;
        record.rx_result=ZDT_Transport_Receive(s_motor_uart,&response[length],s_response_timeout_ms-elapsed);
        if(record.rx_result!=HAL_OK) { result=(HAL_StatusTypeDef)record.rx_result; break; }
        diagnostic_byte(&record,response[length]);
        if (++length < 4U) continue;
        if ((response[0] == id) && (response[1] == 0x3AU) &&
            (response[3] == ZDT_MOTOR_FRAME_TAIL))
        {
            *flags = response[2];
            record.matched=1; record.status_flags=*flags;
            diagnostic_publish(id,false,&record,HAL_OK);
            return HAL_OK;
        }
        response[0] = response[1];
        response[1] = response[2];
        response[2] = response[3];
        length = 3U;
    }
    diagnostic_publish(id,false,&record,result);
    return result;
}

HAL_StatusTypeDef ZDT_Motor_ReadStatus(uint8_t id, uint8_t *flags)
{
    HAL_StatusTypeDef result=read_status(id,flags);
    /* Retry only a failed read transaction, not a valid protection/disabled
     * status. A second failure still reaches the original stop path. */
    if(result!=HAL_OK&&id>=1U&&id<=4U&&flags!=NULL&&
       g_zdt_diagnostics[id-1U].status.tx_result==HAL_OK)
        result=read_status(id,flags);
    if(result!=HAL_OK) ZDT_Motor_Fault(id,"STATUS_READ");
    return result;
}

static HAL_StatusTypeDef ZDT_Motor_SendControl(const uint8_t *frame,
                                               uint16_t length,
                                               uint8_t id, uint8_t function)
{
    ZDT_RawTransaction record;
    bool capture=function==0xFEU&&id>=1U&&id<=4U;
    if(capture) diagnostic_begin(&record,frame,length,g_zdt_diagnostics[id-1U].stop.attempts);
    HAL_StatusTypeDef status = ZDT_Motor_SendRaw(frame, length);
    if(capture) record.tx_result=status;

    if ((status != HAL_OK) || !s_wait_for_ack || (id == ZDT_MOTOR_BROADCAST_ID))
    {
        if(capture) diagnostic_publish(id,true,&record,status);
        if(status!=HAL_OK) ZDT_Motor_Fault(id,function==0xFEU?"STOP":function==0xF3U?"ENABLE":"COMMAND");
        return status;
    }

    status=ZDT_Motor_WaitAck(id,function,capture?&record:NULL);
    if(capture) diagnostic_publish(id,true,&record,status);
    if(status!=HAL_OK) ZDT_Motor_Fault(id,function==0xFEU?"STOP":function==0xF3U?"ENABLE":"COMMAND");
    return status;
}

/* Match ACKs and reject explicit E2/EE replies instead of disguising them as
 * timeouts. Other addresses/frames still pass through the sliding window.
 * ACK 仅证明电机接收控制命令；不自动重发相对位置帧，以免重复移动。 */
static HAL_StatusTypeDef ZDT_Motor_WaitAck(uint8_t id, uint8_t function,ZDT_RawTransaction *record)
{
    uint8_t response[ZDT_ACK_LENGTH];
    uint8_t byte;
    uint8_t received = 0U;
    uint8_t scanned = 0U;
    uint32_t start_tick;

    if ((s_motor_uart == NULL) || (s_response_timeout_ms == 0U))
    {
        return HAL_TIMEOUT;
    }

    start_tick = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_tick) < s_response_timeout_ms)
    {
        uint32_t elapsed = HAL_GetTick() - start_tick;
        if (elapsed >= s_response_timeout_ms) break;
        uint32_t remaining = s_response_timeout_ms - elapsed;

        HAL_StatusTypeDef rx=ZDT_Transport_Receive(s_motor_uart,&byte,remaining);
        if(record) record->rx_result=rx;
        if (rx != HAL_OK)
        {
            break;
        }

        diagnostic_byte(record,byte);
        response[received++] = byte;
        if (received < ZDT_ACK_LENGTH)
        {
            continue;
        }

        if ((response[0] == id) && (response[1] == function) &&
            (response[2] == 0x02U) &&
            (response[3] == ZDT_MOTOR_FRAME_TAIL))
        {
            if(record) record->matched=1;
            return HAL_OK;
        }
        if(response[0]==id&&response[3]==ZDT_MOTOR_FRAME_TAIL&&
           ((response[1]==function&&(response[2]==0xE2U||response[2]==0xEEU))||
            (response[1]==0U&&response[2]==0xEEU))) return HAL_ERROR;

        response[0] = response[1];
        response[1] = response[2];
        response[2] = response[3];
        received = ZDT_ACK_LENGTH - 1U;

        if (++scanned >= ZDT_MAX_ACK_SCAN_BYTES)
        {
            break;
        }
    }

    return HAL_TIMEOUT;
}
