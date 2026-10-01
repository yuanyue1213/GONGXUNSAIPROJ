/**
 ******************************************************************************
 * @file    zdt_motor.c
 * @brief   张大头 X42S Emm 固件闭环步进电机 UART 驱动实现。
 ******************************************************************************
 */

#include "zdt_motor.h"

#define ZDT_ACK_LENGTH              4U
#define ZDT_TX_TIMEOUT_MS           20U
#define ZDT_MAX_ACK_SCAN_BYTES      16U
static UART_HandleTypeDef *s_motor_uart;
static bool s_wait_for_ack;
static uint32_t s_response_timeout_ms;

typedef struct
{
    uint8_t id;
    ZDT_Direction forward_direction;
} ZDT_WheelConfig;

/* 软件正号的安装映射：左侧 CCW、右侧 CW。
 * 车体前进 F 在上层实际使用负脉冲；此表不能直接当作 F 的方向表。 */
static const ZDT_WheelConfig s_wheel_config[ZDT_MOTOR_WHEEL_COUNT] =
{
    [ZDT_WHEEL_INDEX_LEFT_UP] = {ZDT_WHEEL_LEFT_UP, ZDT_DIRECTION_CCW},
    [ZDT_WHEEL_INDEX_RIGHT_UP] = {ZDT_WHEEL_RIGHT_UP, ZDT_DIRECTION_CW},
    [ZDT_WHEEL_INDEX_RIGHT_DOWN] = {ZDT_WHEEL_RIGHT_DOWN, ZDT_DIRECTION_CW},
    [ZDT_WHEEL_INDEX_LEFT_DOWN] = {ZDT_WHEEL_LEFT_DOWN, ZDT_DIRECTION_CCW}
};

static HAL_StatusTypeDef ZDT_Motor_WaitAck(uint8_t id, uint8_t function);
static HAL_StatusTypeDef ZDT_Motor_SendControl(const uint8_t *frame,
                                               uint16_t length,
                                               uint8_t id, uint8_t function);

void ZDT_Motor_Init(UART_HandleTypeDef *huart, bool wait_for_ack,
                    uint32_t response_timeout_ms)
{
    s_motor_uart = huart;
    s_wait_for_ack = wait_for_ack;
    s_response_timeout_ms = response_timeout_ms;
}

HAL_StatusTypeDef ZDT_Motor_SendRaw(const uint8_t *frame, uint16_t length)
{
    if ((s_motor_uart == NULL) || (frame == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

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
    uint8_t frame[57U]; /* 4 字节 AA 帧头 + 四个 13 字节 FD 子帧 + 1 字节帧尾。 */
    uint16_t index = 4U;
    uint32_t wheel;
    HAL_StatusTypeDef result;

    if ((wheel_pulses == NULL) || (speed_rpm == 0U) ||
        (speed_rpm > ZDT_MOTOR_MAX_SPEED_RPM)) return HAL_ERROR;
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
        frame[index++] = (uint8_t)(speed_rpm >> 8);
        frame[index++] = (uint8_t)speed_rpm;
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
    if ((result != HAL_OK) || !s_wait_for_ack) return result;
    /* AA 位置命令只等待 ID1 的 FD 接收确认，不代表四轮均到位。 */
    return ZDT_Motor_WaitAck(1U, 0xFDU);
}

/* 发送 ID 3A 6B，匹配 ID 3A flags 6B；滑动窗口跳过不匹配字节。
 * flags 位含义由 robot_control.c 判断；总接收时间受配置的超时限制。 */
HAL_StatusTypeDef ZDT_Motor_ReadStatus(uint8_t id, uint8_t *flags)
{
    const uint8_t request[] = {id, 0x3AU, ZDT_MOTOR_FRAME_TAIL};
    uint8_t response[4U];
    uint8_t length = 0U;
    uint32_t start;
    if ((flags == NULL) || (id < 1U) || (id > 4U)) return HAL_ERROR;
    if (ZDT_Motor_SendRaw(request, sizeof(request)) != HAL_OK) return HAL_ERROR;
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < s_response_timeout_ms)
    {
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed >= s_response_timeout_ms) break;
        if (HAL_UART_Receive(s_motor_uart, &response[length], 1U,
                            s_response_timeout_ms - elapsed) != HAL_OK) break;
        if (++length < 4U) continue;
        if ((response[0] == id) && (response[1] == 0x3AU) &&
            (response[3] == ZDT_MOTOR_FRAME_TAIL))
        {
            *flags = response[2];
            return HAL_OK;
        }
        response[0] = response[1];
        response[1] = response[2];
        response[2] = response[3];
        length = 3U;
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef ZDT_Motor_SendControl(const uint8_t *frame,
                                               uint16_t length,
                                               uint8_t id, uint8_t function)
{
    HAL_StatusTypeDef status = ZDT_Motor_SendRaw(frame, length);

    if ((status != HAL_OK) || !s_wait_for_ack || (id == ZDT_MOTOR_BROADCAST_ID))
    {
        return status;
    }

    return ZDT_Motor_WaitAck(id, function);
}

/* 匹配地址、功能码、成功码 02 和帧尾 6B。超时或扫描上限到达即失败。
 * ACK 仅证明电机接收控制命令；不自动重发相对位置帧，以免重复移动。 */
static HAL_StatusTypeDef ZDT_Motor_WaitAck(uint8_t id, uint8_t function)
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

        if (HAL_UART_Receive(s_motor_uart, &byte, 1U, remaining) != HAL_OK)
        {
            break;
        }

        response[received++] = byte;
        if (received < ZDT_ACK_LENGTH)
        {
            continue;
        }

        if ((response[0] == id) && (response[1] == function) &&
            (response[2] == 0x02U) &&
            (response[3] == ZDT_MOTOR_FRAME_TAIL))
        {
            return HAL_OK;
        }

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
