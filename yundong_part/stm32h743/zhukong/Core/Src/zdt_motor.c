/**
 ******************************************************************************
 * @file    zdt_motor.c
 * @brief   张大头 X42S Emm 固件闭环步进电机 UART 驱动实现。
 ******************************************************************************
 */

#include "zdt_motor.h"

#define ZDT_ACK_LENGTH              4U
#define ZDT_MAX_ACK_SCAN_BYTES      16U
#define ZDT_MULTI_MOTOR_FUNCTION    0xAAU
#define ZDT_VELOCITY_FRAME_LENGTH   8U
#define ZDT_MULTI_FRAME_LENGTH       \
    (5U + (ZDT_MOTOR_WHEEL_COUNT * ZDT_VELOCITY_FRAME_LENGTH))

static UART_HandleTypeDef *s_motor_uart;
static bool s_wait_for_ack;
static uint32_t s_response_timeout_ms;

typedef struct
{
    uint8_t id;
    ZDT_Direction forward_direction;
} ZDT_WheelConfig;

/* 底盘机械安装标定：前进时左侧 CCW，右侧 CW。 */
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
                             HAL_MAX_DELAY);
}

HAL_StatusTypeDef ZDT_Motor_Enable(uint8_t id, bool enable)
{
    const uint8_t frame[] =
    {
        id, 0xF3U, 0xABU, enable ? 0x01U : 0x00U, 0x00U,
        ZDT_MOTOR_FRAME_TAIL
    };

    return ZDT_Motor_SendControl(frame, sizeof(frame), id, 0xF3U);
}

HAL_StatusTypeDef ZDT_Motor_SetVelocity(uint8_t id, ZDT_Direction direction,
                                        uint16_t speed_rpm,
                                        uint8_t acceleration, bool sync)
{
    const uint8_t frame[] =
    {
        id, 0xF6U, (uint8_t)direction,
        (uint8_t)(speed_rpm >> 8),
        (uint8_t)speed_rpm,
        acceleration,
        sync ? 0x01U : 0x00U,
        ZDT_MOTOR_FRAME_TAIL
    };

    return ZDT_Motor_SendControl(frame, sizeof(frame), id, 0xF6U);
}

HAL_StatusTypeDef ZDT_Motor_SetFourWheelVelocity(
    const ZDT_VelocityCommand commands[ZDT_MOTOR_WHEEL_COUNT])
{
    uint8_t frame[ZDT_MULTI_FRAME_LENGTH];
    uint8_t seen_ids = 0U;
    uint16_t index = 4U;
    uint16_t total_length = ZDT_MULTI_FRAME_LENGTH;
    uint32_t command_index;
    HAL_StatusTypeDef status;

    if (commands == NULL)
    {
        return HAL_ERROR;
    }

    for (command_index = 0U; command_index < ZDT_MOTOR_WHEEL_COUNT;
         ++command_index)
    {
        const ZDT_VelocityCommand *command = &commands[command_index];
        uint8_t id_bit;

        if ((command->id < ZDT_WHEEL_LEFT_UP) ||
            (command->id > ZDT_WHEEL_LEFT_DOWN))
        {
            return HAL_ERROR;
        }

        id_bit = (uint8_t)(1U << (command->id - ZDT_WHEEL_LEFT_UP));
        if ((seen_ids & id_bit) != 0U)
        {
            return HAL_ERROR;
        }
        seen_ids |= id_bit;

        /* 每条内嵌 F6 命令均为完整帧，并固定立即执行。 */
        frame[index++] = command->id;
        frame[index++] = 0xF6U;
        frame[index++] = (uint8_t)command->direction;
        frame[index++] = (uint8_t)(command->speed_rpm >> 8);
        frame[index++] = (uint8_t)command->speed_rpm;
        frame[index++] = command->acceleration;
        frame[index++] = 0x00U;
        frame[index++] = ZDT_MOTOR_FRAME_TAIL;
    }

    if (seen_ids != ((1U << ZDT_MOTOR_WHEEL_COUNT) - 1U))
    {
        return HAL_ERROR;
    }

    frame[0] = ZDT_MOTOR_BROADCAST_ID;
    frame[1] = ZDT_MULTI_MOTOR_FUNCTION;
    frame[2] = (uint8_t)(total_length >> 8);
    frame[3] = (uint8_t)total_length;
    frame[index] = ZDT_MOTOR_FRAME_TAIL;

    status = ZDT_Motor_SendRaw(frame, total_length);
    if ((status != HAL_OK) || !s_wait_for_ack)
    {
        return status;
    }

    /* 原厂协议规定：多电机运动命令只由地址 1 按内嵌 F6 命令确认收到。 */
    return ZDT_Motor_WaitAck(ZDT_WHEEL_LEFT_UP, 0xF6U);
}

HAL_StatusTypeDef ZDT_Motor_SetWheelSpeeds(
    const int16_t wheel_speed_rpm[ZDT_MOTOR_WHEEL_COUNT],
    uint8_t acceleration)
{
    ZDT_VelocityCommand commands[ZDT_MOTOR_WHEEL_COUNT];
    uint32_t wheel_index;

    if (wheel_speed_rpm == NULL)
    {
        return HAL_ERROR;
    }

    for (wheel_index = 0U; wheel_index < ZDT_MOTOR_WHEEL_COUNT;
         ++wheel_index)
    {
        int32_t speed = wheel_speed_rpm[wheel_index];

        if ((speed < -(int32_t)ZDT_MOTOR_MAX_SPEED_RPM) ||
            (speed > (int32_t)ZDT_MOTOR_MAX_SPEED_RPM))
        {
            return HAL_ERROR;
        }

        commands[wheel_index].id = s_wheel_config[wheel_index].id;
        commands[wheel_index].direction =
            (speed >= 0) ? s_wheel_config[wheel_index].forward_direction :
            ((s_wheel_config[wheel_index].forward_direction == ZDT_DIRECTION_CW) ?
             ZDT_DIRECTION_CCW : ZDT_DIRECTION_CW);
        commands[wheel_index].speed_rpm =
            (uint16_t)((speed >= 0) ? speed : -speed);
        commands[wheel_index].acceleration = acceleration;
    }

    return ZDT_Motor_SetFourWheelVelocity(commands);
}

HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id, bool sync)
{
    const uint8_t frame[] =
    {
        id, 0xFEU, 0x98U, sync ? 0x01U : 0x00U,
        ZDT_MOTOR_FRAME_TAIL
    };

    return ZDT_Motor_SendControl(frame, sizeof(frame), id, 0xFEU);
}

HAL_StatusTypeDef ZDT_Motor_SyncMotion(void)
{
    const uint8_t frame[] =
    {
        ZDT_MOTOR_BROADCAST_ID, 0xFFU, 0x66U, ZDT_MOTOR_FRAME_TAIL
    };

    /* 广播地址会有多个电机同时响应，因此不能等待 ACK。 */
    return ZDT_Motor_SendRaw(frame, sizeof(frame));
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
