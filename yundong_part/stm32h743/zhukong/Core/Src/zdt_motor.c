/**
 ******************************************************************************
 * @file    zdt_motor.c
 * @brief   张大头 ZDT/Emm V2 闭环步进电机 UART 驱动实现。
 ******************************************************************************
 */

#include "zdt_motor.h"

#define ZDT_ACK_LENGTH              4U
#define ZDT_MAX_ACK_SCAN_BYTES      16U

static UART_HandleTypeDef *s_motor_uart;
static bool s_wait_for_ack;
static uint32_t s_response_timeout_ms;

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
                                        uint16_t acceleration_rpm_s_x10,
                                        uint16_t speed_rpm_x10, bool sync)
{
    const uint8_t frame[] =
    {
        id, 0xF6U, (uint8_t)direction,
        (uint8_t)(acceleration_rpm_s_x10 >> 8),
        (uint8_t)acceleration_rpm_s_x10,
        (uint8_t)(speed_rpm_x10 >> 8),
        (uint8_t)speed_rpm_x10,
        sync ? 0x01U : 0x00U,
        ZDT_MOTOR_FRAME_TAIL
    };

    return ZDT_Motor_SendControl(frame, sizeof(frame), id, 0xF6U);
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
