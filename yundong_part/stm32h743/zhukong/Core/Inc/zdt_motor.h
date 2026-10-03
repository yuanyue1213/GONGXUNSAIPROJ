/**
 ******************************************************************************
 * @file    zdt_motor.h
 * @brief   张大头 X42S Emm 固件闭环步进电机 UART 驱动。
 ******************************************************************************
 * @note
 *   本驱动使用电机菜单 Checksum = 0x6B 的自定义协议，串口为 115200-8N1。
 *   使用前请确认 P_Serial = UART_FUN，且四个电机的 ID 分别为 1、2、3、4。
 */

#ifndef ZDT_MOTOR_H
#define ZDT_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define ZDT_MOTOR_BROADCAST_ID      0x00U
#define ZDT_MOTOR_FRAME_TAIL        0x6BU
#define ZDT_MOTOR_WHEEL_COUNT       4U
#define ZDT_MOTOR_MAX_SPEED_RPM      3000U

/* 机械安装方向由实际接线决定；CW/CCW 仅表示电机协议中的方向位。 */
typedef enum
{
    ZDT_DIRECTION_CW = 0x00U,
    ZDT_DIRECTION_CCW = 0x01U
} ZDT_Direction;

/* 四轮 ID 与机械位置的固定映射。 */
typedef enum
{
    ZDT_WHEEL_LEFT_UP = 0x01U,
    ZDT_WHEEL_RIGHT_UP = 0x02U,
    ZDT_WHEEL_RIGHT_DOWN = 0x03U,
    ZDT_WHEEL_LEFT_DOWN = 0x04U
} ZDT_WheelId;

/* 脉冲数组顺序：左前、右前、右后、左后。
 * 正号映射为左侧 CCW、右侧 CW；当前车体前进 F 使用负脉冲。 */
typedef enum
{
    ZDT_WHEEL_INDEX_LEFT_UP = 0U,
    ZDT_WHEEL_INDEX_RIGHT_UP,
    ZDT_WHEEL_INDEX_RIGHT_DOWN,
    ZDT_WHEEL_INDEX_LEFT_DOWN
} ZDT_WheelIndex;

/**
 * @brief 初始化驱动。
 * @param huart              已初始化的 UART 句柄（本工程传入 &huart1）。
 * @param wait_for_ack       true 时，每条控制命令等待 ID/FUN/02/6B 应答。
 * @param response_timeout_ms 等待单条应答的总超时，建议 20 ms。
 */
void ZDT_Motor_Init(UART_HandleTypeDef *huart, bool wait_for_ack,
                    uint32_t response_timeout_ms);

/** @brief 仅发送原始协议帧，不等待电机响应。 */
HAL_StatusTypeDef ZDT_Motor_SendRaw(const uint8_t *frame, uint16_t length);

/** @brief 使能（锁定）或去使能（释放）指定电机。 */
HAL_StatusTypeDef ZDT_Motor_Enable(uint8_t id, bool enable);

/** 四轮相对当前实际位置运动，脉冲正负号按安装映射决定方向。
 * 使用 00 AA + 四条 FD，模式 02，同步字段 00（AA 内不主动返回到位）。 */
HAL_StatusTypeDef ZDT_Motor_MoveWheelPulses(
    const int32_t wheel_pulses[ZDT_MOTOR_WHEEL_COUNT],
    uint16_t speed_rpm, uint8_t acceleration);
/** Combined translation/rotation: per-wheel speeds scale with displacement. */
HAL_StatusTypeDef ZDT_Motor_MoveWheelProfile(
    const int32_t wheel_pulses[ZDT_MOTOR_WHEEL_COUNT],
    const uint16_t speed_rpm[ZDT_MOTOR_WHEEL_COUNT], uint8_t acceleration);

/** 读取 3A 状态：bit0 使能，bit1 到位，bit2 堵转，bit3 堵转保护。 */
HAL_StatusTypeDef ZDT_Motor_ReadStatus(uint8_t id, uint8_t *flags);

/** @brief 以电机内部减速度停止。 */
HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id, bool sync);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_MOTOR_H */
