/**
 ******************************************************************************
 * @file    zdt_motor.h
 * @brief   张大头 ZDT/Emm V2 闭环步进电机 UART 驱动。
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

/**
 * @brief 设置速度模式。
 * @param acceleration_rpm_s_x10 加速度，单位 0.1 RPM/s。
 * @param speed_rpm_x10          速度，单位 0.1 RPM，范围由电机参数决定。
 * @param sync                   true: 先缓存命令，等待 ZDT_Motor_SyncMotion() 广播后执行。
 */
HAL_StatusTypeDef ZDT_Motor_SetVelocity(uint8_t id, ZDT_Direction direction,
                                        uint16_t acceleration_rpm_s_x10,
                                        uint16_t speed_rpm_x10, bool sync);

/** @brief 以电机内部减速度停止。 */
HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id, bool sync);

/** @brief 发送广播同步运动命令，使各电机执行已缓存的同步命令。 */
HAL_StatusTypeDef ZDT_Motor_SyncMotion(void);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_MOTOR_H */
