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

/** Emm 固件速度模式（F6）的一条速度命令。 */
typedef struct
{
    uint8_t id;
    ZDT_Direction direction;
    uint16_t speed_rpm;  /* 单位：RPM，范围 0-3000。 */
    uint8_t acceleration; /* 加速度档位，范围 0-255；0 为直接启动。 */
} ZDT_VelocityCommand;

/*
 * 轮速数组的固定顺序：左上、右上、右下、左下。
 * 正 RPM 表示该轮推动底盘前进；驱动内部已封装左逆右顺的安装方向。
 */
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

/**
 * @brief 设置速度模式。
 * @param speed_rpm              速度，单位 RPM，范围由电机参数决定。
 * @param acceleration            加速度档位：0-255；0 为直接启动。
 * @param sync                   true: 先缓存命令，等待 ZDT_Motor_SyncMotion() 广播后执行。
 */
HAL_StatusTypeDef ZDT_Motor_SetVelocity(uint8_t id, ZDT_Direction direction,
                                        uint16_t speed_rpm,
                                        uint8_t acceleration, bool sync);

/**
 * @brief 使用 X42S 多电机命令帧（00 AA）让四个轮子同时进入 Emm 速度模式。
 * @param commands 按任意顺序给出四条命令，但 ID 必须恰好是 1、2、3、4 各一次。
 *
 * 帧格式：00 AA 00 25 [ID1 F6 ... 6B] [ID2 F6 ... 6B]
 *         [ID3 F6 ... 6B] [ID4 F6 ... 6B] 6B。
 * 多电机帧本身就是同时执行，因此内部同步标志固定为 00；仅 ID 1 按其内嵌
 * F6 命令回复 01 F6 02 6B 确认接收。
 */
HAL_StatusTypeDef ZDT_Motor_SetFourWheelVelocity(
    const ZDT_VelocityCommand commands[ZDT_MOTOR_WHEEL_COUNT]);

/**
 * @brief 以底盘运动语义设置四轮速度。
 * @param wheel_speed_rpm 有符号 RPM，顺序为左上、右上、右下、左下。
 *                        正数=该轮前进，负数=该轮后退，范围 -3000 至 3000。
 * @param acceleration    Emm 加速度档位，范围 0-255。
 *
 * 电机 ID 和轮子正向已封装为：左上(1, CCW)、右上(2, CW)、
 * 右下(3, CW)、左下(4, CCW)。
 */
HAL_StatusTypeDef ZDT_Motor_SetWheelSpeeds(
    const int16_t wheel_speed_rpm[ZDT_MOTOR_WHEEL_COUNT],
    uint8_t acceleration);

/** @brief 以电机内部减速度停止。 */
HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id, bool sync);

/** @brief 发送广播同步运动命令，使各电机执行已缓存的同步命令。 */
HAL_StatusTypeDef ZDT_Motor_SyncMotion(void);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_MOTOR_H */
