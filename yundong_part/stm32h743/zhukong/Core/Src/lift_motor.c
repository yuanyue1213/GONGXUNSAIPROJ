#include "lift_motor.h"
#include "zdt_motor.h"

#define LIFT_MOTOR_ID          1U
#define FORE_AFT_MOTOR_ID      2U
#define LIFT_SPEED_RPM         30U
#define LIFT_ACCELERATION      10U
#define LIFT_ACK_TIMEOUT_MS    20U
/* Installed lift: U rises with CCW, D lowers with CW; shared by both motion modes. */
#define LIFT_UP_DIRECTION      ZDT_DIRECTION_CCW
#define LIFT_DOWN_DIRECTION    ZDT_DIRECTION_CW
#define ARM_FORWARD_DIRECTION  ZDT_DIRECTION_CW
#define ARM_BACK_DIRECTION     ZDT_DIRECTION_CCW
#define ARM_FIRMWARE_UNKNOWN   0U
#define ARM_FIRMWARE_EMM       1U
#define ARM_FIRMWARE_X         2U

static UART_HandleTypeDef *s_uart;
static bool s_lift_enabled;
static bool s_fore_aft_enabled;
static uint8_t s_firmware[2];
static const char *s_last_error[2];

/* Configuration reply length is 0x21 for Emm and 0x25 for X firmware. */
static uint8_t ArmMotor_DetectFirmware(uint8_t id)
{
    uint8_t request[] = {id, 0x42U, 0x6CU, ZDT_MOTOR_FRAME_TAIL};
    uint8_t response[37];
    uint8_t previous = 0U;
    uint8_t byte;
    uint8_t length;
    uint32_t start;

    if ((id < LIFT_MOTOR_ID) || (id > FORE_AFT_MOTOR_ID))
        return ARM_FIRMWARE_UNKNOWN;
    if (s_uart == NULL)
    {
        s_last_error[id - 1U] = "UART";
        return ARM_FIRMWARE_UNKNOWN;
    }
    if (s_firmware[id - 1U] != ARM_FIRMWARE_UNKNOWN)
        return s_firmware[id - 1U];
    s_last_error[id - 1U] = "NO_REPLY";
    if (HAL_UART_Transmit(s_uart, request, sizeof(request), 20U) != HAL_OK)
    {
        s_last_error[id - 1U] = "TX";
        return ARM_FIRMWARE_UNKNOWN;
    }

    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < 40U)
    {
        uint32_t remaining = 40U - (uint32_t)(HAL_GetTick() - start);
        if (HAL_UART_Receive(s_uart, &byte, 1U, remaining) != HAL_OK)
            return ARM_FIRMWARE_UNKNOWN;
        if ((previous == id) && (byte == 0x42U)) break;
        previous = byte;
    }
    if ((previous != id) || (byte != 0x42U))
        return ARM_FIRMWARE_UNKNOWN;
    if (HAL_UART_Receive(s_uart, &length, 1U, 10U) != HAL_OK)
        return ARM_FIRMWARE_UNKNOWN;
    if ((length != 0x21U) && (length != 0x25U))
    {
        s_last_error[id - 1U] = "BAD_REPLY";
        return ARM_FIRMWARE_UNKNOWN;
    }
    response[0] = id;
    response[1] = 0x42U;
    response[2] = length;
    if (HAL_UART_Receive(s_uart, &response[3], length - 3U, 20U) != HAL_OK)
        return ARM_FIRMWARE_UNKNOWN;
    if (response[length - 1U] != ZDT_MOTOR_FRAME_TAIL)
    {
        s_last_error[id - 1U] = "BAD_REPLY";
        return ARM_FIRMWARE_UNKNOWN;
    }
    s_firmware[id - 1U] =
        (length == 0x21U) ? ARM_FIRMWARE_EMM : ARM_FIRMWARE_X;
    s_last_error[id - 1U] = "NONE";
    return s_firmware[id - 1U];
}

static HAL_StatusTypeDef LiftMotor_Send(const uint8_t *frame, uint16_t length,
                                        uint8_t id, uint8_t function)
{
    uint8_t response[4];
    uint8_t byte;
    uint8_t count = 0U;
    uint8_t scanned = 0U;
    uint32_t start;

    if (s_uart == NULL)
    {
        s_last_error[id - 1U] = "UART";
        return HAL_ERROR;
    }
    if (HAL_UART_Transmit(s_uart, (uint8_t *)frame, length,
                          LIFT_ACK_TIMEOUT_MS) != HAL_OK)
    {
        s_last_error[id - 1U] = "TX";
        return HAL_ERROR;
    }

    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < LIFT_ACK_TIMEOUT_MS)
    {
        uint32_t remaining = LIFT_ACK_TIMEOUT_MS -
                             (uint32_t)(HAL_GetTick() - start);
        if (HAL_UART_Receive(s_uart, &byte, 1U, remaining) != HAL_OK)
        {
            break;
        }
        response[count++] = byte;
        if (count < sizeof(response))
        {
            continue;
        }
        if ((response[0] == id) &&
            (response[1] == function) &&
            (response[3] == ZDT_MOTOR_FRAME_TAIL))
        {
            if (response[2] == 0x02U)
            {
                s_last_error[id - 1U] = "NONE";
                return HAL_OK;
            }
            s_last_error[id - 1U] =
                (response[2] == 0xE2U) ? "REJECT_E2" :
                (response[2] == 0xEEU) ? "FORMAT_EE" : "BAD_REPLY";
            return HAL_ERROR;
        }
        response[0] = response[1];
        response[1] = response[2];
        response[2] = response[3];
        count = 3U;
        if (++scanned >= 16U)
        {
            break;
        }
    }
    s_last_error[id - 1U] = "NO_REPLY";
    return HAL_TIMEOUT;
}

/* USART2 独立连接升降 ID1、伸缩 ID2，与底盘 USART1 的相同 ID 不冲突。
 * 初始化仅清空使能、固件识别及错误状态，不让机械臂运动。 */
void LiftMotor_Init(UART_HandleTypeDef *uart)
{
    s_uart = uart;
    s_lift_enabled = false;
    s_fore_aft_enabled = false;
    s_firmware[0] = ARM_FIRMWARE_UNKNOWN;
    s_firmware[1] = ARM_FIRMWARE_UNKNOWN;
    s_last_error[0] = (uart == NULL) ? "UART" : "NONE";
    s_last_error[1] = (uart == NULL) ? "UART" : "NONE";
}

/* 首次运动先识别 Emm/X 固件并使能，再按对应格式发送速度命令。
 * 持续运动的松手停止及 300 ms 超时由 robot_control.c 管理。 */
static HAL_StatusTypeDef ArmMotor_Move(uint8_t id, bool *enabled,
                                       ZDT_Direction direction,
                                       uint16_t speed_rpm)
{
    const uint8_t enable_frame[] =
        {id, 0xF3U, 0xABU, 0x01U, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    uint8_t velocity_frame[] =
        {id, 0xF6U, (uint8_t)direction,
         (uint8_t)(speed_rpm >> 8), (uint8_t)speed_rpm,
         LIFT_ACCELERATION, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    uint8_t x_velocity_frame[] =
        {id, 0xF6U, (uint8_t)direction,
         0x00U, 0xC8U, /* X firmware: 200 RPM/s acceleration. */
         (uint8_t)((speed_rpm * 10U) >> 8), (uint8_t)(speed_rpm * 10U),
         0x00U, ZDT_MOTOR_FRAME_TAIL};
    uint8_t firmware = ArmMotor_DetectFirmware(id);

    if (firmware == ARM_FIRMWARE_UNKNOWN) return HAL_ERROR;

    if (!*enabled)
    {
        if (LiftMotor_Send(enable_frame, sizeof(enable_frame), id, 0xF3U) != HAL_OK)
        {
            return HAL_ERROR;
        }
        *enabled = true;
    }
    if (firmware == ARM_FIRMWARE_X)
        return LiftMotor_Send(x_velocity_frame, sizeof(x_velocity_frame), id, 0xF6U);
    return LiftMotor_Send(velocity_frame, sizeof(velocity_frame), id, 0xF6U);
}

static HAL_StatusTypeDef ArmMotor_Stop(uint8_t id, bool enabled)
{
    const uint8_t stop_frame[] =
        {id, 0xFEU, 0x98U, 0x00U, ZDT_MOTOR_FRAME_TAIL};

    if (!enabled)
    {
        return HAL_OK;
    }
    return LiftMotor_Send(stop_frame, sizeof(stop_frame), id, 0xFEU);
}

/* U/D 对应升/降，方向常量取决于机械安装；不接受其他动作字符。 */
HAL_StatusTypeDef LiftMotor_Move(char direction)
{
    if ((direction != 'U') && (direction != 'D')) return HAL_ERROR;
    return ArmMotor_Move(LIFT_MOTOR_ID, &s_lift_enabled,
                         (direction == 'U') ? LIFT_UP_DIRECTION :
                                              LIFT_DOWN_DIRECTION,
                         LIFT_SPEED_RPM);
}

HAL_StatusTypeDef LiftMotor_Stop(void)
{
    return ArmMotor_Stop(LIFT_MOTOR_ID, s_lift_enabled);
}

/* USART2 ID1/ID2 共用定角帧，02 从当前实时位置开始，停止后可重新测量。
 * Emm FD: 微步脉冲；X FD: 0.1°，不能混用两个固件的帧。 */
static HAL_StatusTypeDef ArmMotor_MoveAngleById(uint8_t id, ZDT_Direction direction,
                                               uint32_t angle_tenths,
                                               uint16_t speed_rpm, uint32_t pulses_per_rev,
                                               bool absolute)
{
    if ((id != LIFT_MOTOR_ID && id != FORE_AFT_MOTOR_ID) || (!absolute && angle_tenths == 0U) ||
        (!absolute && angle_tenths > 36000U) || speed_rpm < 5U || speed_rpm > 120U ||
        pulses_per_rev < 200U || pulses_per_rev > 51200U) return HAL_ERROR;
    uint8_t firmware = ArmMotor_DetectFirmware(id);
    if (firmware == ARM_FIRMWARE_UNKNOWN) return HAL_ERROR;
    uint64_t pulse_value = ((uint64_t)angle_tenths * pulses_per_rev + 1800U) / 3600U;
    if (firmware == ARM_FIRMWARE_EMM && (pulse_value > UINT32_MAX || (!absolute && pulse_value == 0U))) return HAL_ERROR;
    uint32_t pulses = (uint32_t)pulse_value;
    const uint8_t enable[] = {id, 0xF3U, 0xABU, 0x01U, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    /* 每次位置测试重新确认使能，避免沿用失效的使能状态。 */
    if (LiftMotor_Send(enable, sizeof(enable), id, 0xF3U) != HAL_OK) return HAL_ERROR;
    if (id == LIFT_MOTOR_ID) s_lift_enabled = true;
    else s_fore_aft_enabled = true;
    uint8_t dir = (uint8_t)direction;
    uint8_t emm[] = {id, 0xFDU, dir, (uint8_t)(speed_rpm >> 8), (uint8_t)speed_rpm,
        LIFT_ACCELERATION, (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)pulses, absolute ? 0x01U : 0x02U, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    uint16_t x_speed = speed_rpm * 10U;
    uint8_t x[] = {id, 0xFDU, dir, 0x00U, 0xC8U, 0x00U, 0xC8U,
        (uint8_t)(x_speed >> 8), (uint8_t)x_speed,
        (uint8_t)(angle_tenths >> 24), (uint8_t)(angle_tenths >> 16),
        (uint8_t)(angle_tenths >> 8), (uint8_t)angle_tenths,
        absolute ? 0x01U : 0x02U, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    return firmware == ARM_FIRMWARE_X ? LiftMotor_Send(x, sizeof(x), id, 0xFDU) :
                                       LiftMotor_Send(emm, sizeof(emm), id, 0xFDU);
}

HAL_StatusTypeDef ArmMotor_MoveAngle(char direction, uint32_t angle_tenths,
                                    uint16_t speed_rpm, uint32_t pulses_per_rev)
{
    if (direction != 'E' && direction != 'C') return HAL_ERROR;
    return ArmMotor_MoveAngleById(FORE_AFT_MOTOR_ID,
        direction == 'E' ? ARM_FORWARD_DIRECTION : ARM_BACK_DIRECTION,
        angle_tenths, speed_rpm, pulses_per_rev, false);
}

HAL_StatusTypeDef LiftMotor_MoveAngle(char direction, uint32_t angle_tenths,
                                     uint16_t speed_rpm, uint32_t pulses_per_rev)
{
    if (direction != 'U' && direction != 'D') return HAL_ERROR;
    return ArmMotor_MoveAngleById(LIFT_MOTOR_ID,
        direction == 'U' ? LIFT_UP_DIRECTION : LIFT_DOWN_DIRECTION,
        angle_tenths, speed_rpm, pulses_per_rev, false);
}

HAL_StatusTypeDef ArmMotor_MoveAbsolute(uint8_t id, int64_t angle_tenths,
                                       uint16_t rpm, uint32_t pulses_per_rev)
{
    if (angle_tenths < -(int64_t)UINT32_MAX || angle_tenths > UINT32_MAX) return HAL_ERROR;
    return ArmMotor_MoveAngleById(id, angle_tenths < 0 ? ZDT_DIRECTION_CCW : ZDT_DIRECTION_CW,
        (uint32_t)(angle_tenths < 0 ? -angle_tenths : angle_tenths), rpm, pulses_per_rev, true);
}

bool ArmMotor_ReadPosition(uint8_t id, int64_t *angle_tenths)
{
    if (angle_tenths == NULL || (id != 1U && id != 2U)) return false;
    uint8_t firmware = ArmMotor_DetectFirmware(id);
    if (firmware == ARM_FIRMWARE_UNKNOWN) return false;
    uint8_t request[] = {id, 0x36U, ZDT_MOTOR_FRAME_TAIL};
    uint8_t response[8], byte, count = 0U;
    if (HAL_UART_Transmit(s_uart, request, sizeof(request), 20U) != HAL_OK) return false;
    uint32_t start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < 20U) {
        uint32_t remaining = 20U - (uint32_t)(HAL_GetTick() - start);
        if (HAL_UART_Receive(s_uart, &byte, 1U, remaining) != HAL_OK) return false;
        response[count++] = byte;
        if (count < sizeof(response)) continue;
        if (response[0] == id && response[1] == 0x36U && response[2] <= 1U && response[7] == ZDT_MOTOR_FRAME_TAIL) {
            uint32_t raw = ((uint32_t)response[3] << 24) | ((uint32_t)response[4] << 16) |
                           ((uint32_t)response[5] << 8) | response[6];
            int64_t angle = firmware == ARM_FIRMWARE_X ? raw :
                (int64_t)(((uint64_t)raw * 3600U + 32768U) / 65536U);
            *angle_tenths = response[2] ? -angle : angle;
            return true;
        }
        for (unsigned i = 0; i < 7U; ++i) response[i] = response[i+1];
        count = 7U;
    }
    return false;
}

HAL_StatusTypeDef ArmMotor_StopForeAft(void)
{
    return ArmMotor_Stop(FORE_AFT_MOTOR_ID, s_fore_aft_enabled);
}

bool LiftMotor_UartReady(void)
{
    return s_uart != NULL;
}

bool LiftMotor_Probe(uint8_t id)
{
    return ArmMotor_DetectFirmware(id) != ARM_FIRMWARE_UNKNOWN;
}

char LiftMotor_FirmwareCode(uint8_t id)
{
    uint8_t firmware;
    if ((id < LIFT_MOTOR_ID) || (id > FORE_AFT_MOTOR_ID)) return '?';
    firmware = s_firmware[id - 1U];
    if (firmware == ARM_FIRMWARE_EMM) return 'E';
    if (firmware == ARM_FIRMWARE_X) return 'X';
    return '?';
}

const char *LiftMotor_LastError(uint8_t id)
{
    if ((id < LIFT_MOTOR_ID) || (id > FORE_AFT_MOTOR_ID))
        return "BAD_ID";
    return s_last_error[id - 1U];
}

bool LiftMotor_ReadStatus(uint8_t id, uint8_t *status)
{
    uint8_t request[] = {id, 0x3AU, ZDT_MOTOR_FRAME_TAIL};
    uint8_t response[4];
    uint8_t count = 0U;
    uint8_t byte;
    uint32_t start;

    if ((s_uart == NULL) || (status == NULL) ||
        (id < LIFT_MOTOR_ID) || (id > FORE_AFT_MOTOR_ID)) return false;
    if (HAL_UART_Transmit(s_uart, request, sizeof(request), 20U) != HAL_OK)
        return false;
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < 20U)
    {
        uint32_t remaining = 20U - (uint32_t)(HAL_GetTick() - start);
        if (HAL_UART_Receive(s_uart, &byte, 1U, remaining) != HAL_OK)
            return false;
        response[count++] = byte;
        if (count < sizeof(response)) continue;
        if ((response[0] == id) && (response[1] == 0x3AU) &&
            (response[3] == ZDT_MOTOR_FRAME_TAIL))
        {
            *status = response[2];
            return true;
        }
        response[0] = response[1];
        response[1] = response[2];
        response[2] = response[3];
        count = 3U;
    }
    return false;
}
