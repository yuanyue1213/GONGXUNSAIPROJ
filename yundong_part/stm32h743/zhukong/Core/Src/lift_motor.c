#include "lift_motor.h"
#include "zdt_motor.h"

#define LIFT_MOTOR_ID          1U
#define FORE_AFT_MOTOR_ID      2U
#define LIFT_SPEED_RPM         30U
#define FORE_AFT_SPEED_RPM     30U
#define LIFT_ACCELERATION      10U
#define LIFT_ACK_TIMEOUT_MS    20U
/* Reverse these two values if the installed mechanism moves the other way. */
#define LIFT_UP_DIRECTION      ZDT_DIRECTION_CW
#define LIFT_DOWN_DIRECTION    ZDT_DIRECTION_CCW
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

HAL_StatusTypeDef ArmMotor_MoveForeAft(char direction)
{
    if ((direction != 'E') && (direction != 'C')) return HAL_ERROR;
    return ArmMotor_Move(FORE_AFT_MOTOR_ID, &s_fore_aft_enabled,
                         (direction == 'E') ? ARM_FORWARD_DIRECTION :
                                              ARM_BACK_DIRECTION,
                         FORE_AFT_SPEED_RPM);
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
