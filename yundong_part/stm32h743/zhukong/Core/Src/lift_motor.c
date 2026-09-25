#include "lift_motor.h"
#include "zdt_motor.h"

#define LIFT_MOTOR_ID          1U
#define LIFT_SPEED_RPM         30U
#define LIFT_ACCELERATION      10U
#define LIFT_ACK_TIMEOUT_MS    20U
/* Reverse these two values if the installed mechanism moves the other way. */
#define LIFT_UP_DIRECTION      ZDT_DIRECTION_CW
#define LIFT_DOWN_DIRECTION    ZDT_DIRECTION_CCW

static UART_HandleTypeDef *s_uart;
static bool s_enabled;

static HAL_StatusTypeDef LiftMotor_Send(const uint8_t *frame, uint16_t length,
                                        uint8_t function)
{
    uint8_t response[4];
    uint8_t byte;
    uint8_t count = 0U;
    uint8_t scanned = 0U;
    uint32_t start;

    if ((s_uart == NULL) ||
        (HAL_UART_Transmit(s_uart, (uint8_t *)frame, length,
                           LIFT_ACK_TIMEOUT_MS) != HAL_OK))
    {
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
        if ((response[0] == LIFT_MOTOR_ID) &&
            (response[1] == function) && (response[2] == 0x02U) &&
            (response[3] == ZDT_MOTOR_FRAME_TAIL))
        {
            return HAL_OK;
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
    return HAL_TIMEOUT;
}

void LiftMotor_Init(UART_HandleTypeDef *uart)
{
    s_uart = uart;
    s_enabled = false;
}

HAL_StatusTypeDef LiftMotor_Move(char direction)
{
    ZDT_Direction motor_direction;
    const uint8_t enable_frame[] =
        {LIFT_MOTOR_ID, 0xF3U, 0xABU, 0x01U, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    uint8_t velocity_frame[] =
        {LIFT_MOTOR_ID, 0xF6U, 0U, 0U, LIFT_SPEED_RPM,
         LIFT_ACCELERATION, 0x00U, ZDT_MOTOR_FRAME_TAIL};

    if ((direction != 'U') && (direction != 'D'))
    {
        return HAL_ERROR;
    }
    motor_direction = (direction == 'U') ? LIFT_UP_DIRECTION :
                                              LIFT_DOWN_DIRECTION;
    if (!s_enabled)
    {
        if (LiftMotor_Send(enable_frame, sizeof(enable_frame), 0xF3U) != HAL_OK)
        {
            return HAL_ERROR;
        }
        s_enabled = true;
    }
    velocity_frame[2] = (uint8_t)motor_direction;
    return LiftMotor_Send(velocity_frame, sizeof(velocity_frame), 0xF6U);
}

HAL_StatusTypeDef LiftMotor_Stop(void)
{
    const uint8_t stop_frame[] =
        {LIFT_MOTOR_ID, 0xFEU, 0x98U, 0x00U, ZDT_MOTOR_FRAME_TAIL};

    if (!s_enabled)
    {
        return HAL_OK;
    }
    return LiftMotor_Send(stop_frame, sizeof(stop_frame), 0xFEU);
}
