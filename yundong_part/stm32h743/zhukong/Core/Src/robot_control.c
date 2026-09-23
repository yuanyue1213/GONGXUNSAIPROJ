/**
 ******************************************************************************
 * @file    robot_control.c
 * @brief   ESP32 <-> STM32 遥控协议处理。
 ******************************************************************************
 */

#include "robot_control.h"
#include "zdt_motor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ROBOT_COMMAND_MAX_LENGTH       63U
#define ROBOT_COMMAND_TIMEOUT_MS       300U
#define ROBOT_SPEED_RPM                45
#define ROBOT_ACCELERATION             10U
#define ROBOT_UART_TX_TIMEOUT_MS       20U

static UART_HandleTypeDef *s_command_uart;
static char s_frame[ROBOT_COMMAND_MAX_LENGTH + 1U];
static uint16_t s_frame_length;
static bool s_dropping_frame;
static char s_motion = 'S';
static bool s_motors_enabled;
static uint32_t s_last_command_tick;

static void RobotControl_Send(const char *message);
static bool RobotControl_Parse(const char *frame, uint32_t *sequence,
                               char *direction);
static HAL_StatusTypeDef RobotControl_EnableMotors(void);
static HAL_StatusTypeDef RobotControl_StopMotors(void);
static HAL_StatusTypeDef RobotControl_ApplyMotion(char direction);
static void RobotControl_HandleFrame(const char *frame);
static void RobotControl_RecoverReceiveErrors(void);

void RobotControl_Init(UART_HandleTypeDef *command_uart)
{
    s_command_uart = command_uart;
    s_frame_length = 0U;
    s_dropping_frame = false;
    s_motion = 'S';
    s_motors_enabled = false;
    s_last_command_tick = HAL_GetTick();

    /* 上电时不使能电机；收到第一条有效运动命令时才使能。 */
    RobotControl_Send("READY\n");
}

void RobotControl_Process(void)
{
    uint8_t byte;

    if (s_command_uart == NULL)
    {
        return;
    }

    while (HAL_UART_Receive(s_command_uart, &byte, 1U, 0U) == HAL_OK)
    {
        if (byte == '\n')
        {
            if (s_dropping_frame)
            {
                RobotControl_Send("ERR,0,TOO_LONG\n");
            }
            else
            {
                if ((s_frame_length > 0U) &&
                    (s_frame[s_frame_length - 1U] == '\r'))
                {
                    --s_frame_length;
                }
                s_frame[s_frame_length] = '\0';
                RobotControl_HandleFrame(s_frame);
            }
            s_frame_length = 0U;
            s_dropping_frame = false;
        }
        else if (!s_dropping_frame)
        {
            if (s_frame_length < ROBOT_COMMAND_MAX_LENGTH)
            {
                s_frame[s_frame_length++] = (char)byte;
            }
            else
            {
                s_dropping_frame = true;
            }
        }
    }

    /* Motor commands can briefly block this polling loop.  USART3 has only
     * a small receive buffer, so discard an incomplete frame and clear UART
     * errors instead of leaving the receiver permanently stalled after an
     * overrun.  The phone sends the next command every 50 ms while held. */
    RobotControl_RecoverReceiveErrors();
}

void RobotControl_Tick(void)
{
    if ((s_motion != 'S') &&
        ((uint32_t)(HAL_GetTick() - s_last_command_tick) >=
         ROBOT_COMMAND_TIMEOUT_MS))
    {
        (void)RobotControl_StopMotors();
        s_motion = 'S';
        RobotControl_Send("EVENT,STOP,TIMEOUT\n");
    }
}

static void RobotControl_Send(const char *message)
{
    if ((s_command_uart != NULL) && (message != NULL))
    {
        (void)HAL_UART_Transmit(s_command_uart, (uint8_t *)message,
                                (uint16_t)strlen(message),
                                ROBOT_UART_TX_TIMEOUT_MS);
    }
}

static bool RobotControl_Parse(const char *frame, uint32_t *sequence,
                               char *direction)
{
    const char *cursor;
    uint32_t value = 0U;

    if ((frame == NULL) || (sequence == NULL) || (direction == NULL) ||
        (strncmp(frame, "CMD,", 4U) != 0))
    {
        return false;
    }

    cursor = frame + 4U;
    if ((*cursor < '0') || (*cursor > '9'))
    {
        return false;
    }

    do
    {
        uint32_t digit = (uint32_t)(*cursor - '0');
        if (value > (UINT32_MAX - digit) / 10U)
        {
            return false;
        }
        value = value * 10U + digit;
        ++cursor;
    } while ((*cursor >= '0') && (*cursor <= '9'));

    if ((cursor[0] != ',') || (cursor[1] == '\0') || (cursor[2] != '\0'))
    {
        return false;
    }

    switch (cursor[1])
    {
    case 'F':
    case 'B':
    case 'L':
    case 'R':
    case 'S':
        *sequence = value;
        *direction = cursor[1];
        return true;
    default:
        return false;
    }
}

static HAL_StatusTypeDef RobotControl_EnableMotors(void)
{
    static const ZDT_WheelId wheels[ZDT_MOTOR_WHEEL_COUNT] =
    {
        ZDT_WHEEL_LEFT_UP,
        ZDT_WHEEL_RIGHT_UP,
        ZDT_WHEEL_RIGHT_DOWN,
        ZDT_WHEEL_LEFT_DOWN
    };
    uint32_t index;

    if (s_motors_enabled)
    {
        return HAL_OK;
    }

    for (index = 0U; index < ZDT_MOTOR_WHEEL_COUNT; ++index)
    {
        if (ZDT_Motor_Enable((uint8_t)wheels[index], true) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    s_motors_enabled = true;
    return HAL_OK;
}

static HAL_StatusTypeDef RobotControl_StopMotors(void)
{
    static const ZDT_WheelId wheels[ZDT_MOTOR_WHEEL_COUNT] =
    {
        ZDT_WHEEL_LEFT_UP,
        ZDT_WHEEL_RIGHT_UP,
        ZDT_WHEEL_RIGHT_DOWN,
        ZDT_WHEEL_LEFT_DOWN
    };
    uint32_t index;
    HAL_StatusTypeDef result = HAL_OK;

    if (!s_motors_enabled)
    {
        return HAL_OK;
    }

    for (index = 0U; index < ZDT_MOTOR_WHEEL_COUNT; ++index)
    {
        if (ZDT_Motor_Stop((uint8_t)wheels[index], false) != HAL_OK)
        {
            result = HAL_ERROR;
        }
    }
    return result;
}

static HAL_StatusTypeDef RobotControl_ApplyMotion(char direction)
{
    int16_t wheel_speeds[ZDT_MOTOR_WHEEL_COUNT];

    if (direction == 'S')
    {
        return RobotControl_StopMotors();
    }

    if (RobotControl_EnableMotors() != HAL_OK)
    {
        return HAL_ERROR;
    }

    switch (direction)
    {
    case 'F':
        wheel_speeds[0] = -ROBOT_SPEED_RPM;
        wheel_speeds[1] = -ROBOT_SPEED_RPM;
        wheel_speeds[2] = -ROBOT_SPEED_RPM;
        wheel_speeds[3] = -ROBOT_SPEED_RPM;
        break;
    case 'B':
        wheel_speeds[0] = ROBOT_SPEED_RPM;
        wheel_speeds[1] = ROBOT_SPEED_RPM;
        wheel_speeds[2] = ROBOT_SPEED_RPM;
        wheel_speeds[3] = ROBOT_SPEED_RPM;
        break;
    case 'L':
        /* 麦克纳姆轮左平移（实车前后方向已反标定）。
         * 轮序：左前、右前、右后、左后。 */
        wheel_speeds[0] = ROBOT_SPEED_RPM;
        wheel_speeds[1] = -ROBOT_SPEED_RPM;
        wheel_speeds[2] = ROBOT_SPEED_RPM;
        wheel_speeds[3] = -ROBOT_SPEED_RPM;
        break;
    case 'R':
        /* 麦克纳姆轮右平移。 */
        wheel_speeds[0] = -ROBOT_SPEED_RPM;
        wheel_speeds[1] = ROBOT_SPEED_RPM;
        wheel_speeds[2] = -ROBOT_SPEED_RPM;
        wheel_speeds[3] = ROBOT_SPEED_RPM;
        break;
    default:
        return HAL_ERROR;
    }

    return ZDT_Motor_SetWheelSpeeds(wheel_speeds, ROBOT_ACCELERATION);
}

static void RobotControl_RecoverReceiveErrors(void)
{
    if (s_command_uart == NULL)
    {
        return;
    }

    if (__HAL_UART_GET_FLAG(s_command_uart, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(s_command_uart);
        s_frame_length = 0U;
        s_dropping_frame = false;
    }
    if (__HAL_UART_GET_FLAG(s_command_uart, UART_FLAG_NE))
    {
        __HAL_UART_CLEAR_NEFLAG(s_command_uart);
    }
    if (__HAL_UART_GET_FLAG(s_command_uart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FEFLAG(s_command_uart);
    }
}

static void RobotControl_HandleFrame(const char *frame)
{
    uint32_t sequence;
    char direction;
    char response[40];

    if (!RobotControl_Parse(frame, &sequence, &direction))
    {
        RobotControl_Send("ERR,0,BAD_FRAME\n");
        return;
    }

    s_last_command_tick = HAL_GetTick();

    /* A held phone button repeats the same command every 50 ms.  The motor
     * controller retains its speed command, therefore repeated frames only
     * refresh the safety timeout; they must not repeatedly block on motor
     * acknowledgements and starve USART3 reception. */
    if (direction == s_motion)
    {
        return;
    }

    if (RobotControl_ApplyMotion(direction) != HAL_OK)
    {
        snprintf(response, sizeof(response), "ERR,%lu,MOTOR\n",
                 (unsigned long)sequence);
        RobotControl_Send(response);
        return;
    }

    s_motion = direction;
    snprintf(response, sizeof(response), "EXEC,%lu,%c\n",
             (unsigned long)sequence, direction);
    RobotControl_Send(response);
}
