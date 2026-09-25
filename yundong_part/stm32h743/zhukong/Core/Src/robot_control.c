/**
 ******************************************************************************
 * @file    robot_control.c
 * @brief   ESP32 <-> STM32 遥控协议处理。
 ******************************************************************************
 */

#include "robot_control.h"
#include "zdt_motor.h"
#include "lift_motor.h"
#include "servo_control.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define ROBOT_COMMAND_MAX_LENGTH       63U
#define ROBOT_COMMAND_TIMEOUT_MS       300U
#define ROBOT_SPEED_RPM                45
#define ROBOT_ACCELERATION             10U
#define ROBOT_UART_TX_TIMEOUT_MS       20U

static UART_HandleTypeDef *s_command_uart;
static bool s_wheel_uart_ready;
static char s_frame[ROBOT_COMMAND_MAX_LENGTH + 1U];
static uint16_t s_frame_length;
static bool s_dropping_frame;
static char s_motion = 'S';
static bool s_motors_enabled;
static uint32_t s_last_command_tick;
static char s_lift_motion = 'H';
static bool s_lift_command_ok = true;
static uint32_t s_last_lift_tick;
static char s_fore_aft_motion = 'Q';
static bool s_fore_aft_command_ok = true;
static uint32_t s_last_fore_aft_tick;

static void RobotControl_Send(const char *message);
static bool RobotControl_Parse(const char *frame, uint32_t *sequence,
                               char *direction);
static bool RobotControl_ParseServo(const char *frame, uint32_t *sequence,
                                    char *channel, uint16_t *angle);
static bool RobotControl_ParseDiag(const char *frame, uint32_t *sequence);
static HAL_StatusTypeDef RobotControl_EnableMotors(void);
static HAL_StatusTypeDef RobotControl_StopMotors(void);
static HAL_StatusTypeDef RobotControl_ApplyMotion(char direction);
static void RobotControl_HandleFrame(const char *frame);
static void RobotControl_RecoverReceiveErrors(void);

void RobotControl_Init(UART_HandleTypeDef *command_uart, bool wheel_uart_ready)
{
    s_command_uart = command_uart;
    s_wheel_uart_ready = wheel_uart_ready;
    s_frame_length = 0U;
    s_dropping_frame = false;
    s_motion = 'S';
    s_motors_enabled = false;
    s_last_command_tick = HAL_GetTick();
    s_lift_motion = 'H';
    s_lift_command_ok = true;
    s_last_lift_tick = HAL_GetTick();
    s_fore_aft_motion = 'Q';
    s_fore_aft_command_ok = true;
    s_last_fore_aft_tick = HAL_GetTick();

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
    if ((s_lift_motion != 'H') &&
        ((uint32_t)(HAL_GetTick() - s_last_lift_tick) >=
         ROBOT_COMMAND_TIMEOUT_MS))
    {
        if (LiftMotor_Stop() == HAL_OK)
        {
            s_lift_motion = 'H';
            s_lift_command_ok = true;
            RobotControl_Send("EVENT,LIFT,STOP,TIMEOUT\n");
        }
    }
    if ((s_fore_aft_motion != 'Q') &&
        ((uint32_t)(HAL_GetTick() - s_last_fore_aft_tick) >=
         ROBOT_COMMAND_TIMEOUT_MS))
    {
        if (ArmMotor_StopForeAft() == HAL_OK)
        {
            s_fore_aft_motion = 'Q';
            s_fore_aft_command_ok = true;
            RobotControl_Send("EVENT,ARM_FORE_AFT,STOP,TIMEOUT\n");
        }
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
    case 'U':
    case 'D':
    case 'H':
    case 'E':
    case 'C':
    case 'Q':
        *sequence = value;
        *direction = cursor[1];
        return true;
    default:
        return false;
    }
}

static bool RobotControl_ParseServo(const char *frame, uint32_t *sequence,
                                    char *channel, uint16_t *angle)
{
    char *end;
    const char *cursor;
    unsigned long value;
    uint16_t maximum;

    if ((frame == NULL) || (strncmp(frame, "SERVO,", 6U) != 0))
    {
        return false;
    }
    cursor = frame + 6U;
    if ((*cursor < '0') || (*cursor > '9')) return false;
    errno = 0;
    value = strtoul(cursor, &end, 10);
    if ((errno == ERANGE) || (value > UINT32_MAX) || (*end != ','))
        return false;
    *sequence = (uint32_t)value;
    cursor = end + 1;
    if ((cursor[0] == '\0') || (cursor[1] != ',')) return false;
    *channel = cursor[0];
    maximum = (*channel == 'B') ? 360U : 270U;
    if ((*channel != 'G') && (*channel != 'T') && (*channel != 'B'))
        return false;
    cursor += 2;
    if ((*cursor < '0') || (*cursor > '9')) return false;
    errno = 0;
    value = strtoul(cursor, &end, 10);
    if ((errno == ERANGE) || (*end != '\0') || (value > maximum))
        return false;
    *angle = (uint16_t)value;
    return true;
}

static bool RobotControl_ParseDiag(const char *frame, uint32_t *sequence)
{
    char *end;
    const char *cursor;
    unsigned long value;

    if (strncmp(frame, "DIAG,", 5U) != 0) return false;
    cursor = frame + 5U;
    if ((*cursor < '0') || (*cursor > '9')) return false;
    errno = 0;
    value = strtoul(cursor, &end, 10);
    if ((errno == ERANGE) || (value > UINT32_MAX) || (*end != '\0'))
        return false;
    *sequence = (uint32_t)value;
    return true;
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
    uint16_t angle;
    char response[64];

    if (strncmp(frame, "DIAG,", 5U) == 0)
    {
        uint8_t uart_ready;
        uint8_t id1_ready;
        uint8_t id2_ready;
        uint8_t status1 = 0xFFU;
        uint8_t status2 = 0xFFU;
        char diagnostic[80];
        if (!RobotControl_ParseDiag(frame, &sequence))
        {
            RobotControl_Send("ERR,0,BAD_FRAME\n");
            return;
        }
        snprintf(response, sizeof(response), "DIAG_BEGIN,%lu,UART1=%u\n",
                 (unsigned long)sequence, (unsigned int)s_wheel_uart_ready);
        RobotControl_Send(response);
        uart_ready = LiftMotor_UartReady() ? 1U : 0U;
        id1_ready = LiftMotor_Probe(1U) ? 1U : 0U;
        id2_ready = LiftMotor_Probe(2U) ? 1U : 0U;
        if (id1_ready != 0U) (void)LiftMotor_ReadStatus(1U, &status1);
        if (id2_ready != 0U) (void)LiftMotor_ReadStatus(2U, &status2);
        snprintf(diagnostic, sizeof(diagnostic),
                 "DIAG,%lu,UART2=%u,ID1=%u,FW1=%c,S1=%02X,ID2=%u,FW2=%c,S2=%02X\n",
                 (unsigned long)sequence, (unsigned int)uart_ready,
                 (unsigned int)id1_ready, LiftMotor_FirmwareCode(1U),
                 (unsigned int)status1, (unsigned int)id2_ready,
                 LiftMotor_FirmwareCode(2U), (unsigned int)status2);
        RobotControl_Send(diagnostic);
        return;
    }

    if (strncmp(frame, "SERVO,", 6U) == 0)
    {
        if (!RobotControl_ParseServo(frame, &sequence, &direction, &angle))
        {
            RobotControl_Send("ERR,0,BAD_FRAME\n");
            return;
        }
        if (!ServoControl_SetAngle(direction, angle))
        {
            snprintf(response, sizeof(response), "ERR,%lu,SERVO\n",
                     (unsigned long)sequence);
        }
        else
        {
            snprintf(response, sizeof(response), "EXEC,%lu,SERVO,%c,%u\n",
                     (unsigned long)sequence, direction, (unsigned int)angle);
        }
        RobotControl_Send(response);
        return;
    }

    if (!RobotControl_Parse(frame, &sequence, &direction))
    {
        RobotControl_Send("ERR,0,BAD_FRAME\n");
        return;
    }

    if ((direction == 'U') || (direction == 'D') || (direction == 'H'))
    {
        s_last_lift_tick = HAL_GetTick();
        if ((direction == s_lift_motion) && s_lift_command_ok)
        {
            return;
        }
        if (((direction == 'H') ? LiftMotor_Stop() :
             LiftMotor_Move(direction)) != HAL_OK)
        {
            /* The command may have reached the motor even without an ACK. */
            if (direction != 'H') s_lift_motion = direction;
            s_lift_command_ok = false;
            snprintf(response, sizeof(response), "ERR,%lu,LIFT_MOTOR,%s\n",
                     (unsigned long)sequence, LiftMotor_LastError(1U));
            RobotControl_Send(response);
            return;
        }
        s_lift_motion = direction;
        s_lift_command_ok = true;
        snprintf(response, sizeof(response), "EXEC,%lu,%c\n",
                 (unsigned long)sequence, direction);
        RobotControl_Send(response);
        return;
    }

    if ((direction == 'E') || (direction == 'C') || (direction == 'Q'))
    {
        s_last_fore_aft_tick = HAL_GetTick();
        if ((direction == s_fore_aft_motion) && s_fore_aft_command_ok)
        {
            return;
        }
        if (((direction == 'Q') ? ArmMotor_StopForeAft() :
             ArmMotor_MoveForeAft(direction)) != HAL_OK)
        {
            if (direction != 'Q') s_fore_aft_motion = direction;
            s_fore_aft_command_ok = false;
            snprintf(response, sizeof(response), "ERR,%lu,ARM_FORE_AFT_MOTOR,%s\n",
                     (unsigned long)sequence, LiftMotor_LastError(2U));
            RobotControl_Send(response);
            return;
        }
        s_fore_aft_motion = direction;
        s_fore_aft_command_ok = true;
        snprintf(response, sizeof(response), "EXEC,%lu,%c\n",
                 (unsigned long)sequence, direction);
        RobotControl_Send(response);
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
