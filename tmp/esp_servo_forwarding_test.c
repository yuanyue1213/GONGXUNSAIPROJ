#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "../yundong_part/shared/robot_distance_protocol.h"
#include "../yundong_part/shared/camera_position_protocol.h"
#define MAX_FRAME_LENGTH 63
static char forwarded[128];
static bool send_to_stm32(const char *frame) { strcpy(forwarded, frame); return true; }
static int64_t esp_timer_get_time(void) { return 0; }
typedef enum {
    MOTION_STOP = 'S',
    MOTION_FORWARD = 'F',
    MOTION_BACKWARD = 'B',
    MOTION_LEFT = 'L',
    MOTION_RIGHT = 'R',
    LIFT_UP = 'U',
    LIFT_DOWN = 'D',
    LIFT_HOLD = 'H',
    ARM_FORE_AFT_HOLD = 'Q',
} motion_t;

/* 仅升降维护续期状态；底盘和前后定距只转发命令，不等待回包。 */
static motion_t current_lift = LIFT_HOLD;

static bool parse_command(const char *frame, uint32_t *sequence, motion_t *motion)
{
    if (strncmp(frame, "CMD,", 4) != 0) {
        return false;
    }

    const char *cursor = frame + 4;
    if (*cursor < '0' || *cursor > '9') {
        return false;
    }

    uint32_t value = 0;
    do {
        uint32_t digit = (uint32_t)(*cursor - '0');
        if (value > (UINT32_MAX - digit) / 10U) {
            return false;
        }
        value = value * 10U + digit;
        ++cursor;
    } while (*cursor >= '0' && *cursor <= '9');

    if (cursor[0] != ',' || cursor[1] == '\0' || cursor[2] != '\0') {
        return false;
    }

    switch (cursor[1]) {
    case 'I': /* App 启动原点初始化 */
    case 'O': /* 返回启动原点 */
    case 'A': /* 抓取固定状态 */
    case 'Z': /* 取消自动舵机步骤 */
    case 'S':
    case 'U':
    case 'D':
    case 'H':
    case 'Q':
        *sequence = value;
        *motion = (motion_t)cursor[1];
        return true;
    default:
        return false;
    }
}

/* SERVO,seq,channel,angle：G 夹子、T 转盘、B 基座。
 * G/T 最大 270°，B 最大 360°；角度校验后原样转交 STM32。 */
static bool parse_servo(const char *frame, uint32_t *sequence,
                        char *channel, uint16_t *angle)
{
    const char *cursor;
    char *end;
    unsigned long value;
    uint16_t maximum;

    if (strncmp(frame, "SERVO,", 6) != 0) return false;
    cursor = frame + 6;
    if (*cursor < '0' || *cursor > '9') return false;
    errno = 0;
    value = strtoul(cursor, &end, 10);
    if (errno == ERANGE || value > UINT32_MAX || *end != ',') return false;
    *sequence = (uint32_t)value;
    cursor = end + 1;
    if (cursor[0] == '\0' || cursor[1] != ',') return false;
    *channel = cursor[0];
    if (*channel != 'G' && *channel != 'T' && *channel != 'B') return false;
    maximum = (*channel == 'B') ? 360U : 270U;
    cursor += 2;
    if (*cursor < '0' || *cursor > '9') return false;
    errno = 0;
    value = strtoul(cursor, &end, 10);
    if (errno == ERANGE || *end != '\0' || value > maximum) return false;
    *angle = (uint16_t)value;
    return true;
}

/* 单向命令：非法帧直接丢弃，UART 写失败则结束连接并请求停车。
 * ESP32 不维护底盘 BUSY 状态，由 STM32 在本地判断是否接受新 MOVE。 */
static bool handle_frame(const char *frame, int64_t *last_lift_us)
{
    uint32_t sequence;
    motion_t motion;
    if (strncmp(frame, "ARM_POSE,", 9U) == 0) {
        RobotArmPoseCommand command;
        if (!RobotProtocol_ParseArmPose(frame, &command)) return true;
    } else if (strncmp(frame, "LIFT_MOVE,", 10U) == 0) {
        RobotLiftDistanceCommand command;
        if (!RobotProtocol_ParseLiftDistance(frame, &command)) return true;
    } else if (strncmp(frame, "LIFT_ANGLE,", 11U) == 0) {
        RobotLiftAngleCommand command;
        if (!RobotProtocol_ParseLiftAngle(frame, &command)) return true;
    } else if (strncmp(frame, "ALIGN,", 6U) == 0) {
        CameraAlignCommand command;
        if (!CameraProtocol_ParseAlign(frame, &command)) return true;
    } else if (strncmp(frame, "ARM_MOVE,", 9U) == 0) {
        RobotArmDistanceCommand command;
        if (!RobotProtocol_ParseArmDistance(frame, &command)) return true;
    } else if (strncmp(frame, "MOVE,", 5U) == 0) {
        RobotDistanceCommand command;
        if (!RobotProtocol_ParseMove(frame, &command) ||
            RobotProtocol_MovePulses(&command) == 0U) return true;
    } else if (strncmp(frame, "SERVO,", 6U) == 0) {
        char channel;
        uint16_t angle;
        if (!parse_servo(frame, &sequence, &channel, &angle)) return true;
    } else {
        if (!parse_command(frame, &sequence, &motion)) return true;
        char command[MAX_FRAME_LENGTH + 2];
        snprintf(command, sizeof(command), "%s\n", frame);
        if (!send_to_stm32(command)) return false;
        if (motion == LIFT_UP || motion == LIFT_DOWN || motion == LIFT_HOLD) {
            current_lift = motion;
            *last_lift_us = esp_timer_get_time();
        }
        return true;
    }
    char command[MAX_FRAME_LENGTH + 2];
    snprintf(command, sizeof(command), "%s\n", frame);
    bool sent = send_to_stm32(command);
    /* Autonomous lift angle tests do not use the continuous-lift renewal timer. */
    if (sent && (strncmp(frame, "ARM_POSE,", 9U) == 0 || strncmp(frame, "LIFT_ANGLE,", 11U) == 0 ||
                 strncmp(frame, "LIFT_MOVE,", 10U) == 0)) current_lift = LIFT_HOLD;
    return sent;
}


int main(void) {
    int64_t lift = 0;
    const char *commands[] = {"SERVO,1577625090,G,90", "SERVO,1577625091,T,270", "SERVO,1577625092,B,360", "CMD,1577625093,A", "CMD,1577625094,Z", "ARM_MOVE,1577625095,E,127,10,3200", "ALIGN,1577625096,9889,12000", "LIFT_ANGLE,1577625097,U,90,5,3200", "LIFT_ANGLE,1577625098,D,360,10,6400", "LIFT_MOVE,1577625099,U,10,5,3200", "LIFT_MOVE,1577625100,D,40,10,6400"};
    for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        char expected[128];
        snprintf(expected, sizeof(expected), "%s\n", commands[i]);
        assert(handle_frame(commands[i], &lift));
        assert(strcmp(forwarded, expected) == 0);
    }
    forwarded[0] = 0;
    assert(handle_frame("ARM_POSE,1,120,-20,40,5,3200", &lift));
    assert(strcmp(forwarded, "ARM_POSE,1,120,-20,40,5,3200\n") == 0);
    assert(current_lift == LIFT_HOLD);
    assert(handle_frame("CMD,2,O", &lift));
    assert(handle_frame("CMD,3,I", &lift));
    assert(strcmp(forwarded, "CMD,3,I\n") == 0);
    assert(handle_frame("CMD,2,O", &lift));
    assert(strcmp(forwarded, "CMD,2,O\n") == 0);
    forwarded[0] = 0;
    assert(handle_frame("ARM_POSE,3,0,0,401,5,3200", &lift));
    assert(forwarded[0] == 0);
    assert(handle_frame("ALIGN,1,0,9889", &lift));
    assert(forwarded[0] == 0);
    assert(handle_frame("ALIGN,1,9889", &lift));
    assert(forwarded[0] == 0);
    assert(handle_frame("LIFT_ANGLE,1,U,0,5,3200", &lift));
    assert(forwarded[0] == 0);
    current_lift = LIFT_UP;
    assert(handle_frame("LIFT_MOVE,1,U,401,5,3200", &lift));
    assert(forwarded[0] == 0 && current_lift == LIFT_UP);
    assert(handle_frame("LIFT_MOVE,2,U,10,5,3200", &lift));
    assert(current_lift == LIFT_HOLD);
    current_lift = LIFT_UP;
    assert(handle_frame("LIFT_ANGLE,2,U,90,5,3200", &lift));
    assert(current_lift == LIFT_HOLD);
    puts("PASS: ESP32 forwards SERVO/state/ARM/ALIGN/LIFT_ANGLE commands without replies");
}
