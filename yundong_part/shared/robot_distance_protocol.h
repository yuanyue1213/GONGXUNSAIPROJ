#ifndef ROBOT_DISTANCE_PROTOCOL_H
#define ROBOT_DISTANCE_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    uint32_t sequence;
    char direction;
    uint32_t distance_mm;
    uint32_t speed_rpm;
    uint32_t pulses_per_metre;
} RobotDistanceCommand;

/* Strict decimal parsing, identical on both MCUs; reject signs/space/overflow. */
static inline bool RobotProtocol_U32(const char **cursor, uint32_t *value,
                                     char delimiter)
{
    const char *p = *cursor;
    uint32_t number = 0U;
    if ((*p < '0') || (*p > '9')) return false;
    do
    {
        uint32_t digit = (uint32_t)(*p - '0');
        if (number > (UINT32_MAX - digit) / 10U) return false;
        number = number * 10U + digit;
        ++p;
    } while ((*p >= '0') && (*p <= '9'));
    if (*p != delimiter) return false;
    *value = number;
    *cursor = p + (delimiter != '\0' ? 1 : 0);
    return true;
}

static inline bool RobotProtocol_ParseMove(const char *frame,
                                           RobotDistanceCommand *command)
{
    const char *p;
    if (strncmp(frame, "MOVE,", 5U) != 0) return false;
    p = frame + 5U;
    if (!RobotProtocol_U32(&p, &command->sequence, ',')) return false;
    command->direction = *p;
    if ((*p == '\0') || (p[1] != ',') ||
        ((*p != 'F') && (*p != 'B') && (*p != 'L') && (*p != 'R')))
        return false;
    p += 2;
    if (!RobotProtocol_U32(&p, &command->distance_mm, ',') ||
        !RobotProtocol_U32(&p, &command->speed_rpm, ',') ||
        !RobotProtocol_U32(&p, &command->pulses_per_metre, '\0')) return false;
    return (command->sequence != 0U) &&
           (command->distance_mm >= 1U) && (command->distance_mm <= 10000U) &&
           (command->speed_rpm >= 5U) && (command->speed_rpm <= 300U) &&
           (command->pulses_per_metre >= 1U) &&
           (command->pulses_per_metre <= 1000000U);
}

static inline uint32_t RobotProtocol_MovePulses(const RobotDistanceCommand *command)
{
    return (uint32_t)(((uint64_t)command->distance_mm *
                      command->pulses_per_metre + 500U) / 1000U);
}

/* 前后定距：实测电机一圈移动 127 mm；细分参数只用于 Emm。 */
typedef struct {
    uint32_t sequence;
    char direction;
    uint32_t distance_mm;
    uint32_t speed_rpm;
    uint32_t pulses_per_revolution;
} RobotArmDistanceCommand;

static inline bool RobotProtocol_ParseArmDistance(const char *frame, RobotArmDistanceCommand *command)
{
    if (strncmp(frame, "ARM_MOVE,", 9U) != 0) return false;
    const char *p = frame + 9U;
    if (!RobotProtocol_U32(&p, &command->sequence, ',')) return false;
    command->direction = *p;
    if ((*p != 'E' && *p != 'C') || p[1] != ',') return false;
    p += 2;
    if (!RobotProtocol_U32(&p, &command->distance_mm, ',') ||
        !RobotProtocol_U32(&p, &command->speed_rpm, ',') ||
        !RobotProtocol_U32(&p, &command->pulses_per_revolution, '\0')) return false;
    return command->sequence != 0U && command->distance_mm >= 1U &&
           command->distance_mm <= 1000U && command->speed_rpm >= 5U &&
           command->speed_rpm <= 60U && command->pulses_per_revolution >= 200U &&
           command->pulses_per_revolution <= 51200U;
}
static inline uint32_t RobotProtocol_ArmDistanceAngle(const RobotArmDistanceCommand *command)
{
    /* 输出 0.1°，先按实测 127 mm/圈换算，再由驱动选择 Emm/X 帧。 */
    return (command->distance_mm * 3600U + 63U) / 127U;
}

/* Lift calibration: turn by a known motor angle and measure travel manually. */
typedef struct {
    uint32_t sequence;
    char direction;
    uint32_t angle_degrees;
    uint32_t speed_rpm;
    uint32_t pulses_per_revolution;
} RobotLiftAngleCommand;

static inline bool RobotProtocol_ParseLiftAngle(const char *frame, RobotLiftAngleCommand *command)
{
    if (strncmp(frame, "LIFT_ANGLE,", 11U) != 0) return false;
    const char *p = frame + 11U;
    if (!RobotProtocol_U32(&p, &command->sequence, ',')) return false;
    command->direction = *p;
    if ((*p != 'U' && *p != 'D') || p[1] != ',') return false;
    p += 2;
    if (!RobotProtocol_U32(&p, &command->angle_degrees, ',') ||
        !RobotProtocol_U32(&p, &command->speed_rpm, ',') ||
        !RobotProtocol_U32(&p, &command->pulses_per_revolution, '\0')) return false;
    return command->sequence != 0U && command->angle_degrees >= 1U &&
           command->angle_degrees <= 3600U && command->speed_rpm >= 5U &&
           command->speed_rpm <= 60U && command->pulses_per_revolution >= 200U &&
           command->pulses_per_revolution <= 51200U;
}

/* Measured lift travel: 360 degrees = 40 mm. */
#define ROBOT_LIFT_MM_PER_REV 40U
typedef struct {
    uint32_t sequence;
    char direction;
    uint32_t distance_mm;
    uint32_t speed_rpm;
    uint32_t pulses_per_revolution;
} RobotLiftDistanceCommand;

static inline bool RobotProtocol_ParseLiftDistance(const char *frame, RobotLiftDistanceCommand *command)
{
    if (strncmp(frame, "LIFT_MOVE,", 10U) != 0) return false;
    const char *p = frame + 10U;
    if (!RobotProtocol_U32(&p, &command->sequence, ',')) return false;
    command->direction = *p;
    if ((*p != 'U' && *p != 'D') || p[1] != ',') return false;
    p += 2;
    if (!RobotProtocol_U32(&p, &command->distance_mm, ',') ||
        !RobotProtocol_U32(&p, &command->speed_rpm, ',') ||
        !RobotProtocol_U32(&p, &command->pulses_per_revolution, '\0')) return false;
    /* Same ten-turn limit as the motor angle test; not a mechanical travel limit. */
    return command->sequence != 0U && command->distance_mm >= 1U &&
           command->distance_mm <= 400U && command->speed_rpm >= 5U &&
           command->speed_rpm <= 60U && command->pulses_per_revolution >= 200U &&
           command->pulses_per_revolution <= 51200U;
}
static inline uint32_t RobotProtocol_LiftDistanceAngle(const RobotLiftDistanceCommand *command)
{
    return (command->distance_mm * 3600U + ROBOT_LIFT_MM_PER_REV / 2U) / ROBOT_LIFT_MM_PER_REV;
}

/* Cylindrical command: theta degrees, r/z signed mm from the startup origin. */
typedef struct {
    uint32_t sequence, theta, speed_rpm, pulses_per_revolution;
    int32_t r_mm, z_mm;
} RobotArmPoseCommand;

static inline bool RobotProtocol_SignedMm(const char **p, int32_t *value)
{
    bool negative = **p == '-';
    if (negative) ++*p;
    uint32_t magnitude;
    if (!RobotProtocol_U32(p, &magnitude, ',') || magnitude > 1000U) return false;
    *value = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    return true;
}
static inline bool RobotProtocol_ParseArmPose(const char *frame, RobotArmPoseCommand *command)
{
    if (strncmp(frame, "ARM_POSE,", 9U) != 0) return false;
    const char *p = frame + 9U;
    if (!RobotProtocol_U32(&p, &command->sequence, ',') ||
        !RobotProtocol_U32(&p, &command->theta, ',') ||
        !RobotProtocol_SignedMm(&p, &command->r_mm) ||
        !RobotProtocol_SignedMm(&p, &command->z_mm) ||
        !RobotProtocol_U32(&p, &command->speed_rpm, ',') ||
        !RobotProtocol_U32(&p, &command->pulses_per_revolution, '\0')) return false;
    return command->sequence != 0U && command->theta <= 270U &&
        command->z_mm >= -400 && command->z_mm <= 400 &&
        command->speed_rpm >= 5U && command->speed_rpm <= 60U &&
        command->pulses_per_revolution >= 200U && command->pulses_per_revolution <= 51200U;
}

#endif
