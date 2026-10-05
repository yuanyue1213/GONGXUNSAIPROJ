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
        ((*p != 'F') && (*p != 'B') && (*p != 'L') && (*p != 'R') && (*p != 'C') && (*p != 'W')))
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
           command->speed_rpm <= 120U && command->pulses_per_revolution >= 200U &&
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
           command->speed_rpm <= 120U && command->pulses_per_revolution >= 200U &&
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
           command->speed_rpm <= 120U && command->pulses_per_revolution >= 200U &&
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
        command->speed_rpm >= 5U && command->speed_rpm <= 120U &&
        command->pulses_per_revolution >= 200U && command->pulses_per_revolution <= 51200U;
}

/* Atomic configurable sequence: r targets in 0.1 mm, z targets in mm. */
typedef struct {
    uint32_t sequence, r_rpm, up_rpm, down_rpm, gripper_dps, theta, open_angle, close_angle, base_home, base_tilt;
    uint32_t plan_index;
    char mode;
    int32_t r1, z1, r2, z2;
} RobotSequenceCommand;
static inline bool RobotProtocol_SequenceCoordinate(const char **p, int32_t *value, uint32_t limit)
{
    bool negative = **p == '-';
    if (negative) ++*p;
    uint32_t magnitude;
    if (!RobotProtocol_U32(p, &magnitude, ',') || magnitude > limit) return false;
    *value = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    return true;
}
static inline bool RobotProtocol_ParseSequence(const char *frame, RobotSequenceCommand *cmd)
{
    bool item = strncmp(frame, "PLAN_ITEM,", 10U) == 0;
    if (!item && strncmp(frame, "STATE,", 6U) != 0) return false;
    const char *p = frame + (item ? 10U : 6U);
    *cmd = (RobotSequenceCommand){0};
    if (!RobotProtocol_U32(&p, &cmd->sequence, ',')) return false;
    if (item && (!RobotProtocol_U32(&p, &cmd->plan_index, ',') || cmd->plan_index >= 16U)) return false;
    if ((*p != 'A' && *p != 'P' && *p != 'T') || p[1] != ',') return false;
    cmd->mode = *p; p += 2;
    /* T card: start/end angles and angular speed, independent of arm coordinates. */
    if (cmd->mode == 'T') {
        return RobotProtocol_U32(&p, &cmd->theta, ',') &&
            RobotProtocol_U32(&p, &cmd->open_angle, ',') &&
            RobotProtocol_U32(&p, &cmd->gripper_dps, '\0') && cmd->sequence != 0U &&
            cmd->theta <= 270U && cmd->open_angle <= 270U &&
            cmd->gripper_dps >= 1U && cmd->gripper_dps <= 360U;
    }
    if (!RobotProtocol_SequenceCoordinate(&p, &cmd->r1, 10000U) ||
        !RobotProtocol_SequenceCoordinate(&p, &cmd->z1, 400U) ||
        !RobotProtocol_SequenceCoordinate(&p, &cmd->r2, 10000U) ||
        !RobotProtocol_SequenceCoordinate(&p, &cmd->z2, 400U) ||
        !RobotProtocol_U32(&p, &cmd->r_rpm, ',') ||
        !RobotProtocol_U32(&p, &cmd->up_rpm, ',')) return false;
    cmd->gripper_dps = 60U; /* Older STATE frames retain the one-second opening. */
    bool has_gripper_speed = strchr(p, ',') != NULL;
    cmd->theta = 0U; cmd->open_angle = 60U; cmd->close_angle = 0U;
    cmd->base_home = 248U; cmd->base_tilt = 140U;
    if (!RobotProtocol_U32(&p, &cmd->down_rpm, has_gripper_speed ? ',' : '\0')) return false;
    if (has_gripper_speed) {
        bool angles = strchr(p, ',') != NULL;
        if (!RobotProtocol_U32(&p, &cmd->gripper_dps, angles ? ',' : '\0')) return false;
        if (angles && (!RobotProtocol_U32(&p, &cmd->theta, ',') ||
            !RobotProtocol_U32(&p, &cmd->open_angle, ',') || !RobotProtocol_U32(&p, &cmd->close_angle, ',') ||
            !RobotProtocol_U32(&p, &cmd->base_home, ',') || !RobotProtocol_U32(&p, &cmd->base_tilt, '\0'))) return false;
    }
    return cmd->sequence != 0U && cmd->r_rpm >= 5U && cmd->r_rpm <= 160U &&
        cmd->up_rpm >= 5U && cmd->up_rpm <= 160U && cmd->down_rpm >= 5U && cmd->down_rpm <= 160U &&
        cmd->gripper_dps >= 6U && cmd->gripper_dps <= 300U && cmd->theta <= 270U &&
        cmd->open_angle <= 270U && cmd->close_angle <= 270U && cmd->base_home <= 360U && cmd->base_tilt <= 360U;
}
typedef struct { uint32_t sequence, start, end, dps; bool stop; } RobotGripperCommand;
typedef struct { uint32_t sequence, target, dps; } RobotBaseCommand;
static inline bool RobotProtocol_ParseBase(const char *frame, RobotBaseCommand *cmd)
{
    if (strncmp(frame, "BASE,", 5U) != 0) return false;
    const char *p = frame + 5U;
    return RobotProtocol_U32(&p, &cmd->sequence, ',') &&
        RobotProtocol_U32(&p, &cmd->target, ',') && RobotProtocol_U32(&p, &cmd->dps, '\0') &&
        cmd->sequence != 0U && cmd->target <= 360U && cmd->dps >= 1U && cmd->dps <= 360U;
}
static inline bool RobotProtocol_ParseGripper(const char *frame, RobotGripperCommand *cmd)
{
    const char *p;
    cmd->stop = strncmp(frame, "GRIP_STOP,", 10U) == 0;
    if (cmd->stop) {
        p = frame + 10U;
        return RobotProtocol_U32(&p, &cmd->sequence, '\0') && cmd->sequence != 0U;
    }
    if (strncmp(frame, "GRIP,", 5U) != 0) return false;
    p = frame + 5U;
    return RobotProtocol_U32(&p, &cmd->sequence, ',') &&
        RobotProtocol_U32(&p, &cmd->start, ',') && RobotProtocol_U32(&p, &cmd->end, ',') &&
        RobotProtocol_U32(&p, &cmd->dps, '\0') && cmd->sequence != 0U &&
        cmd->start <= 270U && cmd->end <= 270U && cmd->dps >= 6U && cmd->dps <= 300U;
}
#define ROBOT_PLAN_MAX_ITEMS 16U
typedef struct { uint32_t sequence, count; bool run; } RobotPlanCommand;
static inline bool RobotProtocol_ParsePlan(const char *frame, RobotPlanCommand *cmd)
{
    cmd->run = strncmp(frame, "PLAN_RUN,", 9U) == 0;
    if (!cmd->run && strncmp(frame, "PLAN_BEGIN,", 11U) != 0) return false;
    const char *p = frame + (cmd->run ? 9U : 11U);
    cmd->count = 0U;
    if (!RobotProtocol_U32(&p, &cmd->sequence, cmd->run ? '\0' : ',')) return false;
    if (!cmd->run && (!RobotProtocol_U32(&p, &cmd->count, '\0') ||
        cmd->count == 0U || cmd->count > ROBOT_PLAN_MAX_ITEMS)) return false;
    return cmd->sequence != 0U;
}
typedef struct { uint32_t sequence, base; } RobotOriginCommand;
static inline bool RobotProtocol_ParseOrigin(const char *frame, RobotOriginCommand *cmd)
{
    if (strncmp(frame, "ORIGIN,", 7U) != 0) return false;
    const char *p = frame + 7U;
    return RobotProtocol_U32(&p, &cmd->sequence, ',') &&
        RobotProtocol_U32(&p, &cmd->base, '\0') && cmd->sequence != 0U && cmd->base <= 360U;
}
#endif
