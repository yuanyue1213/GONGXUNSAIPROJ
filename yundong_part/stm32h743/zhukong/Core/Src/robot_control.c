/**
 ******************************************************************************
 * @file    robot_control.c
 * @brief   ESP32 -> STM32 遥控协议处理。
 ******************************************************************************
 */

#include "robot_control.h"
#include "zdt_motor.h"
#include "lift_motor.h"
#include "servo_control.h"
#include "camera_link.h"
#include "../../../../shared/robot_distance_protocol.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Track the last successfully commanded base angle across origin, automatic
 * states and manual controls. This is a PWM target, not encoder feedback. */
static bool s_base_angle_known, s_manual_base_active;
static uint16_t s_base_angle;
static uint32_t s_manual_base_sequence, s_manual_base_tick, s_manual_base_update, s_manual_base_ms;
static int32_t s_manual_base_start, s_manual_base_target;
static bool RobotControl_SetServoAngle(char channel, uint16_t angle)
{
    if (!ServoControl_SetAngle(channel, angle)) return false;
    if (channel == 'B') { s_base_angle = angle; s_base_angle_known = true; }
    return true;
}
static void RobotControl_TickManualBase(void)
{
    if (!s_manual_base_active) return;
    uint32_t now = HAL_GetTick(), elapsed = (uint32_t)(now - s_manual_base_tick);
    if (elapsed < s_manual_base_ms && (uint32_t)(now - s_manual_base_update) < 20U) return;
    uint32_t progress = elapsed < s_manual_base_ms ? elapsed : s_manual_base_ms;
    int32_t delta = s_manual_base_target - s_manual_base_start;
    uint16_t angle = (uint16_t)(s_manual_base_start + delta * (int32_t)progress / (int32_t)s_manual_base_ms);
    if (!RobotControl_SetServoAngle('B', angle) || elapsed >= s_manual_base_ms) s_manual_base_active = false;
    s_manual_base_update = now;
}

/* App 文本命令经 ESP32 转发到 USART3；一行最多 127 字节，不含换行。
 * 300 ms 续期保护仅用于机械臂升降；底盘定距已取消 KEEP，保留 30 分钟总时限。 */
#define ROBOT_COMMAND_MAX_LENGTH       127U
#define ROBOT_COMMAND_TIMEOUT_MS       300U
#define ROBOT_ACCELERATION             10U
#define ROBOT_MOVE_MAX_TIME_MS          1800000U
#define ROBOT_STATUS_INTERVAL_MS        50U
#define ROBOT_RX_BUFFER_SIZE            512U

/* 底盘任务状态与电机驱动分离：s_motion 保存请求方向，s_reached_mask
 * 的四个位记录最近一轮状态查询结果，s_stop_pending 表示停车尚需重试。 */
static UART_HandleTypeDef *s_command_uart;
static bool s_wheel_uart_ready;
static char s_frame[ROBOT_COMMAND_MAX_LENGTH + 1U];
static uint16_t s_frame_length;
static bool s_dropping_frame;
static char s_motion = 'S';
static uint32_t s_last_move_sequence;
static uint32_t s_move_started_tick;
static uint32_t s_status_tick;
static uint8_t s_status_wheel;
static uint8_t s_reached_mask;
static bool s_stop_pending;
static bool s_wheel_move_reached;
/* USART3 中断逐字节写入环形队列，主循环取出后组帧。
 * ISR 不解析命令、不等待电机 ACK，避免长时间占用中断。 */
static uint8_t s_rx_byte;
static volatile uint8_t s_rx_buffer[ROBOT_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile bool s_rx_fault;
static volatile bool s_rx_restart_pending;
static char s_lift_motion = 'H';
static bool s_lift_command_ok = true;
static uint32_t s_last_lift_tick;
static char s_fore_aft_motion = 'Q';
static bool s_fore_aft_command_ok = true;

#define ROBOT_ARM_MOVE_MAX_MS     180000U
#define ROBOT_ARM_STATUS_LOSS_MS     500U
static bool s_arm_move_active;
static bool s_arm_move_reached;
static bool s_arm_move_stop_pending;
static uint32_t s_last_arm_move_sequence;
static uint32_t s_arm_move_start_tick;
static uint32_t s_arm_move_status_tick;
static uint32_t s_arm_move_last_status_tick;

static void RobotControl_StopArmMove(void)
{
    s_arm_move_reached = false;
    s_arm_move_stop_pending = ArmMotor_StopForeAft() != HAL_OK;
    s_arm_move_active = s_arm_move_stop_pending;
    s_fore_aft_motion = s_arm_move_stop_pending ? 'P' : 'Q';
    s_fore_aft_command_ok = !s_arm_move_stop_pending;
}

static void RobotControl_TickArmMove(void)
{
    if (!s_arm_move_active) return;
    if (s_arm_move_stop_pending) { RobotControl_StopArmMove(); return; }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_arm_move_start_tick) >= ROBOT_ARM_MOVE_MAX_MS)
    { RobotControl_StopArmMove(); return; }
    if ((uint32_t)(now - s_arm_move_status_tick) < 50U) return;
    s_arm_move_status_tick = now;
    uint8_t flags;
    if (!LiftMotor_ReadStatus(2U, &flags)) {
        /* Tolerate a missed status reply without resending FD or guessing arrival. */
        if ((uint32_t)(HAL_GetTick() - s_arm_move_last_status_tick) >= ROBOT_ARM_STATUS_LOSS_MS)
            RobotControl_StopArmMove();
        return;
    }
    s_arm_move_last_status_tick = HAL_GetTick();
    if ((flags & 0x01U) == 0U || (flags & 0x0CU) != 0U)
    { RobotControl_StopArmMove(); return; }
    if ((flags & 0x02U) != 0U) {
        s_arm_move_reached = true;
        s_arm_move_active = false;
        s_fore_aft_motion = 'Q';
        s_fore_aft_command_ok = true;
    }
}

/* 手动定距与抓取插入运动共用到位/故障处理，不生成模拟 App 序号。 */
static bool RobotControl_StartArmMove(char direction, uint32_t distance_mm,
                                     uint16_t rpm, uint32_t pulses_per_rev)
{
    if (s_arm_move_active || s_fore_aft_motion != 'Q' || !s_fore_aft_command_ok) return false;
    RobotArmDistanceCommand command = {.distance_mm = distance_mm};
    s_arm_move_reached = false;
    if (ArmMotor_MoveAngle(direction, RobotProtocol_ArmDistanceAngle(&command),
                           rpm, pulses_per_rev) != HAL_OK)
    { RobotControl_StopArmMove(); return false; }
    s_arm_move_active = true;
    s_arm_move_stop_pending = false;
    s_fore_aft_motion = 'P';
    s_fore_aft_command_ok = true;
    s_arm_move_start_tick = HAL_GetTick();
    s_arm_move_status_tick = s_arm_move_start_tick;
    s_arm_move_last_status_tick = s_arm_move_start_tick;
    return true;
}

/* ID1 calibration runs independently of the hold-to-run lift watchdog. */
static bool s_lift_angle_active, s_lift_angle_stop_pending;
static bool s_lift_angle_reached;
static uint32_t s_last_lift_angle_sequence;
static uint32_t s_lift_angle_start_tick, s_lift_angle_status_tick, s_lift_angle_last_status_tick;

static void RobotControl_StopLiftAngle(void)
{
    s_lift_angle_reached = false;
    s_lift_angle_stop_pending = LiftMotor_Stop() != HAL_OK;
    s_lift_angle_active = s_lift_angle_stop_pending;
    s_lift_motion = 'H';
    s_lift_command_ok = !s_lift_angle_stop_pending;
}

static void RobotControl_TickLiftAngle(void)
{
    if (!s_lift_angle_active) return;
    if (s_lift_angle_stop_pending) { RobotControl_StopLiftAngle(); return; }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_lift_angle_start_tick) >= ROBOT_ARM_MOVE_MAX_MS)
    { RobotControl_StopLiftAngle(); return; }
    if ((uint32_t)(now - s_lift_angle_status_tick) < ROBOT_STATUS_INTERVAL_MS) return;
    s_lift_angle_status_tick = now;
    uint8_t flags;
    if (!LiftMotor_ReadStatus(1U, &flags)) {
        if ((uint32_t)(HAL_GetTick() - s_lift_angle_last_status_tick) >= ROBOT_ARM_STATUS_LOSS_MS)
            RobotControl_StopLiftAngle();
        return;
    }
    s_lift_angle_last_status_tick = HAL_GetTick();
    if ((flags & 0x01U) == 0U || (flags & 0x0CU) != 0U)
    { RobotControl_StopLiftAngle(); return; }
    if ((flags & 0x02U) != 0U) {
        s_lift_angle_reached = true;
        s_lift_angle_active = false;
        s_lift_motion = 'H';
        s_lift_command_ok = true;
    }
}

/* Boot position is a software origin, not a limit-switch homing operation.
 * Capture each real motor position once; never re-zero after a stop or reconnect. */
static int64_t s_arm_origin, s_lift_origin;
static bool s_arm_origin_valid, s_lift_origin_valid;
static uint32_t s_origin_retry_tick;
static bool s_origin_initializing;
static uint32_t s_last_origin_sequence;
static uint16_t s_origin_base = 248U;
static bool s_pose_active, s_pose_home;
static uint8_t s_pose_phase;
static uint32_t s_last_pose_sequence;
static RobotArmPoseCommand s_pose;
static uint16_t s_grab_turntable;

static void RobotControl_CaptureOrigin(void)
{
    int64_t arm, lift;
    /* Commit both zeros together; a partial read never enables axis movement. */
    if (ArmMotor_ReadPosition(2U, &arm) && ArmMotor_ReadPosition(1U, &lift)) {
        s_arm_origin = arm; s_lift_origin = lift;
        s_arm_origin_valid = s_lift_origin_valid = true;
        s_origin_initializing = false;
    }
    s_origin_retry_tick = HAL_GetTick();
}
static int64_t RobotControl_MmAngle(int32_t mm, uint32_t mm_per_rev)
{
    int64_t magnitude = ((int64_t)(mm < 0 ? -mm : mm) * 3600 + mm_per_rev / 2U) / mm_per_rev;
    return mm < 0 ? -magnitude : magnitude;
}
/* Sequence r positions use 0.1 mm units to retain fractional millimetres. */
static bool RobotControl_StartArmAbsoluteTenths(int32_t r_tenths_mm, uint16_t rpm, uint32_t pulses)
{
    if (!s_arm_origin_valid || s_arm_move_active || s_fore_aft_motion != 'Q' || !s_fore_aft_command_ok) return false;
    s_arm_move_reached = false;
    int64_t r = r_tenths_mm;
    int64_t angle = ((r < 0 ? -r : r) * 360 + 63) / 127;
    if (r < 0) angle = -angle;
    if (ArmMotor_MoveAbsolute(2U, s_arm_origin + angle, rpm, pulses) != HAL_OK)
    { RobotControl_StopArmMove(); return false; }
    s_arm_move_active = true; s_arm_move_stop_pending = false;
    s_fore_aft_motion = 'P'; s_fore_aft_command_ok = true;
    s_arm_move_start_tick = s_arm_move_status_tick = s_arm_move_last_status_tick = HAL_GetTick();
    return true;
}
static bool RobotControl_StartArmAbsolute(int32_t r_mm, uint16_t rpm, uint32_t pulses)
{
    return RobotControl_StartArmAbsoluteTenths(r_mm * 10, rpm, pulses);
}
static bool RobotControl_StartLiftAbsolute(int32_t z_mm, uint16_t rpm, uint32_t pulses)
{
    if (!s_lift_origin_valid || s_lift_angle_active || s_lift_motion != 'H' || !s_lift_command_ok) return false;
    s_lift_angle_reached = false;
    if (ArmMotor_MoveAbsolute(1U, s_lift_origin - RobotControl_MmAngle(z_mm, ROBOT_LIFT_MM_PER_REV), rpm, pulses) != HAL_OK)
    { RobotControl_StopLiftAngle(); return false; }
    s_lift_angle_active = true; s_lift_angle_stop_pending = false;
    s_lift_motion = 'P'; s_lift_command_ok = true;
    s_lift_angle_start_tick = s_lift_angle_status_tick = s_lift_angle_last_status_tick = HAL_GetTick();
    return true;
}
static void RobotControl_CancelPose(void)
{
    if (!s_pose_active) return;
    s_pose_active = false;
    if (s_arm_move_active) RobotControl_StopArmMove();
    if (s_lift_angle_active) RobotControl_StopLiftAngle();
}
/* Sequential r -> z -> theta; retain G/B unless explicitly returning home. */
static void RobotControl_TickPose(void)
{
    if (!s_pose_active) return;
    if (s_pose_phase == 0U) {
        if (s_arm_move_stop_pending) { RobotControl_CancelPose(); return; }
        if (s_arm_move_active) return;
        if (!s_arm_move_reached || !RobotControl_StartLiftAbsolute(s_pose.z_mm,
            (uint16_t)s_pose.speed_rpm, s_pose.pulses_per_revolution))
        { RobotControl_CancelPose(); return; }
        s_pose_phase = 1U;
    } else {
        if (s_lift_angle_stop_pending) { RobotControl_CancelPose(); return; }
        if (s_lift_angle_active) return;
        if (!s_lift_angle_reached) { RobotControl_CancelPose(); return; }
        (void)RobotControl_SetServoAngle('T', (uint16_t)s_pose.theta);
        if (s_pose_home) {
            (void)RobotControl_SetServoAngle('G', 15U);
            (void)RobotControl_SetServoAngle('B', s_origin_base);
            s_grab_turntable = 0U;
        }
        s_pose_active = false;
    }
}

/* Eight grab states. r/z are absolute offsets from the startup origin.
 * A step is held for one second only after both positioning axes reach it. */
typedef struct {
    uint16_t gripper, base;
    int32_t r_tenths_mm, z_mm;
    bool position;
} GrabStep;
static const GrabStep s_grab_steps[] = {
    {60U, 248U,   0,   0, true},
    {60U, 248U, 1100, -50, true},
    { 0U, 248U,   0,   0, false},
    { 0U, 248U,   0,   0, true},
    { 0U, 140U,   0,   0, false},
    { 0U, 140U, 200, -40, true},
    {60U, 140U,   0,   0, true},
    {60U, 248U,   0,   0, false},
};
static const GrabStep s_release_steps[] = {
    {60U, 248U,   0,   0, true},
    {60U, 140U,   0,   0, false},
    { 0U, 140U, 200, -40, true},
    { 0U, 140U,   0,   0, true},
    { 0U, 248U,   0,   0, false},
    { 0U, 248U, 1300, -110, true},
    {60U, 248U,   0,   0, false},
    {60U, 248U,   0,   0, true},
};
#define ROBOT_GRAB_ARM_RPM        20U
#define ROBOT_SEQUENCE_DOWN_RPM   50U
static GrabStep s_sequence_steps[8];
static uint16_t s_sequence_r_rpm, s_sequence_up_rpm, s_sequence_down_rpm;
#define ROBOT_GRAB_ARM_PULSES     3200U
#define ROBOT_GRAB_HOLD_MS       1000U
#define ROBOT_BASE_ROTATE_MS     2000U
#define ROBOT_BASE_UPDATE_MS       20U
#define ROBOT_GRIPPER_OPEN_MS    1000U
static uint32_t s_gripper_open_ms, s_sequence_gripper_dps;
static uint16_t s_sequence_theta;
static int32_t s_sequence_gripper_start, s_sequence_gripper_end;
static bool s_grab_active;
static bool s_release_mode;
static uint32_t s_last_release_sequence;
static bool s_grab_arm_wait, s_grab_lift_wait;
static bool s_grab_base_rotating;
static bool s_grab_gripper_opening;
static uint32_t s_gripper_update_tick;
static uint8_t s_grab_step;
static uint32_t s_grab_tick;
static uint32_t s_base_update_tick;
static uint32_t s_last_grab_sequence;
/* App uploads an entire plan before RUN. No reply or estimated sleep is needed:
 * only a successfully completed card may start its successor. */
static RobotSequenceCommand s_plan_items[ROBOT_PLAN_MAX_ITEMS];
static uint32_t s_plan_sequence, s_last_plan_sequence, s_plan_mask;
static uint8_t s_plan_count, s_plan_index;
static bool s_plan_active;
static bool RobotControl_StartSequence(const RobotSequenceCommand *settings, char direction);
static void RobotControl_CancelGrab(void);
static const GrabStep *RobotControl_SequenceSteps(void)
{
    return s_sequence_steps;
}
static bool RobotControl_SequenceZFirst(void)
{
    return s_release_mode ? (s_grab_step == 2U || s_grab_step == 3U) : s_grab_step == 1U;
}
static uint16_t RobotControl_SequenceLiftRpm(void)
{
    /* Compare consecutive absolute Z positions, including intervening holds.
     * Negative Z is downward; only descending sequence moves run faster. */
    const GrabStep *steps = RobotControl_SequenceSteps();
    int32_t previous_z = 0;
    for (unsigned i = 0; i < s_grab_step; ++i) {
        if (steps[i].position) previous_z = steps[i].z_mm;
    }
    return steps[s_grab_step].z_mm < previous_z ? s_sequence_down_rpm : s_sequence_up_rpm;
}
static bool RobotControl_FinishSequencePosition(void)
{
    /* Release step 3 closes only after BOTH axes reach the pickup location. */
    if (s_release_mode && s_grab_step == 2U && !RobotControl_SetServoAngle('G', s_sequence_steps[2].gripper)) {
        RobotControl_CancelGrab(); return false;
    }
    s_grab_tick = HAL_GetTick();
    return true;
}
/* Completed rotation target only advances after the final step succeeds. */

static void RobotControl_CancelGrab(void)
{
    s_plan_active = false; s_plan_count = 0U; s_plan_mask = 0U;
    s_grab_active = false;
    if (s_grab_arm_wait && s_arm_move_active) RobotControl_StopArmMove();
    if (s_grab_lift_wait && s_lift_angle_active) RobotControl_StopLiftAngle();
    s_grab_arm_wait = s_grab_lift_wait = false;
    s_grab_base_rotating = false;
    s_grab_gripper_opening = false;
}

static bool RobotControl_StartSequencePosition(void)
{
    const GrabStep *step = &RobotControl_SequenceSteps()[s_grab_step];
    if (!step->position) return true;
    if (RobotControl_SequenceZFirst()) {
        s_grab_lift_wait = RobotControl_StartLiftAbsolute(step->z_mm,
            RobotControl_SequenceLiftRpm(), ROBOT_GRAB_ARM_PULSES);
        return s_grab_lift_wait;
    }
    s_grab_arm_wait = RobotControl_StartArmAbsoluteTenths(step->r_tenths_mm,
        s_sequence_r_rpm, ROBOT_GRAB_ARM_PULSES);
    return s_grab_arm_wait;
}
static bool s_manual_gripper_active;
static uint32_t s_manual_gripper_sequence, s_manual_gripper_tick, s_manual_gripper_update, s_manual_gripper_ms;
static int32_t s_manual_gripper_start, s_manual_gripper_end;
static void RobotControl_TickManualGripper(void)
{
    if (!s_manual_gripper_active) return;
    uint32_t now = HAL_GetTick(), elapsed = (uint32_t)(now - s_manual_gripper_tick);
    if (elapsed < s_manual_gripper_ms && (uint32_t)(now - s_manual_gripper_update) < 20U) return;
    uint32_t progress = elapsed < s_manual_gripper_ms ? elapsed : s_manual_gripper_ms;
    int32_t delta = s_manual_gripper_end - s_manual_gripper_start;
    int32_t angle = s_manual_gripper_start + delta * (int32_t)progress / (int32_t)s_manual_gripper_ms;
    if (!RobotControl_SetServoAngle('G', (uint16_t)angle) || elapsed >= s_manual_gripper_ms)
        s_manual_gripper_active = false;
    s_manual_gripper_update = now;
}

static bool RobotControl_ApplyGrabStep(void)
{
    const GrabStep *steps = RobotControl_SequenceSteps();
    const GrabStep *step = &steps[s_grab_step];
    /* Two-second base ramps on steps 5 and 8. */
    s_grab_base_rotating = s_release_mode ? (s_grab_step == 1U || s_grab_step == 4U) :
        (s_grab_step == 4U || s_grab_step == 7U);
    uint16_t base = s_grab_base_rotating ? steps[s_grab_step-1U].base : step->base;
    uint16_t gripper = s_release_mode && s_grab_step == 2U ? steps[1].gripper : step->gripper;
    s_grab_gripper_opening = s_grab_step == 6U && steps[5].gripper != gripper;
    if (s_grab_gripper_opening) {
        s_sequence_gripper_start = steps[5].gripper; s_sequence_gripper_end = gripper;
        uint32_t distance = (uint32_t)(s_sequence_gripper_end > s_sequence_gripper_start ?
            s_sequence_gripper_end - s_sequence_gripper_start : s_sequence_gripper_start - s_sequence_gripper_end);
        s_gripper_open_ms = (distance * 1000U + s_sequence_gripper_dps - 1U) / s_sequence_gripper_dps;
        gripper = (uint16_t)s_sequence_gripper_start;
    }
    s_grab_tick = s_base_update_tick = s_gripper_update_tick = HAL_GetTick();
    if (!RobotControl_SetServoAngle('T', s_sequence_theta) || !RobotControl_SetServoAngle('G', gripper) ||
        !RobotControl_SetServoAngle('B', base)) return false;
    /* Finish releasing before moving away from the object. */
    return s_grab_gripper_opening || RobotControl_StartSequencePosition();
}

static void RobotControl_TickGrab(void)
{
    if (!s_grab_active) return;
    if (s_grab_step >= 8U) {
        /* Keep ownership during the inter-card hold, including the grab's final
         * turntable command; a new card must not immediately overwrite it. */
        if ((uint32_t)(HAL_GetTick() - s_grab_tick) < ROBOT_GRAB_HOLD_MS) return;
        s_grab_active = false;
        const RobotSequenceCommand *next = &s_plan_items[s_plan_index];
        if (!RobotControl_StartSequence(next, next->mode)) RobotControl_CancelGrab();
        return;
    }
    if (s_grab_gripper_opening) {
        uint32_t now = HAL_GetTick(), elapsed = (uint32_t)(now - s_grab_tick);
        if (elapsed < s_gripper_open_ms &&
            (uint32_t)(now - s_gripper_update_tick) < ROBOT_BASE_UPDATE_MS) return;
        uint32_t progress = elapsed < s_gripper_open_ms ? elapsed : s_gripper_open_ms;
        int32_t delta = s_sequence_gripper_end - s_sequence_gripper_start;
        uint16_t angle = (uint16_t)(s_sequence_gripper_start + delta * (int32_t)progress / (int32_t)s_gripper_open_ms);
        if (!RobotControl_SetServoAngle('G', angle)) { RobotControl_CancelGrab(); return; }
        s_gripper_update_tick = now;
        if (elapsed >= s_gripper_open_ms) {
            s_grab_gripper_opening = false; s_grab_tick = now;
            if (!RobotControl_StartSequencePosition()) RobotControl_CancelGrab();
        }
        return;
    }
    if (s_grab_arm_wait) {
        if (s_arm_move_stop_pending) { RobotControl_CancelGrab(); return; }
        if (s_arm_move_active) return;
        s_grab_arm_wait = false;
        if (!s_arm_move_reached) { RobotControl_CancelGrab(); return; }
        if (RobotControl_SequenceZFirst()) { (void)RobotControl_FinishSequencePosition(); return; }
        s_grab_lift_wait = RobotControl_StartLiftAbsolute(RobotControl_SequenceSteps()[s_grab_step].z_mm,
            RobotControl_SequenceLiftRpm(), ROBOT_GRAB_ARM_PULSES);
        if (!s_grab_lift_wait) RobotControl_CancelGrab();
        return;
    }
    if (s_grab_lift_wait) {
        if (s_lift_angle_stop_pending) { RobotControl_CancelGrab(); return; }
        if (s_lift_angle_active) return;
        s_grab_lift_wait = false;
        if (!s_lift_angle_reached) { RobotControl_CancelGrab(); return; }
        if (RobotControl_SequenceZFirst()) {
            s_grab_arm_wait = RobotControl_StartArmAbsoluteTenths(RobotControl_SequenceSteps()[s_grab_step].r_tenths_mm,
                s_sequence_r_rpm, ROBOT_GRAB_ARM_PULSES);
            if (!s_grab_arm_wait) RobotControl_CancelGrab();
            return;
        }
        (void)RobotControl_FinishSequencePosition();
        return;
    }
    uint32_t now = HAL_GetTick(), elapsed = (uint32_t)(now - s_grab_tick);
    if (s_grab_base_rotating) {
        if (elapsed < ROBOT_BASE_ROTATE_MS && (uint32_t)(now - s_base_update_tick) < ROBOT_BASE_UPDATE_MS) return;
        uint32_t progress = elapsed < ROBOT_BASE_ROTATE_MS ? elapsed : ROBOT_BASE_ROTATE_MS;
        const GrabStep *steps = RobotControl_SequenceSteps();
        int32_t start = steps[s_grab_step-1U].base;
        int32_t delta = (int32_t)steps[s_grab_step].base - start;
        int32_t numerator = delta * (int32_t)progress;
        numerator += delta < 0 ? -(int32_t)(ROBOT_BASE_ROTATE_MS/2U) : (int32_t)(ROBOT_BASE_ROTATE_MS/2U);
        if (!RobotControl_SetServoAngle('B', (uint16_t)(start + numerator/(int32_t)ROBOT_BASE_ROTATE_MS)))
        { RobotControl_CancelGrab(); return; }
        s_base_update_tick = now;
        if (elapsed >= ROBOT_BASE_ROTATE_MS) { s_grab_base_rotating = false; s_grab_tick = now; }
        return;
    }
    if (elapsed < ROBOT_GRAB_HOLD_MS) return;
    if (++s_grab_step >= sizeof(s_grab_steps)/sizeof(s_grab_steps[0])) {
        if (!s_release_mode) {
            uint16_t target = (s_grab_turntable + 120U) % 360U;
            if (!RobotControl_SetServoAngle('T', target)) { RobotControl_CancelGrab(); return; }
            s_grab_turntable = target;
        }
        if (s_plan_active && ++s_plan_index < s_plan_count) {
            s_grab_tick = HAL_GetTick();
        } else {
            s_grab_active = false;
            s_plan_active = false; s_plan_count = 0U; s_plan_mask = 0U;
        }
    } else if (!RobotControl_ApplyGrabStep()) RobotControl_CancelGrab();
}

/* 摄像头持续输出坐标。每次移动等待四轮到位、静置 300 ms 后重新定位。 */
#define ROBOT_ALIGN_SAMPLE_MS       3000U
#define ROBOT_ALIGN_TOTAL_MS      120000U
#define ROBOT_ALIGN_SETTLE_MS        300U
#define ROBOT_ALIGN_MAX_MOVES         60U
typedef enum { ALIGN_WAIT, ALIGN_MOVING, ALIGN_SETTLE, ALIGN_COMPENSATING } AlignPhase;
static bool s_align_active, s_align_parallel;
static CameraParallelCommand s_parallel;
static CameraAlignSettings s_align_settings;
static AlignPhase s_align_phase;
static uint32_t s_align_start_tick, s_align_phase_tick, s_last_align_sequence;
static uint32_t s_align_forward_ppm, s_align_lateral_ppm;
static unsigned s_align_moves, s_align_centered;
static HAL_StatusTypeDef RobotControl_ApplyDistance(const RobotDistanceCommand *command);
static HAL_StatusTypeDef RobotControl_ApplyWheelPulses(char direction, int32_t pulses, uint16_t rpm);
static HAL_StatusTypeDef RobotControl_StopMotors(void);

static void RobotControl_CancelAlignment(void)
{
    if (s_align_active && (s_align_phase == ALIGN_MOVING || s_align_phase == ALIGN_COMPENSATING) &&
        s_motion != 'S' && !s_stop_pending) {
        s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        if (!s_stop_pending) s_motion = 'S';
    }
    s_align_active = false;
    CameraLink_Discard();
}
static void RobotControl_TickAlignment(void)
{
    if (!s_align_active) return;
    uint32_t now = HAL_GetTick();
    if (s_stop_pending || (uint32_t)(now - s_align_start_tick) >= ROBOT_ALIGN_TOTAL_MS)
    { RobotControl_CancelAlignment(); return; }
    if (s_align_phase == ALIGN_MOVING || s_align_phase == ALIGN_COMPENSATING) {
        CameraCenter ignored; (void)CameraLink_TakeCenter(&ignored);
        if (s_motion == 'S') {
            if (!s_wheel_move_reached) { RobotControl_CancelAlignment(); return; }
            if (s_align_phase == ALIGN_COMPENSATING) { RobotControl_CancelAlignment(); return; }
            s_align_phase = ALIGN_SETTLE; s_align_phase_tick = now;
        }
        return;
    }
    if (s_align_phase == ALIGN_SETTLE) {
        if ((uint32_t)(now - s_align_phase_tick) >= ROBOT_ALIGN_SETTLE_MS) {
            CameraLink_Discard(); s_align_phase = ALIGN_WAIT; s_align_phase_tick = now;
        }
        return;
    }
    if ((uint32_t)(now - s_align_phase_tick) >= ROBOT_ALIGN_SAMPLE_MS)
    { RobotControl_CancelAlignment(); return; }
    CameraCenter center = {256U,160U};
    char direction; uint32_t mm; uint16_t rpm;
    bool compensating = false;
    uint32_t fine_turn_pulses = 0U;
    if (s_align_parallel) {
        CameraCenter rings[3]; int32_t slope;
        if (!CameraLink_TakeRings(rings) || !CameraProtocol_RingSlope(rings, &slope)) return;
        s_align_phase_tick = now;
        uint32_t error = (uint32_t)(slope < 0 ? -slope : slope);
        if (CameraProtocol_RingYSpread(rings) <= 1U) {
            if (++s_align_centered >= 3U) RobotControl_CancelAlignment();
            return;
        }
        /* A zero fitted slope with unequal heights indicates center noise or
         * a bent/non-horizontal detection. Do not guess a turning direction. */
        if (slope == 0) { s_align_centered = 0U; return; }
        direction = (slope > 0) != (s_parallel.reverse != 0U) ? 'W' : 'C';
        mm = error / 25U + 1U;
        if (mm > s_parallel.step_mm) mm = s_parallel.step_mm;
        rpm = (uint16_t)s_parallel.rpm;
        if (error <= 50U) {
            /* Near horizontal, use 0.2 mm of wheel travel rather than rounding
             * up to a full millimetre. Preserve the actual calibrated ppm. */
            fine_turn_pulses = (s_parallel.ppm * 2U + 5000U) / 10000U;
            if (fine_turn_pulses == 0U) fine_turn_pulses = 1U;
            if (rpm > 5U) rpm = 5U;
        }
    } else {
        if (!CameraLink_TakeCenter(&center)) return;
        s_align_phase_tick = now;
        rpm = CameraProtocol_ConfiguredRpm(&center, &s_align_settings);
        if (!CameraProtocol_Correction(&center, &direction, &mm)) {
        if (++s_align_centered < 2U) return;
        /* Both camera modes apply the configured final X offset once.
         * Do not re-center afterward, which would undo this offset. */
        if (s_align_settings.offset_mm == 0) { RobotControl_CancelAlignment(); return; }
        direction = s_align_settings.offset_mm > 0 ? CAMERA_IMAGE_RIGHT_DIRECTION :
            CameraProtocol_Opposite(CAMERA_IMAGE_RIGHT_DIRECTION);
        mm = (uint32_t)(s_align_settings.offset_mm > 0 ? s_align_settings.offset_mm : -s_align_settings.offset_mm);
        compensating = true;
        rpm = (uint16_t)s_align_settings.fine_rpm;
        }
    }
    s_align_centered = 0U;
    if (!compensating && s_align_moves >= ROBOT_ALIGN_MAX_MOVES)
    { RobotControl_CancelAlignment(); return; }
    RobotDistanceCommand move = {0U, direction, mm,
        rpm,
        (direction == 'F' || direction == 'B') ? s_align_forward_ppm : s_align_lateral_ppm};
    s_wheel_move_reached = false;
    HAL_StatusTypeDef result = fine_turn_pulses != 0U ?
        RobotControl_ApplyWheelPulses(direction, (int32_t)fine_turn_pulses, rpm) : RobotControl_ApplyDistance(&move);
    if (result != HAL_OK) {
        s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        if (!s_stop_pending) s_motion = 'S';
        s_align_active = false; return;
    }
    s_motion = direction; s_move_started_tick = HAL_GetTick();
    s_status_tick = s_move_started_tick; s_status_wheel = 0U; s_reached_mask = 0U;
    if (!compensating) ++s_align_moves;
    s_align_phase = compensating ? ALIGN_COMPENSATING : ALIGN_MOVING;
    s_align_phase_tick = s_move_started_tick;
}

static bool RobotControl_Parse(const char *frame, uint32_t *sequence,
                               char *direction);
static bool RobotControl_ParseServo(const char *frame, uint32_t *sequence,
                                    char *channel, uint16_t *angle);
static HAL_StatusTypeDef RobotControl_EnableMotors(void);
static HAL_StatusTypeDef RobotControl_StopMotors(void);
static HAL_StatusTypeDef RobotControl_ApplyDistance(const RobotDistanceCommand *command);
static void RobotControl_HandleFrame(const char *frame);
static void RobotControl_ArmReceive(void);

static void RobotControl_ArmReceive(void)
{
    HAL_StatusTypeDef status;
    if (s_command_uart == NULL) return;
    status = HAL_UART_Receive_IT(s_command_uart, &s_rx_byte, 1U);
    /* BUSY 表示已有接收在进行；ERROR/TIMEOUT 标记为主循环待重试。 */
    s_rx_restart_pending = (status != HAL_OK) && (status != HAL_BUSY);
}

/* 上电只开启接收；舵机姿态和两轴零点由 App 的 I 命令初始化。 */
void RobotControl_Init(UART_HandleTypeDef *command_uart, bool wheel_uart_ready)
{
    s_command_uart = command_uart;
    s_wheel_uart_ready = wheel_uart_ready;
    s_frame_length = 0U;
    s_dropping_frame = false;
    s_motion = 'S';
    s_last_move_sequence = 0U;
    s_wheel_move_reached = false;
    s_align_active = false;
    s_last_align_sequence = 0U;
    s_arm_move_active = false;
    s_arm_move_reached = false;
    s_arm_move_stop_pending = false;
    s_last_arm_move_sequence = 0U;
    s_arm_move_last_status_tick = 0U;
    s_lift_angle_active = s_lift_angle_stop_pending = false;
    s_lift_angle_reached = false;
    s_last_lift_angle_sequence = 0U;
    s_grab_active = false;
    s_manual_gripper_active = false; s_manual_gripper_sequence = 0U;
    s_manual_base_active = false; s_manual_base_sequence = 0U; s_base_angle_known = false;
    s_release_mode = false; s_last_release_sequence = 0U;
    s_grab_arm_wait = s_grab_lift_wait = false;
    s_grab_turntable = 0U;
    s_grab_base_rotating = false;
    s_grab_step = 0U;
    s_last_grab_sequence = 0U;
    s_stop_pending = false;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_fault = false;
    s_rx_restart_pending = false;
    /* 主循环等待电机 ACK 时，中断仍可缓存 App 的 STOP 等命令。 */
    if (s_command_uart != NULL)
    {
        HAL_NVIC_SetPriority(USART3_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        RobotControl_ArmReceive();
    }
    s_lift_motion = 'H';
    s_lift_command_ok = true;
    s_last_lift_tick = HAL_GetTick();
    s_fore_aft_motion = 'Q';
    s_fore_aft_command_ok = true;
    s_pose_active = false; s_last_pose_sequence = 0U;
    s_arm_origin_valid = s_lift_origin_valid = false;
    s_origin_initializing = false; s_last_origin_sequence = 0U; s_origin_base = 248U;
    s_plan_active = false; s_plan_count = 0U; s_plan_mask = 0U;
    s_plan_sequence = s_last_plan_sequence = 0U;

}

/* 在主循环执行：恢复接收、读取环形队列，遇到 LF/CRLF 后分发完整命令。
 * 队列溢出或串口错误后丢弃受损内容，并等待下一换行重新对齐帧边界。 */
void RobotControl_Process(void)
{
    CameraLink_Process();
    uint8_t byte;

    if (s_command_uart == NULL)
    {
        return;
    }

    /* 接收启动失败或 HAL 已结束接收时重新挂接，避免永久收不到命令。 */
    if (s_rx_restart_pending || (s_command_uart->RxState == HAL_UART_STATE_READY))
        RobotControl_ArmReceive();

    while (s_rx_head != s_rx_tail || s_rx_fault)
    {
        if (s_rx_fault)
        {
            uint32_t interrupt_state = __get_PRIMASK();
            __disable_irq();
            s_rx_tail = s_rx_head;
            s_rx_fault = false;
            __set_PRIMASK(interrupt_state);
            s_frame_length = 0U;
            s_dropping_frame = true;
            break;
        }
        byte = s_rx_buffer[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % ROBOT_RX_BUFFER_SIZE);
        if (byte == '\n')
        {
            if (!s_dropping_frame)
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

}

/* HAL 收满一个字节后调用；只处理命令串口，入队后立即挂接下一字节。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (CameraLink_RxCallback(huart)) return;
    if (huart != s_command_uart) return;
    uint16_t next = (uint16_t)((s_rx_head + 1U) % ROBOT_RX_BUFFER_SIZE);
    if (next == s_rx_tail) s_rx_fault = true;
    else
    {
        s_rx_buffer[s_rx_head] = s_rx_byte;
        s_rx_head = next;
    }
    RobotControl_ArmReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (CameraLink_ErrorCallback(huart)) return;
    if (huart != s_command_uart) return;
    s_rx_fault = true;
    /* ORE 会中止 HAL 中断接收；清除溢出、噪声、帧错误后尝试恢复。 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    RobotControl_ArmReceive();
}

/* 主循环中的任务维护：停车重试优先，其次检查底盘总时限和电机状态。
 * 升降按住模式检查 300 ms 续期；定角标定和前后定距自主运行到位。 */
void RobotControl_Tick(void)
{
    if (s_origin_initializing &&
        (uint32_t)(HAL_GetTick() - s_origin_retry_tick) >= 500U) RobotControl_CaptureOrigin();
    RobotControl_TickLiftAngle();
    RobotControl_TickArmMove();
    RobotControl_TickPose();
    RobotControl_TickGrab();
    RobotControl_TickManualGripper();
    RobotControl_TickManualBase();
    /* 停车失败期间持续重试，并拒绝新的 MOVE。 */
    if (s_stop_pending)
    {
        if (RobotControl_StopMotors() == HAL_OK)
        {
            s_stop_pending = false;
            s_motion = 'S';
        }
    }
    else if ((s_motion != 'S') &&
        ((uint32_t)(HAL_GetTick() - s_move_started_tick) >= ROBOT_MOVE_MAX_TIME_MS))
    {
        s_stop_pending = (RobotControl_StopMotors() != HAL_OK);
        if (!s_stop_pending) s_motion = 'S';
    }
    else if ((s_motion != 'S') &&
             ((uint32_t)(HAL_GetTick() - s_status_tick) >= ROBOT_STATUS_INTERVAL_MS))
    {
        uint8_t flags = 0U;
        s_status_tick = HAL_GetTick();
        /* 每隔至少 50 ms 查询一个轮子，一轮四个轮子约 200 ms。
         * 3A 状态：bit0 使能，bit1 到位，bit2 堵转，bit3 堵转保护。 */
        if ((ZDT_Motor_ReadStatus((uint8_t)(s_status_wheel + 1U), &flags) != HAL_OK) ||
            ((flags & 0x01U) == 0U) || ((flags & 0x0CU) != 0U))
        {
            s_stop_pending = (RobotControl_StopMotors() != HAL_OK);
            if (!s_stop_pending) s_motion = 'S';
        }
        else
        {
            uint8_t bit = (uint8_t)(1U << s_status_wheel);
            if ((flags & 0x02U) != 0U) s_reached_mask |= bit;
            else s_reached_mask &= (uint8_t)~bit;
            s_status_wheel = (uint8_t)((s_status_wheel + 1U) % ZDT_MOTOR_WHEEL_COUNT);
            /* 一轮查询完成且四轮均到位才结束任务；不能只看一个电机。 */
            if ((s_status_wheel == 0U) && (s_reached_mask == 0x0FU))
            {
                s_motion = 'S';
                s_wheel_move_reached = true;
            }
        }
    }
    RobotControl_TickAlignment();
    if (!s_lift_angle_active && (s_lift_motion != 'H') &&
        ((uint32_t)(HAL_GetTick() - s_last_lift_tick) >=
         ROBOT_COMMAND_TIMEOUT_MS))
    {
        if (LiftMotor_Stop() == HAL_OK)
        {
            s_lift_motion = 'H';
            s_lift_command_ok = true;
        }
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
    case 'I': /* App 启动原点初始化 */
    case 'O': /* 返回启动原点，不重新定义零点 */
    case 'P': /* 固定放下状态 */
    case 'A': /* 固定抓取状态 */
    case 'Z': /* 取消抓取，保持当前舵机角度 */
    case 'S':
    case 'U':
    case 'D':
    case 'H':
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

    for (index = 0U; index < ZDT_MOTOR_WHEEL_COUNT; ++index)
    {
        if (ZDT_Motor_Enable((uint8_t)wheels[index], true) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/* 对四轮分别发送立即停止，某一轮失败也继续尝试其他轮。
 * 只有四轮应答均成功才返回 HAL_OK。 */
static HAL_StatusTypeDef RobotControl_StopMotors(void)
{
    s_wheel_move_reached = false;
    static const ZDT_WheelId wheels[ZDT_MOTOR_WHEEL_COUNT] =
    {
        ZDT_WHEEL_LEFT_UP,
        ZDT_WHEEL_RIGHT_UP,
        ZDT_WHEEL_RIGHT_DOWN,
        ZDT_WHEEL_LEFT_DOWN
    };
    uint32_t index;
    HAL_StatusTypeDef result = HAL_OK;

    for (index = 0U; index < ZDT_MOTOR_WHEEL_COUNT; ++index)
    {
        if (ZDT_Motor_Stop((uint8_t)wheels[index], false) != HAL_OK)
        {
            result = HAL_ERROR;
        }
    }
    return result;
}

/* 目标脉冲 = round(距离毫米 × 每米脉冲 / 1000)。每米脉冲已由 App
 * 根据轮径、车轮每圈脉冲和修正系数计算，不在这里再次乘减速比。
 * 四轮顺序：左前、右前、右后、左后；正负号由驱动映射到 CW/CCW。 */
static HAL_StatusTypeDef RobotControl_ApplyDistance(const RobotDistanceCommand *command)
{
    return RobotControl_ApplyWheelPulses(command->direction, (int32_t)RobotProtocol_MovePulses(command),
        (uint16_t)command->speed_rpm);
}
static HAL_StatusTypeDef RobotControl_ApplyWheelPulses(char direction, int32_t pulses, uint16_t rpm)
{
    int32_t wheel_pulses[ZDT_MOTOR_WHEEL_COUNT];
    if (pulses <= 0) return HAL_ERROR;

    if (RobotControl_EnableMotors() != HAL_OK)
    {
        return HAL_ERROR;
    }

    switch (direction)
    {
    case 'F':
        wheel_pulses[0] = -pulses;
        wheel_pulses[1] = -pulses;
        wheel_pulses[2] = -pulses;
        wheel_pulses[3] = -pulses;
        break;
    case 'B':
        wheel_pulses[0] = pulses;
        wheel_pulses[1] = pulses;
        wheel_pulses[2] = pulses;
        wheel_pulses[3] = pulses;
        break;
    case 'L':
        /* 麦克纳姆轮左平移（实车前后方向已反标定）。
         * 轮序：左前、右前、右后、左后。 */
        wheel_pulses[0] = pulses;
        wheel_pulses[1] = -pulses;
        wheel_pulses[2] = pulses;
        wheel_pulses[3] = -pulses;
        break;
    case 'C': /* Left turn: left wheels backward, right wheels forward. */
        wheel_pulses[0] = pulses; wheel_pulses[1] = -pulses;
        wheel_pulses[2] = -pulses; wheel_pulses[3] = pulses;
        break;
    case 'W': /* Right turn. */
        wheel_pulses[0] = -pulses; wheel_pulses[1] = pulses;
        wheel_pulses[2] = pulses; wheel_pulses[3] = -pulses;
        break;
    case 'R':
        /* 麦克纳姆轮右平移。 */
        wheel_pulses[0] = -pulses;
        wheel_pulses[1] = pulses;
        wheel_pulses[2] = -pulses;
        wheel_pulses[3] = pulses;
        break;
    default:
        return HAL_ERROR;
    }

    return ZDT_Motor_MoveWheelPulses(wheel_pulses, rpm,
                                    ROBOT_ACCELERATION);
}

static bool RobotControl_SequenceIdle(void)
{
    return !s_manual_base_active && !s_manual_gripper_active && !s_pose_active && !s_grab_active && !s_align_active &&
        s_arm_origin_valid && s_lift_origin_valid && !s_origin_initializing &&
        s_motion == 'S' && !s_stop_pending && s_lift_motion == 'H' && s_lift_command_ok &&
        !s_arm_move_active && !s_lift_angle_active && !s_arm_move_stop_pending && !s_lift_angle_stop_pending &&
        s_fore_aft_motion == 'Q' && s_fore_aft_command_ok;
}
static bool RobotControl_StartSequence(const RobotSequenceCommand *settings, char direction)
{
    if (!RobotControl_SequenceIdle()) return false;
    s_release_mode = direction == 'P';
    memcpy(s_sequence_steps, s_release_mode ? s_release_steps : s_grab_steps, sizeof(s_sequence_steps));
    s_sequence_r_rpm = ROBOT_GRAB_ARM_RPM;
    s_sequence_up_rpm = ROBOT_GRAB_ARM_RPM;
    s_sequence_down_rpm = ROBOT_SEQUENCE_DOWN_RPM;
    s_gripper_open_ms = ROBOT_GRIPPER_OPEN_MS;
    s_sequence_gripper_dps = 60U; s_sequence_theta = 0U;
    if (settings != NULL) {
        unsigned first = s_release_mode ? 2U : 1U;
        s_sequence_steps[first].r_tenths_mm = settings->r1;
        s_sequence_steps[first].z_mm = settings->z1;
        s_sequence_steps[5].r_tenths_mm = settings->r2;
        s_sequence_steps[5].z_mm = settings->z2;
        s_sequence_r_rpm = (uint16_t)settings->r_rpm;
        s_sequence_up_rpm = (uint16_t)settings->up_rpm;
        s_sequence_down_rpm = (uint16_t)settings->down_rpm;
        s_sequence_gripper_dps = settings->gripper_dps;
        s_sequence_theta = (uint16_t)settings->theta;
        for (unsigned i = 0; i < 8U; ++i) {
            s_sequence_steps[i].gripper = (uint16_t)(s_sequence_steps[i].gripper == 60U ? settings->open_angle : settings->close_angle);
            s_sequence_steps[i].base = (uint16_t)(s_sequence_steps[i].base == 248U ? settings->base_home : settings->base_tilt);
        }
    }
    s_grab_step = 0U;
    s_grab_active = RobotControl_ApplyGrabStep();
    s_grab_tick = HAL_GetTick();
    if (!s_grab_active) RobotControl_CancelGrab();
    return s_grab_active;
}

/* 单向命令：格式错误、重复或忙碌任务直接忽略，不向 ESP32 回包。
 * 电机底层 ACK、到位查询、故障停车和停车重试仍保留。 */
static void RobotControl_HandleFrame(const char *frame)
{
    uint32_t sequence;
    char direction;
    uint16_t angle;
    if (strncmp(frame, "PLAN_BEGIN,", 11U) == 0 || strncmp(frame, "PLAN_RUN,", 9U) == 0) {
        RobotPlanCommand cmd;
        if (!RobotProtocol_ParsePlan(frame, &cmd) || s_plan_active || !RobotControl_SequenceIdle()) return;
        if (!cmd.run) {
            if (cmd.sequence == s_last_plan_sequence) return;
            s_plan_sequence = cmd.sequence; s_plan_count = (uint8_t)cmd.count; s_plan_mask = 0U;
        } else {
            if (cmd.sequence != s_plan_sequence || cmd.sequence == s_last_plan_sequence || s_plan_count == 0U ||
                s_plan_mask != ((1UL << s_plan_count) - 1U)) return;
            s_last_plan_sequence = cmd.sequence; s_plan_index = 0U; s_plan_active = true;
            if (!RobotControl_StartSequence(&s_plan_items[0], s_plan_items[0].mode)) RobotControl_CancelGrab();
        }
        return;
    }
    if (strncmp(frame, "PLAN_ITEM,", 10U) == 0) {
        RobotSequenceCommand item;
        if (!RobotProtocol_ParseSequence(frame, &item) || s_plan_active || !RobotControl_SequenceIdle() ||
            item.sequence != s_plan_sequence || item.plan_index >= s_plan_count) return;
        s_plan_items[item.plan_index] = item; s_plan_mask |= 1UL << item.plan_index;
        return;
    }
    RobotOriginCommand origin = {.base = 248U};
    bool origin_command = strncmp(frame, "ORIGIN,", 7U) == 0;
    if (origin_command && !RobotProtocol_ParseOrigin(frame, &origin)) return;
    if (origin_command) sequence = origin.sequence;
    if (origin_command && (s_origin_initializing || (s_arm_origin_valid && s_lift_origin_valid))) {
        /* Updating the initial base angle is a manual override, not re-homing.
         * Preserve captured r/z origins, turntable and gripper target. */
        if (sequence == s_last_origin_sequence) return;
        s_manual_base_active = false; s_manual_gripper_active = false;
        RobotControl_CancelPose(); RobotControl_CancelGrab();
        if (!RobotControl_SetServoAngle('B', (uint16_t)origin.base)) return;
        s_origin_base = (uint16_t)origin.base; s_last_origin_sequence = sequence;
        return;
    }
    if (origin_command || (RobotControl_Parse(frame, &sequence, &direction) && direction == 'I')) {
        if (sequence == 0U || sequence == s_last_origin_sequence || s_origin_initializing ||
            s_arm_origin_valid || s_lift_origin_valid || s_manual_base_active || s_manual_gripper_active || s_pose_active || s_grab_active || s_align_active ||
            s_motion != 'S' || s_stop_pending || s_arm_move_active || s_lift_angle_active ||
            s_lift_motion != 'H' || !s_lift_command_ok || s_fore_aft_motion != 'Q' || !s_fore_aft_command_ok) return;
        s_last_origin_sequence = sequence;
        if (!RobotControl_SetServoAngle('T', 0U) || !RobotControl_SetServoAngle('G', 15U) ||
            !RobotControl_SetServoAngle('B', (uint16_t)origin.base)) return;
        s_origin_base = (uint16_t)origin.base;
        s_grab_turntable = 0U; s_origin_initializing = true;
        RobotControl_CaptureOrigin();
        return;
    }
    if (strncmp(frame, "ARM_POSE,", 9U) == 0 || strncmp(frame, "CMD,", 4U) == 0) {
        RobotArmPoseCommand pose;
        bool home = false, pose_command = strncmp(frame, "ARM_POSE,", 9U) == 0;
        if (pose_command) {
            if (!RobotProtocol_ParseArmPose(frame, &pose)) return;
        } else if (RobotControl_Parse(frame, &sequence, &direction) && direction == 'O') {
            if (sequence == 0U) return;
            pose = (RobotArmPoseCommand){.sequence = sequence, .theta = 0U, .r_mm = 0,
                .z_mm = 0, .speed_rpm = 20U, .pulses_per_revolution = 3200U};
            home = true; pose_command = true;
        }
        if (pose_command) {
            if (s_manual_base_active || s_manual_gripper_active || !s_arm_origin_valid || !s_lift_origin_valid || s_pose_active || s_grab_active ||
                s_align_active || s_motion != 'S' || s_stop_pending || s_arm_move_active ||
                s_lift_angle_active || s_lift_motion != 'H' || !s_lift_command_ok ||
                s_fore_aft_motion != 'Q' || !s_fore_aft_command_ok || pose.sequence == s_last_pose_sequence) return;
            s_last_pose_sequence = pose.sequence; s_pose = pose; s_pose_home = home;
            s_pose_active = RobotControl_StartArmAbsolute(pose.r_mm, (uint16_t)pose.speed_rpm,
                                                        pose.pulses_per_revolution);
            s_pose_phase = 0U;
            return;
        }
    }
    if (strncmp(frame, "LIFT_ANGLE,", 11U) == 0 || strncmp(frame, "LIFT_MOVE,", 10U) == 0) {
        if (s_pose_active || !s_lift_origin_valid) return;
        RobotLiftAngleCommand command = {0};
        uint32_t angle_tenths;
        if (strncmp(frame, "LIFT_MOVE,", 10U) == 0) {
            RobotLiftDistanceCommand move;
            if (!RobotProtocol_ParseLiftDistance(frame, &move)) return;
            command.sequence = move.sequence; command.direction = move.direction;
            command.speed_rpm = move.speed_rpm; command.pulses_per_revolution = move.pulses_per_revolution;
            angle_tenths = RobotProtocol_LiftDistanceAngle(&move);
        } else {
            if (!RobotProtocol_ParseLiftAngle(frame, &command)) return;
            angle_tenths = command.angle_degrees * 10U;
        }
        if (s_lift_angle_active ||
            s_lift_motion != 'H' || !s_lift_command_ok || s_grab_active ||
            command.sequence == s_last_lift_angle_sequence) return;
        s_last_lift_angle_sequence = command.sequence;
        if (LiftMotor_MoveAngle(command.direction, angle_tenths,
            (uint16_t)command.speed_rpm, command.pulses_per_revolution) != HAL_OK)
        { RobotControl_StopLiftAngle(); return; }
        s_lift_angle_active = true; s_lift_angle_stop_pending = false;
        s_lift_motion = 'P'; s_lift_command_ok = true;
        s_lift_angle_start_tick = s_lift_angle_status_tick =
            s_lift_angle_last_status_tick = HAL_GetTick();
        return;
    }
    if (strncmp(frame, "PARALLEL,", 9U) == 0) {
        CameraParallelCommand cmd;
        if (!CameraProtocol_ParseParallel(frame, &cmd) || !CameraLink_Ready() || s_pose_active ||
            s_grab_active || s_align_active || s_arm_move_active || s_lift_angle_active || s_motion != 'S' ||
            s_stop_pending || !s_wheel_uart_ready || cmd.sequence == s_last_align_sequence) return;
        s_parallel = cmd; s_align_parallel = true; s_align_active = true; s_align_phase = ALIGN_WAIT;
        s_last_align_sequence = cmd.sequence; s_align_forward_ppm = s_align_lateral_ppm = cmd.ppm;
        s_align_start_tick = s_align_phase_tick = HAL_GetTick(); s_align_moves = s_align_centered = 0U;
        CameraLink_SelectTarget(4U); return;
    }
    if ((strncmp(frame, "ALIGN,", 6U) == 0 || strncmp(frame, "ALIGN_RING,", 11U) == 0 || strncmp(frame, "ALIGN_CFG,", 10U) == 0)) {
        if (s_pose_active) return;
        CameraAlignCommand cmd;
        if (!CameraProtocol_ParseAlign(frame, &cmd) || !CameraLink_Ready() ||
            s_align_active || s_grab_active || s_arm_move_active || s_motion != 'S' ||
            s_stop_pending || !s_wheel_uart_ready || cmd.sequence == s_last_align_sequence) return;
        s_last_align_sequence = cmd.sequence; s_align_parallel = false; s_align_active = true; s_align_phase = ALIGN_WAIT;
        s_align_forward_ppm = cmd.forward_ppm; s_align_lateral_ppm = cmd.lateral_ppm;
        s_align_settings = cmd.settings;
        s_align_start_tick = s_align_phase_tick = HAL_GetTick();
        s_align_moves = s_align_centered = 0U; CameraLink_SelectTarget(cmd.ring_index); return;
    }
    if (strncmp(frame, "ARM_MOVE,", 9U) == 0) {
        if (s_pose_active || !s_arm_origin_valid) return;
        RobotArmDistanceCommand command;
        if (!RobotProtocol_ParseArmDistance(frame, &command) ||
            command.sequence == s_last_arm_move_sequence || s_arm_move_active || s_grab_active || s_align_active ||
            s_fore_aft_motion != 'Q' || !s_fore_aft_command_ok) return;
        s_last_arm_move_sequence = command.sequence;
        (void)RobotControl_StartArmMove(command.direction, command.distance_mm,
            (uint16_t)command.speed_rpm, command.pulses_per_revolution);
        return;
    }
    if (strncmp(frame, "MOVE,", 5U) == 0) {
        if (s_pose_active) return;
        RobotDistanceCommand command;
        if (!RobotProtocol_ParseMove(frame, &command) ||
            RobotProtocol_MovePulses(&command) == 0U ||
            command.sequence == s_last_move_sequence ||
            s_motion != 'S' || s_align_active || s_stop_pending || !s_wheel_uart_ready) return;
        s_last_move_sequence = command.sequence;
        s_wheel_move_reached = false;
        if (RobotControl_ApplyDistance(&command) != HAL_OK) {
            s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        } else {
            s_motion = command.direction;
            s_move_started_tick = HAL_GetTick();
            s_status_tick = s_move_started_tick;
            s_status_wheel = 0U;
            s_reached_mask = 0U;
        }
        return;
    }
    if (strncmp(frame, "BASE,", 5U) == 0) {
        RobotBaseCommand cmd;
        if (!RobotProtocol_ParseBase(frame, &cmd) || cmd.sequence == s_manual_base_sequence || s_origin_initializing) return;
        RobotControl_CancelPose(); RobotControl_CancelGrab();
        s_manual_base_active = false;
        s_manual_gripper_active = false;
        s_manual_base_sequence = cmd.sequence;
        /* Before any origin/manual/automatic base command, physical position is
         * unknown. Establish the first target directly; never invent a start. */
        if (!s_base_angle_known) { (void)RobotControl_SetServoAngle('B', (uint16_t)cmd.target); return; }
        uint32_t distance = s_base_angle > cmd.target ? s_base_angle - cmd.target : cmd.target - s_base_angle;
        if (distance == 0U) return;
        s_manual_base_start = (int32_t)s_base_angle; s_manual_base_target = (int32_t)cmd.target;
        s_manual_base_ms = (distance * 1000U + cmd.dps - 1U) / cmd.dps;
        s_manual_base_tick = s_manual_base_update = HAL_GetTick();
        s_manual_base_active = true; return;
    }
    if (strncmp(frame, "GRIP,", 5U) == 0 || strncmp(frame, "GRIP_STOP,", 10U) == 0) {
        RobotGripperCommand cmd;
        if (!RobotProtocol_ParseGripper(frame, &cmd)) return;
        if (cmd.stop) { s_manual_gripper_active = false; return; }
        if (cmd.sequence == s_manual_gripper_sequence || s_origin_initializing) return;
        RobotControl_CancelPose(); RobotControl_CancelGrab();
        s_manual_base_active = false;
        s_manual_gripper_active = false;
        if (!RobotControl_SetServoAngle('G', (uint16_t)cmd.start)) return;
        s_manual_gripper_sequence = cmd.sequence;
        s_manual_gripper_start = (int32_t)cmd.start; s_manual_gripper_end = (int32_t)cmd.end;
        uint32_t distance = cmd.start > cmd.end ? cmd.start - cmd.end : cmd.end - cmd.start;
        s_manual_gripper_ms = (distance * 1000U + cmd.dps - 1U) / cmd.dps;
        if (distance == 0U) return;
        s_manual_gripper_tick = s_manual_gripper_update = HAL_GetTick();
        s_manual_gripper_active = true; return;
    }
    if (strncmp(frame, "SERVO,", 6U) == 0) {
        if (RobotControl_ParseServo(frame, &sequence, &direction, &angle)) {
            s_manual_base_active = false;
            s_manual_gripper_active = false;
            RobotControl_CancelPose();
            RobotControl_CancelGrab(); /* 手动调角优先，并停止抓取插入的前后移动。 */
            (void)RobotControl_SetServoAngle(direction, angle);
        }
        return;
    }
    RobotSequenceCommand settings;
    bool configured = strncmp(frame, "STATE,", 6U) == 0;
    if (configured) {
        if (!RobotProtocol_ParseSequence(frame, &settings)) return;
        sequence = settings.sequence; direction = settings.mode;
    } else if (!RobotControl_Parse(frame, &sequence, &direction)) return;
    if (direction == 'Z') { s_manual_base_active = false; s_manual_gripper_active = false; s_origin_initializing = false; RobotControl_CancelPose(); RobotControl_CancelAlignment(); RobotControl_CancelGrab(); return; }
    if (direction == 'A' || direction == 'P') {
        if (s_plan_active || !RobotControl_SequenceIdle()) return;
        uint32_t *last_sequence = direction == 'P' ? &s_last_release_sequence : &s_last_grab_sequence;
        if (sequence == 0U || sequence == *last_sequence || s_grab_active || s_align_active || s_lift_angle_active ||
            s_arm_move_active || s_fore_aft_motion != 'Q' || !s_fore_aft_command_ok) return;
        *last_sequence = sequence;
        (void)RobotControl_StartSequence(configured ? &settings : NULL, direction);
        return;
    }
    if (direction == 'U' || direction == 'D' || direction == 'H') {
        if (s_grab_active) {
            if (direction == 'H') RobotControl_CancelGrab();
            return;
        }
        if (s_pose_active) {
            if (direction == 'H') RobotControl_CancelPose();
            return;
        }
        if (direction != 'H' && !s_lift_origin_valid) return;
        if (s_lift_angle_active) {
            if (direction == 'H') RobotControl_StopLiftAngle();
            return;
        }
        s_last_lift_tick = HAL_GetTick();
        if (direction == s_lift_motion && s_lift_command_ok) return;
        if ((direction == 'H' ? LiftMotor_Stop() : LiftMotor_Move(direction)) != HAL_OK) {
            s_lift_motion = direction;
            s_lift_command_ok = false;
            if (LiftMotor_Stop() == HAL_OK) s_lift_motion = 'H';
            return;
        }
        s_lift_motion = direction;
        s_lift_command_ok = true;
        return;
    }
    if (direction == 'Q') {
        if (s_pose_active) { RobotControl_CancelPose(); return; }
        if (s_grab_active) { RobotControl_CancelGrab(); return; }
        /* 即使任务状态为空闲也向已使能的 ID2 请求停止。 */
        RobotControl_StopArmMove();
        return;
    }
    s_align_active = false;
    CameraLink_Discard();
    s_stop_pending = RobotControl_StopMotors() != HAL_OK;
    if (!s_stop_pending) s_motion = 'S';
}
