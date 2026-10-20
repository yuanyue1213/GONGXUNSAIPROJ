from pathlib import Path
p = Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c')
s = p.read_text(encoding='utf-8').replace('#include "../../../../shared/camera_pose_math.h"', '#include "../../../../shared/camera_pose_math.h"\n#include "../../../../shared/arm_vector_math.h"')
s = s.replace('static uint16_t s_sequence_r_rpm, s_sequence_up_rpm, s_sequence_down_rpm;', 'static uint16_t s_sequence_combined_rpm;\nstatic bool s_sequence_position_active, s_sequence_descent_tail;\nstatic int64_t s_sequence_final_z;')
s = s.replace('#define ROBOT_GRAB_HOLD_MS       1000U', '#define ROBOT_GRAB_HOLD_MS        500U')
a = s.index('static bool RobotControl_SequenceZFirst(void)')
b = s.index('static void RobotControl_CancelGrab(void);', a)
s = s[:a] + s[b:]
s = s.replace('s_grab_arm_wait = s_grab_lift_wait = false;\n    s_grab_base_rotating', 's_grab_arm_wait = s_grab_lift_wait = false;\n    s_sequence_position_active = s_sequence_descent_tail = false;\n    s_grab_base_rotating')
a = s.index('static bool RobotControl_StartSequencePosition(void)')
b = s.index('static bool s_manual_gripper_active;', a)
s = s[:a] + '''/* Start both independent motor targets in the same control iteration; each
 * retains its own status watchdog. Any second-axis failure cancels the first. */
static bool RobotControl_StartSequenceAxis(uint8_t id, int64_t target, uint16_t rpm)
{
    if (ArmMotor_MoveAbsolute(id, target, rpm, ROBOT_GRAB_ARM_PULSES) != HAL_OK) {
        if (id == 2U) RobotControl_StopArmMove(); else RobotControl_StopLiftAngle();
        return false;
    }
    uint32_t now = HAL_GetTick();
    if (id == 2U) {
        s_grab_arm_wait = s_arm_move_active = true;
        s_arm_move_stop_pending = s_arm_move_reached = false;
        s_fore_aft_motion = 'P'; s_fore_aft_command_ok = true;
        s_arm_move_start_tick = s_arm_move_status_tick = s_arm_move_last_status_tick = now;
    } else {
        s_grab_lift_wait = s_lift_angle_active = true;
        s_lift_angle_stop_pending = s_lift_angle_reached = false;
        s_lift_motion = 'P'; s_lift_command_ok = true;
        s_lift_angle_start_tick = s_lift_angle_status_tick = s_lift_angle_last_status_tick = now;
    }
    return true;
}
static bool RobotControl_StartSequencePosition(void)
{
    const GrabStep *step = &RobotControl_SequenceSteps()[s_grab_step];
    if (!step->position) return true;
    int64_t current_r, current_z;
    if (!ArmMotor_ReadPosition(2U, &current_r) || !ArmMotor_ReadPosition(1U, &current_z)) return false;
    int64_t r = step->r_tenths_mm;
    int64_t r_angle = (llabs(r) * 360 + 63) / 127;
    int64_t target_r = s_arm_origin + (r < 0 ? -r_angle : r_angle);
    s_sequence_final_z = s_lift_origin - RobotControl_MmAngle(step->z_mm, ROBOT_LIFT_MM_PER_REV);
    int64_t stage_z = ArmVector_DescentStage(current_z, s_sequence_final_z);
    uint16_t r_rpm, z_rpm;
    if (!ArmVector_Speeds(target_r-current_r, stage_z-current_z, s_sequence_combined_rpm, &r_rpm, &z_rpm)) return false;
    s_sequence_descent_tail = stage_z != s_sequence_final_z;
    s_sequence_position_active = true;
    s_grab_arm_wait = s_grab_lift_wait = false;
    if (r_rpm != 0U && !RobotControl_StartSequenceAxis(2U, target_r, r_rpm)) return false;
    if (z_rpm != 0U && !RobotControl_StartSequenceAxis(1U, stage_z, z_rpm)) {
        RobotControl_CancelGrab(); return false;
    }
    return true;
}
''' + s[b:]
s = s.replace('(s_grab_step == 4U || s_grab_step == 7U || (s_grab_step == 0U && s_base_angle != step->base))', '(s_grab_step == 4U || (s_grab_step == 0U && s_base_angle != step->base))')
a = s.index('    if (s_grab_arm_wait) {', s.index('static void RobotControl_TickGrab(void)'))
b = s.index('    uint32_t now = HAL_GetTick(), elapsed', a)
s = s[:a] + '''    if (s_sequence_position_active) {
        if ((s_grab_arm_wait && (s_arm_move_stop_pending || (!s_arm_move_active && !s_arm_move_reached))) ||
            (s_grab_lift_wait && (s_lift_angle_stop_pending || (!s_lift_angle_active && !s_lift_angle_reached)))) {
            RobotControl_CancelGrab(); return;
        }
        if ((s_grab_arm_wait && s_arm_move_active) || (s_grab_lift_wait && s_lift_angle_active)) return;
        s_grab_arm_wait = s_grab_lift_wait = false;
        if (s_sequence_descent_tail) {
            s_sequence_descent_tail = false;
            if (!RobotControl_StartSequenceAxis(1U, s_sequence_final_z, 300U)) RobotControl_CancelGrab();
            return;
        }
        s_sequence_position_active = false;
        (void)RobotControl_FinishSequencePosition();
        return;
    }
''' + s[b:]
s = s.replace('s_sequence_r_rpm = ROBOT_GRAB_ARM_RPM;\n    s_sequence_up_rpm = ROBOT_GRAB_ARM_RPM;\n    s_sequence_down_rpm = ROBOT_SEQUENCE_DOWN_RPM;', 's_sequence_combined_rpm = ROBOT_GRAB_ARM_RPM;\n    s_sequence_position_active = s_sequence_descent_tail = false;')
s = s.replace('s_sequence_r_rpm = (uint16_t)settings->r_rpm;\n        s_sequence_up_rpm = (uint16_t)settings->up_rpm;\n        s_sequence_down_rpm = (uint16_t)settings->down_rpm;', 's_sequence_combined_rpm = (uint16_t)settings->combined_rpm;')
s = s.replace('    s_grab_step = 0U;\n    s_release_origin_pending', '    if (!s_release_mode) s_sequence_steps[7].base = s_sequence_steps[5].base; /* Keep storage angle at grab end. */\n    s_grab_step = 0U;\n    s_release_origin_pending')
s = s.replace('if (strncmp(frame, "PLAN_ITEM,", 10U) == 0)', 'if (strncmp(frame, "PLAN_ITEM,", 10U) == 0 || strncmp(frame, "PLAN_ITEM_V2,", 13U) == 0)')
s = s.replace('bool configured = strncmp(frame, "STATE,", 6U) == 0;', 'bool configured = strncmp(frame, "STATE,", 6U) == 0 || strncmp(frame, "STATE_V2,", 9U) == 0;')
s = s.replace('#define ROBOT_SEQUENCE_DOWN_RPM   50U\n', '')
p.write_text(s, encoding='utf-8')
