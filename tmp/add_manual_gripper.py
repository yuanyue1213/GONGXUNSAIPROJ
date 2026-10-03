from pathlib import Path
p=Path('yundong_part/shared/robot_distance_protocol.h');s=p.read_text(encoding='utf-8').replace('speed_rpm <= 60U','speed_rpm <= 120U').replace('r_rpm <= 60U','r_rpm <= 120U').replace('up_rpm <= 60U','up_rpm <= 120U').replace('down_rpm <= 60U','down_rpm <= 120U');pos=s.rfind('#endif');s=s[:pos]+'''typedef struct { uint32_t sequence, start, end, dps; bool stop; } RobotGripperCommand;
static inline bool RobotProtocol_ParseGripper(const char *frame, RobotGripperCommand *cmd)
{
    const char *p;
    cmd->stop = strncmp(frame, "GRIP_STOP,", 10U) == 0;
    if (cmd->stop) {
        p = frame + 10U;
        return RobotProtocol_U32(&p, &cmd->sequence, '\\0') && cmd->sequence != 0U;
    }
    if (strncmp(frame, "GRIP,", 5U) != 0) return false;
    p = frame + 5U;
    return RobotProtocol_U32(&p, &cmd->sequence, ',') &&
        RobotProtocol_U32(&p, &cmd->start, ',') && RobotProtocol_U32(&p, &cmd->end, ',') &&
        RobotProtocol_U32(&p, &cmd->dps, '\\0') && cmd->sequence != 0U &&
        cmd->start <= 270U && cmd->end <= 270U && cmd->dps >= 6U && cmd->dps <= 300U;
}
'''+s[pos:];p.write_text(s,encoding='utf-8')
p=Path('yundong_part/stm32h743/zhukong/Core/Src/lift_motor.c');s=p.read_text(encoding='utf-8').replace('speed_rpm > 60U','speed_rpm > 120U');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c');s=p.read_text(encoding='utf-8').replace('263U','248U');pos=s.index('static bool RobotControl_ApplyGrabStep(void)');s=s[:pos]+'''static bool s_manual_gripper_active;
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
    if (!ServoControl_SetAngle('G', (uint16_t)angle) || elapsed >= s_manual_gripper_ms)
        s_manual_gripper_active = false;
    s_manual_gripper_update = now;
}

'''+s[pos:];s=s.replace('    RobotControl_TickGrab();','    RobotControl_TickGrab();\n    RobotControl_TickManualGripper();').replace('    s_grab_active = false;\n    s_release_mode = false;', '    s_grab_active = false;\n    s_manual_gripper_active = false; s_manual_gripper_sequence = 0U;\n    s_release_mode = false;')
pos=s.index('    if (strncmp(frame, "SERVO,", 6U) == 0) {');s=s[:pos]+'''    if (strncmp(frame, "GRIP,", 5U) == 0 || strncmp(frame, "GRIP_STOP,", 10U) == 0) {
        RobotGripperCommand cmd;
        if (!RobotProtocol_ParseGripper(frame, &cmd)) return;
        if (cmd.stop) { s_manual_gripper_active = false; return; }
        if (cmd.sequence == s_manual_gripper_sequence || s_origin_initializing) return;
        RobotControl_CancelPose(); RobotControl_CancelGrab();
        s_manual_gripper_active = false;
        if (!ServoControl_SetAngle('G', (uint16_t)cmd.start)) return;
        s_manual_gripper_sequence = cmd.sequence;
        s_manual_gripper_start = (int32_t)cmd.start; s_manual_gripper_end = (int32_t)cmd.end;
        uint32_t distance = cmd.start > cmd.end ? cmd.start - cmd.end : cmd.end - cmd.start;
        s_manual_gripper_ms = (distance * 1000U + cmd.dps - 1U) / cmd.dps;
        if (distance == 0U) return;
        s_manual_gripper_tick = s_manual_gripper_update = HAL_GetTick();
        s_manual_gripper_active = true; return;
    }
'''+s[pos:];s=s.replace('            RobotControl_CancelPose();\n            RobotControl_CancelGrab();','            s_manual_gripper_active = false;\n            RobotControl_CancelPose();\n            RobotControl_CancelGrab();').replace("if (direction == 'Z') {", "if (direction == 'Z') { s_manual_gripper_active = false;")
# Absolute pose/origin/automatic sequences cannot compete with an active gripper ramp.
s=s.replace("if (s_pose_active || !s_arm_origin_valid || !s_lift_origin_valid", "if (s_manual_gripper_active || s_pose_active || !s_arm_origin_valid || !s_lift_origin_valid")
s=s.replace('s_arm_origin_valid || s_lift_origin_valid || s_pose_active','s_arm_origin_valid || s_lift_origin_valid || s_manual_gripper_active || s_pose_active').replace('if (!s_arm_origin_valid || !s_lift_origin_valid || s_pose_active','if (s_manual_gripper_active || !s_arm_origin_valid || !s_lift_origin_valid || s_pose_active');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/esp32s3/hello_world/main/robot_remote_main.c');s=p.read_text(encoding='utf-8').replace('    } else if (strncmp(frame, "STATE,", 6U) == 0) {','''    } else if (strncmp(frame, "GRIP,", 5U) == 0 || strncmp(frame, "GRIP_STOP,", 10U) == 0) {
        RobotGripperCommand command;
        if (!RobotProtocol_ParseGripper(frame, &command)) return true;
    } else if (strncmp(frame, "STATE,", 6U) == 0) {''');p.write_text(s,encoding='utf-8')
