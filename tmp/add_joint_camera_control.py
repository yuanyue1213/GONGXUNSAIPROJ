from pathlib import Path
p = Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c')
s = p.read_text(encoding='utf-8').replace('#include "../../../../shared/robot_distance_protocol.h"', '#include "../../../../shared/robot_distance_protocol.h"\n#include "../../../../shared/camera_pose_math.h"')
s = s.replace('static bool s_align_active, s_align_parallel;', 'static bool s_align_active, s_align_parallel, s_align_joint;\nstatic CameraPoseAlignCommand s_joint;')
idx = s.index('static void RobotControl_TickAlignment(void)')
s = s[:idx] + '''static void RobotControl_JointAlignment(uint32_t now)
{
    CameraCenter rings[3]; int32_t pulses[4]; uint16_t speeds[4]; bool aligned;
    if (!CameraLink_TakeRings(rings)) return;
    if (!CameraProtocol_PoseProfile(&s_joint, rings, pulses, speeds, &aligned)) {
        s_align_centered = 0U; return;
    }
    s_align_phase_tick = now;
    if (aligned) {
        if (++s_align_centered >= 3U) RobotControl_CancelAlignment();
        return;
    }
    s_align_centered = 0U;
    if (s_align_moves >= ROBOT_ALIGN_MAX_MOVES) { RobotControl_CancelAlignment(); return; }
    s_wheel_move_reached = false;
    if (RobotControl_EnableMotors() != HAL_OK ||
        ZDT_Motor_MoveWheelProfile(pulses, speeds, ROBOT_ACCELERATION) != HAL_OK) {
        s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        if (!s_stop_pending) s_motion = 'S';
        s_align_active = false; return;
    }
    /* One AA packet moves translation and yaw together. Re-measure only after
     * all four wheels arrive and the existing settle phase has completed. */
    s_motion = 'J'; s_move_started_tick = HAL_GetTick(); s_status_tick = s_move_started_tick;
    s_status_wheel = s_reached_mask = 0U; ++s_align_moves;
    s_align_phase = ALIGN_MOVING; s_align_phase_tick = s_move_started_tick;
}
''' + s[idx:]
s = s.replace('    CameraCenter center = {256U,160U};', '    if (s_align_joint) { RobotControl_JointAlignment(now); return; }\n    CameraCenter center = {256U,160U};')
# Declarations for the motor calls used by the new early camera tick helper.
s = s.replace('static HAL_StatusTypeDef RobotControl_ApplyWheelPulses(char direction, int32_t pulses, uint16_t rpm);', 'static HAL_StatusTypeDef RobotControl_ApplyWheelPulses(char direction, int32_t pulses, uint16_t rpm);\nstatic HAL_StatusTypeDef RobotControl_EnableMotors(void);\nstatic HAL_StatusTypeDef RobotControl_StopMotors(void);')
s = s.replace('s_parallel = cmd; s_align_parallel = true;', 's_parallel = cmd; s_align_joint = false; s_align_parallel = true;')
s = s.replace('s_last_align_sequence = cmd.sequence; s_align_parallel = false;', 's_last_align_sequence = cmd.sequence; s_align_joint = false; s_align_parallel = false;')
idx = s.index('    if (strncmp(frame, "PARALLEL,", 9U)', s.index('static void RobotControl_HandleFrame(const char *frame)\n{'))
s = s[:idx] + '''    if (strncmp(frame, "ALIGN_POSE,", 11U) == 0) {
        CameraPoseAlignCommand cmd;
        if (!CameraProtocol_ParsePoseAlign(frame, &cmd) || !CameraLink_Ready() || s_pose_active ||
            s_grab_active || s_align_active || s_arm_move_active || s_lift_angle_active || s_motion != 'S' ||
            s_stop_pending || !s_wheel_uart_ready || cmd.sequence == s_last_align_sequence) return;
        s_joint = cmd; s_align_joint = true; s_align_parallel = false; s_align_active = true;
        s_last_align_sequence = cmd.sequence; s_align_phase = ALIGN_WAIT;
        s_align_start_tick = s_align_phase_tick = HAL_GetTick(); s_align_moves = s_align_centered = 0U;
        CameraLink_SelectTarget(4U); return;
    }
''' + s[idx:]
p.write_text(s, encoding='utf-8')
p = Path('yundong_part/esp32s3/hello_world/main/robot_remote_main.c')
s = p.read_text(encoding='utf-8')
idx = s.index('\n', s.index('#include "', s.index('robot_distance_protocol.h')-40)) if False else 0
s = '#include "../../../shared/camera_pose_math.h"\n' + s
s = s.replace('    if (strncmp(frame, "BASE,", 5U) == 0) {', '''    if (strncmp(frame, "ALIGN_POSE,", 11U) == 0) {
        CameraPoseAlignCommand command;
        if (!CameraProtocol_ParsePoseAlign(frame, &command)) return true;
    } else if (strncmp(frame, "BASE,", 5U) == 0) {''', 1)
p.write_text(s, encoding='utf-8')
p = Path('tmp/test_esp_servo_forwarding.py')
s = p.read_text(encoding='utf-8').replace('#include "../yundong_part/shared/camera_position_protocol.h"', '#include "../yundong_part/shared/camera_position_protocol.h"\n#include "../yundong_part/shared/camera_pose_math.h"')
s = s.replace('const char *commands[] = {', 'const char *commands[] = {"ALIGN_POSE,1004,2,9889,9889,10,20,10,300,250,0,0,0", ')
p.write_text(s, encoding='utf-8')
