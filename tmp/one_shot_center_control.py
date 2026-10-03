from pathlib import Path
p = Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c')
s = p.read_text(encoding='utf-8')
s = s.replace('/* 摄像头持续输出坐标。每次移动等待四轮到位、静置 300 ms 后重新定位。 */', '/* 中心修正只读取一次坐标；平行/联合姿态修正仍按到位、静置、重采样循环。 */')
s = s.replace('static HAL_StatusTypeDef RobotControl_StopMotors(void);\nstatic HAL_StatusTypeDef RobotControl_StopMotors(void);', 'static HAL_StatusTypeDef RobotControl_StopMotors(void);')
a = s.index('static void RobotControl_TickAlignment(void)')
s = s[:a] + '''static void RobotControl_StartCenterCompensation(void)
{
    if (s_align_settings.offset_mm == 0) { RobotControl_CancelAlignment(); return; }
    int32_t offset = s_align_settings.offset_mm;
    RobotDistanceCommand move = {0U, offset > 0 ? CAMERA_IMAGE_RIGHT_DIRECTION :
        CameraProtocol_Opposite(CAMERA_IMAGE_RIGHT_DIRECTION),
        (uint32_t)(offset > 0 ? offset : -offset), s_align_settings.fine_rpm, s_align_forward_ppm};
    s_wheel_move_reached = false;
    if (RobotControl_ApplyDistance(&move) != HAL_OK) {
        s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        if (!s_stop_pending) s_motion = 'S';
        s_align_active = false; return;
    }
    s_motion = move.direction; s_move_started_tick = HAL_GetTick();
    s_status_tick = s_move_started_tick; s_status_wheel = s_reached_mask = 0U;
    s_align_phase = ALIGN_COMPENSATING; s_align_phase_tick = s_move_started_tick;
}
static void RobotControl_CenterAlignment(void)
{
    CameraCenter center; int32_t pulses[4]; uint16_t speeds[4]; bool aligned;
    if (!CameraLink_TakeCenter(&center)) return;
    if (!CameraProtocol_CenterProfile(&center, s_align_forward_ppm, s_align_lateral_ppm,
        &s_align_settings, pulses, speeds, &aligned)) { RobotControl_CancelAlignment(); return; }
    if (aligned) { RobotControl_StartCenterCompensation(); return; }
    s_wheel_move_reached = false;
    if (RobotControl_EnableMotors() != HAL_OK ||
        ZDT_Motor_MoveWheelProfile(pulses, speeds, ROBOT_ACCELERATION) != HAL_OK) {
        s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        if (!s_stop_pending) s_motion = 'S';
        s_align_active = false; return;
    }
    /* One AA packet contains both complete translation axes. Further samples
     * are ignored; after arrival only the configured final offset may run. */
    s_motion = 'J'; s_move_started_tick = HAL_GetTick(); s_status_tick = s_move_started_tick;
    s_status_wheel = s_reached_mask = 0U; ++s_align_moves;
    s_align_phase = ALIGN_MOVING; s_align_phase_tick = s_move_started_tick;
}
''' + s[a:]
s = s.replace('''            if (s_align_phase == ALIGN_COMPENSATING) { RobotControl_CancelAlignment(); return; }
            s_align_phase = ALIGN_SETTLE;''', '''            if (s_align_phase == ALIGN_COMPENSATING) { RobotControl_CancelAlignment(); return; }
            if (!s_align_parallel && !s_align_joint && s_align_settings.offset_mm == 0) {
                RobotControl_CancelAlignment(); return;
            }
            s_align_phase = ALIGN_SETTLE;''')
s = s.replace('''            CameraLink_Discard(); s_align_phase = ALIGN_WAIT; s_align_phase_tick = now;''', '''            CameraLink_Discard();
            if (!s_align_parallel && !s_align_joint) { RobotControl_StartCenterCompensation(); return; }
            s_align_phase = ALIGN_WAIT; s_align_phase_tick = now;''')
s = s.replace('''    if (s_align_joint) { RobotControl_JointAlignment(now); return; }
    CameraCenter center = {256U,160U};''', '''    if (s_align_joint) { RobotControl_JointAlignment(now); return; }
    if (!s_align_parallel) { RobotControl_CenterAlignment(); return; }''')
s = s.replace('    bool compensating = false;\n', '')
a = s.index('    } else {\n        if (!CameraLink_TakeCenter(&center)) return;', s.index('static void RobotControl_TickAlignment(void)'))
b = s.index('    s_align_centered = 0U;', a)
s = s[:a] + '    }\n' + s[b:]
s = s.replace('    if (!compensating && s_align_moves >= ROBOT_ALIGN_MAX_MOVES)', '    if (s_align_moves >= ROBOT_ALIGN_MAX_MOVES)')
s = s.replace('    if (!compensating) ++s_align_moves;\n    s_align_phase = compensating ? ALIGN_COMPENSATING : ALIGN_MOVING;', '    ++s_align_moves;\n    s_align_phase = ALIGN_MOVING;')
p.write_text(s, encoding='utf-8')
