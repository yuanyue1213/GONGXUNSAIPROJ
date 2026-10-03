from pathlib import Path
p=Path('yundong_part/shared/robot_distance_protocol.h');s=p.read_text(encoding='utf-8').replace("(*p != 'L') && (*p != 'R')","(*p != 'L') && (*p != 'R') && (*p != 'C') && (*p != 'W')");p.write_text(s,encoding='utf-8')
p=Path('yundong_part/shared/camera_position_protocol.h');s=p.read_text(encoding='utf-8');pos=s.rfind('#endif');s=s[:pos]+'''typedef struct { uint32_t sequence, ppm, rpm, step_mm, reverse; } CameraParallelCommand;
static inline bool CameraProtocol_ParseParallel(const char *frame, CameraParallelCommand *cmd)
{
    if (strncmp(frame, "PARALLEL,", 9U) != 0) return false;
    const char *p = frame + 9U;
    return RobotProtocol_U32(&p, &cmd->sequence, ',') && RobotProtocol_U32(&p, &cmd->ppm, ',') &&
        RobotProtocol_U32(&p, &cmd->rpm, ',') && RobotProtocol_U32(&p, &cmd->step_mm, ',') &&
        RobotProtocol_U32(&p, &cmd->reverse, '\\0') && cmd->sequence != 0U && cmd->ppm >= 1U &&
        cmd->ppm <= 1000000U && cmd->rpm >= 5U && cmd->rpm <= 60U && cmd->step_mm >= 1U &&
        cmd->step_mm <= 10U && cmd->reverse <= 1U;
}
/* Least-squares slope using all three centers, in thousandths (image Y points down).
 * Reject narrow/degenerate baselines and non-collinear detections (>5 px residual). */
static inline bool CameraProtocol_RingSlope(const CameraCenter c[3], int32_t *slope)
{
    if (c[2].x < c[0].x + 80U || c[1].x <= c[0].x || c[2].x <= c[1].x) return false;
    int64_t sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (unsigned i = 0; i < 3U; ++i) {
        sx += c[i].x; sy += c[i].y; sxx += (int64_t)c[i].x*c[i].x; sxy += (int64_t)c[i].x*c[i].y;
    }
    int64_t den = 3*sxx - sx*sx, num = 3*sxy - sx*sy;
    if (den <= 0) return false;
    for (unsigned i = 0; i < 3U; ++i) {
        int64_t residual = (3*(int64_t)c[i].y - sy)*den - num*(3*(int64_t)c[i].x - sx);
        if (residual < 0) residual = -residual;
        if (residual > 15*den) return false;
    }
    *slope = (int32_t)(num*1000/den); return true;
}
'''+s[pos:];p.write_text(s,encoding='utf-8')
p=Path('yundong_part/stm32h743/zhukong/Core/Inc/camera_link.h');s=p.read_text(encoding='utf-8').replace('void CameraLink_SelectTarget','bool CameraLink_TakeRings(CameraCenter rings[3]);\nvoid CameraLink_SelectTarget');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/stm32h743/zhukong/Core/Src/camera_link.c');s=p.read_text(encoding='utf-8').replace('static CameraCenter s_center;','static CameraCenter s_center, s_rings[3];').replace('''                } else if (s_ring_index >= 1U && s_ring_index <= 3U &&
                           CameraProtocol_ParseRings(s_frame, rings)) {
                    s_center = rings[s_ring_index - 1U]; s_valid = true;
''','''                } else if (s_ring_index >= 1U && s_ring_index <= 4U &&
                           CameraProtocol_ParseRings(s_frame, rings)) {
                    memcpy(s_rings, rings, sizeof(s_rings));
                    if (s_ring_index <= 3U) s_center = rings[s_ring_index - 1U];
                    s_valid = true;
''').replace('    if (!s_valid) return false;','    if (!s_valid || s_ring_index == 4U) return false;');s+='''
bool CameraLink_TakeRings(CameraCenter rings[3])
{
    if (!s_valid || s_ring_index != 4U) return false;
    memcpy(rings, s_rings, sizeof(s_rings)); s_valid = false; return true;
}
''';p.write_text(s,encoding='utf-8')
p=Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c');s=p.read_text(encoding='utf-8').replace('static bool s_align_active;','static bool s_align_active, s_align_parallel;\nstatic CameraParallelCommand s_parallel;')
a='''    CameraCenter center;
    if (!CameraLink_TakeCenter(&center)) return;
    s_align_phase_tick = now;
    char direction; uint32_t mm;
    bool compensating = false;
    if (!CameraProtocol_Correction(&center, &direction, &mm)) {''';b='''    CameraCenter center = {256U,160U};
    char direction; uint32_t mm; uint16_t rpm;
    bool compensating = false;
    if (s_align_parallel) {
        CameraCenter rings[3]; int32_t slope;
        if (!CameraLink_TakeRings(rings) || !CameraProtocol_RingSlope(rings, &slope)) return;
        s_align_phase_tick = now;
        uint32_t error = (uint32_t)(slope < 0 ? -slope : slope);
        if (error <= 17U) {
            if (++s_align_centered >= 3U) RobotControl_CancelAlignment();
            return;
        }
        direction = (slope > 0) != (s_parallel.reverse != 0U) ? 'C' : 'W';
        mm = error / 25U + 1U;
        if (mm > s_parallel.step_mm) mm = s_parallel.step_mm;
        rpm = (uint16_t)s_parallel.rpm;
    } else {
        if (!CameraLink_TakeCenter(&center)) return;
        s_align_phase_tick = now;
        rpm = CameraProtocol_ConfiguredRpm(&center, &s_align_settings);
        if (!CameraProtocol_Correction(&center, &direction, &mm)) {''';assert a in s;s=s.replace(a,b)
a='''        compensating = true;
    }
    s_align_centered = 0U;''';b='''        compensating = true;
        rpm = (uint16_t)s_align_settings.fine_rpm;
        }
    }
    s_align_centered = 0U;''';assert a in s;s=s.replace(a,b).replace('compensating ? s_align_settings.fine_rpm : CameraProtocol_ConfiguredRpm(&center, &s_align_settings),','rpm,')
s=s.replace("    case 'R':\n        /*", "    case 'C': /* Left turn: left wheels backward, right wheels forward. */\n        wheel_pulses[0] = pulses; wheel_pulses[1] = -pulses;\n        wheel_pulses[2] = -pulses; wheel_pulses[3] = pulses;\n        break;\n    case 'W': /* Right turn. */\n        wheel_pulses[0] = -pulses; wheel_pulses[1] = pulses;\n        wheel_pulses[2] = pulses; wheel_pulses[3] = -pulses;\n        break;\n    case 'R':\n        /*")
pos=s.index('    if ((strncmp(frame, "ALIGN,"');s=s[:pos]+'''    if (strncmp(frame, "PARALLEL,", 9U) == 0) {
        CameraParallelCommand cmd;
        if (!CameraProtocol_ParseParallel(frame, &cmd) || !CameraLink_Ready() || s_pose_active ||
            s_grab_active || s_align_active || s_arm_move_active || s_lift_angle_active || s_motion != 'S' ||
            s_stop_pending || !s_wheel_uart_ready || cmd.sequence == s_last_align_sequence) return;
        s_parallel = cmd; s_align_parallel = true; s_align_active = true; s_align_phase = ALIGN_WAIT;
        s_last_align_sequence = cmd.sequence; s_align_forward_ppm = s_align_lateral_ppm = cmd.ppm;
        s_align_start_tick = s_align_phase_tick = HAL_GetTick(); s_align_moves = s_align_centered = 0U;
        CameraLink_SelectTarget(4U); return;
    }
'''+s[pos:];s=s.replace('s_last_align_sequence = cmd.sequence; s_align_active = true; s_align_phase = ALIGN_WAIT;','s_last_align_sequence = cmd.sequence; s_align_parallel = false; s_align_active = true; s_align_phase = ALIGN_WAIT;');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/esp32s3/hello_world/main/robot_remote_main.c');s=p.read_text(encoding='utf-8').replace('    } else if (strncmp(frame, "GRIP,",','''    } else if (strncmp(frame, "PARALLEL,", 9U) == 0) {
        CameraParallelCommand command;
        if (!CameraProtocol_ParseParallel(frame, &command)) return true;
    } else if (strncmp(frame, "GRIP,",''');p.write_text(s,encoding='utf-8')
