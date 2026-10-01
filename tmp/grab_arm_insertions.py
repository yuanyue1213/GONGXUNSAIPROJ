from pathlib import Path
root=Path('F:/GONGXUNSAIPROJ/yundong_part')
p=root/'stm32h743/zhukong/Core/Src/robot_control.c'
s=p.read_text(encoding='utf-8')
s=s.replace('static bool s_arm_move_active;', 'static bool s_arm_move_active;\nstatic bool s_arm_move_reached;')
s=s.replace('static void RobotControl_StopArmMove(void)\n{','static void RobotControl_StopArmMove(void)\n{\n    s_arm_move_reached = false;')
s=s.replace("    if ((flags & 0x02U) != 0U) {\n        s_arm_move_active = false;", "    if ((flags & 0x02U) != 0U) {\n        s_arm_move_reached = true;\n        s_arm_move_active = false;")
a=s.index('/* 抓取固定姿态')
s=s[:a]+'''/* 手动定距与抓取插入运动共用到位/故障处理，不生成模拟 App 序号。 */
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
    return true;
}

'''+s[a:]
s=s.replace('#define ROBOT_GRAB_HOLD_MS', '#define ROBOT_GRAB_ARM_RPM        10U\n#define ROBOT_GRAB_ARM_PULSES     3200U\n#define ROBOT_GRAB_HOLD_MS')
s=s.replace('static bool s_grab_active;', 'static bool s_grab_active;\nstatic bool s_grab_arm_wait;')
a=s.index('static bool RobotControl_ApplyGrabStep(void)')
s=s[:a]+'''static void RobotControl_CancelGrab(void)
{
    s_grab_active = false;
    if (s_grab_arm_wait) RobotControl_StopArmMove();
    s_grab_arm_wait = false;
}

'''+s[a:]
s=s.replace('    if (!s_grab_active) return;\n    uint32_t now', '''    if (!s_grab_active) return;
    if (s_grab_arm_wait) {
        if (s_arm_move_stop_pending) { s_grab_active = false; s_grab_arm_wait = false; return; }
        if (s_arm_move_active) return;
        s_grab_arm_wait = false;
        if (!s_arm_move_reached) { s_grab_active = false; return; }
        s_grab_active = RobotControl_ApplyGrabStep();
        s_grab_tick = HAL_GetTick();
        return;
    }
    uint32_t now''')
s=s.replace('''    else
    {
        s_grab_active = RobotControl_ApplyGrabStep();
        s_grab_tick = HAL_GetTick();
    }
}

static bool RobotControl_Parse''','''    else if (s_grab_step == 1U || s_grab_step == 2U)
    {
        /* 状态 1→2 插入前伸 110 mm；状态 2→3 插入后收 90 mm。
         * 不按延时猜测完成，必须收到 ID2 到位状态才继续舵机步骤。 */
        s_grab_arm_wait = RobotControl_StartArmMove(s_grab_step == 1U ? 'E' : 'C',
            s_grab_step == 1U ? 110U : 90U, ROBOT_GRAB_ARM_RPM, ROBOT_GRAB_ARM_PULSES);
        if (!s_grab_arm_wait) s_grab_active = false;
    }
    else
    {
        s_grab_active = RobotControl_ApplyGrabStep();
        s_grab_tick = HAL_GetTick();
    }
}

static bool RobotControl_Parse''')
s=s.replace('    s_arm_move_active = false;\n    s_arm_move_stop_pending = false;', '    s_arm_move_active = false;\n    s_arm_move_reached = false;\n    s_arm_move_stop_pending = false;')
s=s.replace('    s_grab_active = false;\n    s_grab_base_rotating = false;', '    s_grab_active = false;\n    s_grab_arm_wait = false;\n    s_grab_base_rotating = false;')
s=s.replace('    RobotControl_TickGrab();\n    RobotControl_TickArmMove();', '    RobotControl_TickArmMove();\n    RobotControl_TickGrab();')
s=s.replace('command.sequence == s_last_arm_move_sequence || s_arm_move_active ||', 'command.sequence == s_last_arm_move_sequence || s_arm_move_active || s_grab_active ||')
a=s.index('        if (ArmMotor_MoveAngle(command.direction',s.index('static void RobotControl_HandleFrame(const char *frame)\n{'))
b=s.index('        return;',a)
s=s[:a]+'''        (void)RobotControl_StartArmMove(command.direction, command.distance_mm,
            (uint16_t)command.speed_rpm, command.pulses_per_revolution);
'''+s[b:]
s=s.replace('s_grab_active = false; /* 手动调角优先，取消自动后续步骤。 */','RobotControl_CancelGrab(); /* 手动调角优先，并停止抓取插入的前后移动。 */')
s=s.replace("if (direction == 'Z') { s_grab_active = false; return; }", "if (direction == 'Z') { RobotControl_CancelGrab(); return; }")
s=s.replace('sequence == s_last_grab_sequence || s_grab_active) return;', "sequence == s_last_grab_sequence || s_grab_active ||\n            s_arm_move_active || s_fore_aft_motion != 'Q' || !s_fore_aft_command_ok) return;")
s=s.replace("    if (direction == 'Q') {\n", "    if (direction == 'Q') {\n        if (s_grab_arm_wait) { RobotControl_CancelGrab(); return; }\n")
p.write_text(s,encoding='utf-8')
p=root/'application/app/src/main/java/com/example/app/MainActivity.kt'
s=p.read_text(encoding='utf-8').replace('Text("固定状态：抓取（6 步，第 3、6 步基座平滑旋转 3 秒，无到位回包）")', 'Text("固定状态：抓取（状态 1→前伸 11 cm→状态 2→后收 9 cm→状态 3–6）")\n        Text("第 3、6 步基座平滑旋转 3 秒；前后移动到位后继续，无应用回包。")')
p.write_text(s,encoding='utf-8')
p=root/'esp32s3/PROTOCOL.md'
s=p.read_text(encoding='utf-8')
s=s.replace('第 5 步转盘从 0° 加到 120°，第 6 步再加 120°，目标为 240°。', '''完整顺序：状态 1 → 前伸 110 mm → 状态 2 → 后收 90 mm → 状态 3–6。
每次前后移动均等待 USART2 ID2 到位状态才继续，不用固定延时代替到位。
插入移动使用 10 RPM、3200 脉冲/电机圈、127 mm/圈标定，常量在 robot_control.c。
对应目标约为前伸 311.8°、后收 255.1°，均相对当前实时位置运动。
前后运动失败、超时或故障终止抓取，停止失败仍重试。
取消抓取、手动 SERVO 或前后停止 Q 在插入运动期间会停止该轴并终止抓取。
抓取过程中忽略手动 ARM_MOVE；独立前后移动期间不启动抓取，防止争用 ID2。
第 5 步转盘从 0° 加到 120°，第 6 步再加 120°，目标为 240°。''')
p.write_text(s,encoding='utf-8')
