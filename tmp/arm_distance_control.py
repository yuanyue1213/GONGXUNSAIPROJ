from pathlib import Path
root=Path('F:/GONGXUNSAIPROJ/yundong_part')
def edit(name,fn):
 p=root/name; p.write_text(fn(p.read_text(encoding='utf-8')),encoding='utf-8')
def protocol(s):
 s=s.replace('/* 前后电机定角测试；角度以 0.1° 为单位，细分参数只用于 Emm。 */','/* 前后定距：实测电机一圈移动 127 mm；细分参数只用于 Emm。 */')
 s=s.replace('RobotArmAngleCommand','RobotArmDistanceCommand').replace('RobotProtocol_ParseArmAngle','RobotProtocol_ParseArmDistance')
 s=s.replace('uint32_t angle_tenths;', 'uint32_t distance_mm;')
 s=s.replace('strncmp(frame, "ARM,", 4U)', 'strncmp(frame, "ARM_MOVE,", 9U)').replace('const char *p = frame + 4U;', 'const char *p = frame + 9U;')
 s=s.replace('command->angle_tenths','command->distance_mm').replace('command->distance_mm <= 36000U','command->distance_mm <= 1000U')
 a=s.index('static inline uint32_t RobotProtocol_ArmAnglePulses')
 s=s[:a]+'''static inline uint32_t RobotProtocol_ArmDistanceAngle(const RobotArmDistanceCommand *command)
{
    /* 输出 0.1°，先按实测 127 mm/圈换算，再由驱动选择 Emm/X 帧。 */
    return (command->distance_mm * 3600U + 63U) / 127U;
}

#endif
'''
 return s
edit(Path('shared/robot_distance_protocol.h'),protocol)
def stm(s):
 s=s.replace('strncmp(frame, "ARM,", 4U)', 'strncmp(frame, "ARM_MOVE,", 9U)').replace('RobotArmAngleCommand','RobotArmDistanceCommand').replace('RobotProtocol_ParseArmAngle','RobotProtocol_ParseArmDistance').replace('command.direction, command.angle_tenths','command.direction, RobotProtocol_ArmDistanceAngle(&command)')
 s=s.replace('static uint32_t s_last_fore_aft_tick;\n','').replace('    s_last_fore_aft_tick = HAL_GetTick();\n','')
 a=s.index("    if (!s_arm_test_active && (s_fore_aft_motion != 'Q') &&")
 b=s.index('\n}\n',a)
 s=s[:a]+s[b:]
 s=s.replace("    case 'E':\n",'').replace("    case 'C':\n",'')
 a=s.index("    if (direction == 'E' || direction == 'C' || direction == 'Q') {")
 b=s.index('    s_stop_pending = RobotControl_StopMotors()',a)
 s=s[:a]+'''    if (direction == 'Q') {
        /* 即使任务状态为空闲也向已使能的 ID2 请求停止。 */
        RobotControl_StopArmTest();
        return;
    }
'''+s[b:]
 s=s.replace('s_arm_test','s_arm_move').replace('s_last_arm_test','s_last_arm_move').replace('ROBOT_ARM_TEST','ROBOT_ARM_MOVE').replace('RobotControl_StopArmTest','RobotControl_StopArmMove').replace('RobotControl_TickArmTest','RobotControl_TickArmMove')
 return s
edit(Path('stm32h743/zhukong/Core/Src/robot_control.c'),stm)
def esp(s):
 s=s.replace('static motion_t current_fore_aft = ARM_FORE_AFT_HOLD;\n','').replace('    current_fore_aft = ARM_FORE_AFT_HOLD;\n','')
 s=s.replace("    ARM_FORWARD = 'E',\n",'').replace("    ARM_BACKWARD = 'C',\n",'').replace("    case 'E':\n",'').replace("    case 'C':\n",'')
 s=s.replace('static bool handle_frame(const char *frame, int64_t *last_lift_us,\n                         int64_t *last_fore_aft_us)', 'static bool handle_frame(const char *frame, int64_t *last_lift_us)')
 s=s.replace('strncmp(frame, "ARM,", 4U)', 'strncmp(frame, "ARM_MOVE,", 9U)').replace('RobotArmAngleCommand','RobotArmDistanceCommand').replace('RobotProtocol_ParseArmAngle','RobotProtocol_ParseArmDistance')
 s=s.replace('        /* 自主位置运动不能继承之前按住按钮的 250 ms 续期计时。 */\n','')
 a=s.index('        } else if (motion == ARM_FORWARD')
 b=s.index('        return true;',a)
 s=s[:a]+'        }\n'+s[b:]
 s=s.replace('    int64_t last_fore_aft_us = esp_timer_get_time();\n','').replace('&last_lift_us, &last_fore_aft_us','&last_lift_us')
 a=s.index('        /* 伸缩按住时')
 b=s.index('\n    }\n\ndisconnected:',a)
 s=s[:a]+s[b:]
 s=s.replace('机械臂升降、伸缩各自超过 250 ms 未续期时请求停止；底盘无心跳超时','机械臂升降超过 250 ms 未续期时请求停止；底盘和前后定距无心跳超时').replace('机械臂两轴独立维护续期状态；底盘只转发命令，不等待回包','仅升降维护续期状态；底盘和前后定距只转发命令，不等待回包').replace('E/C/Q 伸/收/停','Q 停止前后移动；E/C 只用于 ARM_MOVE 的方向')
 return s
edit(Path('esp32s3/hello_world/main/robot_remote_main.c'),esp)
def driver(s):
 s=s.replace('#define FORE_AFT_SPEED_RPM     30U\n','')
 a=s.index('/* E/C 对应伸出/收回')
 b=s.index('/* USART2 ID2 定角移动',a)
 return s[:a]+s[b:]
edit(Path('stm32h743/zhukong/Core/Src/lift_motor.c'),driver)
edit(Path('stm32h743/zhukong/Core/Inc/lift_motor.h'),lambda s:s.replace('HAL_StatusTypeDef ArmMotor_MoveForeAft(char direction);\n',''))
def client(s):
 s=s.replace('fun sendArmAngle(direction: Char, angleTenths: Int, rpm: Int, pulsesPerRev: Int)', 'fun sendArmDistance(direction: Char, distanceMm: Int, rpm: Int, pulsesPerRev: Int)')
 s=s.replace('angleTenths !in 1..36000','distanceMm !in 1..1000').replace('"ARM,${nextSequence()},$direction,$angleTenths,$rpm,$pulsesPerRev\\n"','"ARM_MOVE,${nextSequence()},$direction,$distanceMm,$rpm,$pulsesPerRev\\n"')
 return s.replace('direction !in "SUDHECQAZ"','direction !in "SUDHQAZ"')
edit(Path('application/app/src/main/java/com/example/app/RobotTcpClient.kt'),client)
def app(s):
 s=s.replace('    private var repeatedForeAftCommand: Runnable? = null\n','').replace('    private var activeForeAftDirection: Char? = null\n','')
 s=s.replace('                        cancelRepeatedForeAftCommand()\n','')
 s=s.replace('                    onStartForeAft = ::startForeAft,\n','').replace('    onStartForeAft: (Char) -> Unit,\n','').replace('            onStartForeAft = {},\n','')
 a=s.index('                    onArmAngleTest = {')
 b=s.index('                    onGrab = {',a)
 s=s[:a]+'''                    onArmDistance = { direction, distance, rpm, pulses ->
                        robotClient.sendArmDistance(direction, distance, rpm, pulses)
                    },
'''+s[b:]
 a=s.index('    private fun startForeAft(')
 b=s.index('    private fun queueServoAngle(',a)
 s=s[:a]+'''    private fun stopForeAft() {
        robotClient.sendMotion('Q')
    }

'''+s[b:]
 s=s.replace('onArmAngleTest','onArmDistance')
 a=s.index('        Text("机械臂前后：按住运动')
 b=s.index('        ArmAngleTestPanel(',a)
 s=s[:a]+s[b:]
 s=s.replace('ArmAngleTestPanel','ArmDistancePanel')
 a=s.index('@Composable\nprivate fun ArmDistancePanel(')
 b=s.index('@Composable\nprivate fun MotionButton(',a)
 s=s[:a]+'''@Composable
private fun ArmDistancePanel(
    connected: Boolean,
    onMove: (Char, Int, Int, Int) -> Unit,
    onStop: () -> Unit,
) {
    var distance by rememberSaveable { mutableStateOf("10") }
    var rpm by rememberSaveable { mutableStateOf("10") }
    var pulses by rememberSaveable { mutableStateOf("3200") }
    var note by rememberSaveable { mutableStateOf("等待前后定距移动") }
    Text("机械臂前后：定距移动", style = MaterialTheme.typography.titleMedium)
    Text("实测标定：电机一圈 360° = 127 mm（12.7 cm）。")
    DistanceInput("移动距离（mm，1–1000）", distance, connected) { distance = it }
    DistanceInput("速度（RPM，5–60）", rpm, connected) { rpm = it }
    DistanceInput("电机每圈脉冲（默认 1.8° / 16 细分：3200）", pulses, connected) { pulses = it }
    fun move(direction: Char) {
        val mm = distance.toIntOrNull()
        val speed = rpm.toIntOrNull()
        val perRev = pulses.toIntOrNull()
        if (mm == null || mm !in 1..1000 || speed == null || speed !in 5..60 ||
            perRev == null || perRev !in 200..51200) {
            note = "请输入有效距离、速度和每圈脉冲"
            return
        }
        onMove(direction, mm, speed, perRev)
        note = "已提交${if (direction == 'E') "前伸" else "后收"} $mm mm"
    }
    Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        Button(onClick = { move('E') }, enabled = connected) { Text("前伸") }
        Button(onClick = { move('C') }, enabled = connected) { Text("后收") }
        Button(onClick = { onStop(); note = "已请求停止前后移动" }, enabled = connected) { Text("停止") }
    }
    Text(note)
    Text("每次只发一次，无到位回包；机构停稳后再发送。每圈脉冲需匹配电机细分。")
    Spacer(Modifier.height(16.dp))
}

'''+s[b:]
 return s
edit(Path('application/app/src/main/java/com/example/app/MainActivity.kt'),app)
