from pathlib import Path
root=Path('F:/GONGXUNSAIPROJ/yundong_part')
def edit(name, fn):
 p=root/name; p.write_text(fn(p.read_text(encoding='utf-8')),encoding='utf-8')
def protocol(s):
 a=s.index('\n#endif')
 return s[:a]+'''
/* 前后电机定角测试；角度以 0.1° 为单位，细分参数只用于 Emm。 */
typedef struct {
    uint32_t sequence;
    char direction;
    uint32_t angle_tenths;
    uint32_t speed_rpm;
    uint32_t pulses_per_revolution;
} RobotArmAngleCommand;

static inline bool RobotProtocol_ParseArmAngle(const char *frame, RobotArmAngleCommand *command)
{
    if (strncmp(frame, "ARM,", 4U) != 0) return false;
    const char *p = frame + 4U;
    if (!RobotProtocol_U32(&p, &command->sequence, ',')) return false;
    command->direction = *p;
    if ((*p != 'E' && *p != 'C') || p[1] != ',') return false;
    p += 2;
    if (!RobotProtocol_U32(&p, &command->angle_tenths, ',') ||
        !RobotProtocol_U32(&p, &command->speed_rpm, ',') ||
        !RobotProtocol_U32(&p, &command->pulses_per_revolution, '\\0')) return false;
    return command->sequence != 0U && command->angle_tenths >= 1U &&
           command->angle_tenths <= 36000U && command->speed_rpm >= 5U &&
           command->speed_rpm <= 60U && command->pulses_per_revolution >= 200U &&
           command->pulses_per_revolution <= 51200U;
}
static inline uint32_t RobotProtocol_ArmAnglePulses(const RobotArmAngleCommand *command)
{
    return (uint32_t)(((uint64_t)command->angle_tenths * command->pulses_per_revolution + 1800U) / 3600U);
}
'''+s[a:]
edit(Path('shared/robot_distance_protocol.h'),protocol)
def driver(s):
 a=s.index('HAL_StatusTypeDef ArmMotor_StopForeAft(void)')
 return s[:a]+'''/* USART2 ID2 定角移动，02 从当前实时位置开始，停止后可重新测量。
 * Emm FD: 微步脉冲；X FD: 0.1°，不能混用两个固件的帧。 */
HAL_StatusTypeDef ArmMotor_MoveAngle(char direction, uint32_t angle_tenths,
                                    uint16_t speed_rpm, uint32_t pulses_per_rev)
{
    if ((direction != 'E' && direction != 'C') || angle_tenths == 0U ||
        angle_tenths > 36000U || speed_rpm < 5U || speed_rpm > 60U ||
        pulses_per_rev < 200U || pulses_per_rev > 51200U) return HAL_ERROR;
    uint8_t id = FORE_AFT_MOTOR_ID;
    uint8_t firmware = ArmMotor_DetectFirmware(id);
    if (firmware == ARM_FIRMWARE_UNKNOWN) return HAL_ERROR;
    uint32_t pulses = (uint32_t)(((uint64_t)angle_tenths * pulses_per_rev + 1800U) / 3600U);
    if (firmware == ARM_FIRMWARE_EMM && pulses == 0U) return HAL_ERROR;
    const uint8_t enable[] = {id, 0xF3U, 0xABU, 0x01U, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    /* 每次位置测试重新确认使能，避免沿用失效的使能状态。 */
    if (LiftMotor_Send(enable, sizeof(enable), id, 0xF3U) != HAL_OK) return HAL_ERROR;
    s_fore_aft_enabled = true;
    uint8_t dir = direction == 'E' ? ARM_FORWARD_DIRECTION : ARM_BACK_DIRECTION;
    uint8_t emm[] = {id, 0xFDU, dir, (uint8_t)(speed_rpm >> 8), (uint8_t)speed_rpm,
        LIFT_ACCELERATION, (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)pulses, 0x02U, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    uint16_t x_speed = speed_rpm * 10U;
    uint8_t x[] = {id, 0xFDU, dir, 0x00U, 0xC8U, 0x00U, 0xC8U,
        (uint8_t)(x_speed >> 8), (uint8_t)x_speed,
        (uint8_t)(angle_tenths >> 24), (uint8_t)(angle_tenths >> 16),
        (uint8_t)(angle_tenths >> 8), (uint8_t)angle_tenths,
        0x02U, 0x00U, ZDT_MOTOR_FRAME_TAIL};
    return firmware == ARM_FIRMWARE_X ? LiftMotor_Send(x, sizeof(x), id, 0xFDU) :
                                       LiftMotor_Send(emm, sizeof(emm), id, 0xFDU);
}

'''+s[a:]
edit(Path('stm32h743/zhukong/Core/Src/lift_motor.c'),driver)
edit(Path('stm32h743/zhukong/Core/Inc/lift_motor.h'),lambda s:s.replace('HAL_StatusTypeDef ArmMotor_StopForeAft(void);','''HAL_StatusTypeDef ArmMotor_StopForeAft(void);
/* 前后电机轴转角，不是舵机角度；angle_tenths 单位 0.1°。 */
HAL_StatusTypeDef ArmMotor_MoveAngle(char direction, uint32_t angle_tenths,
                                    uint16_t speed_rpm, uint32_t pulses_per_rev);'''))
def stm(s):
 a=s.index('/* 抓取固定姿态')
 s=s[:a]+'''#define ROBOT_ARM_TEST_MAX_MS     180000U
static bool s_arm_test_active;
static bool s_arm_test_stop_pending;
static uint32_t s_last_arm_test_sequence;
static uint32_t s_arm_test_start_tick;
static uint32_t s_arm_test_status_tick;

static void RobotControl_StopArmTest(void)
{
    s_arm_test_stop_pending = ArmMotor_StopForeAft() != HAL_OK;
    s_arm_test_active = s_arm_test_stop_pending;
    s_fore_aft_motion = s_arm_test_stop_pending ? 'P' : 'Q';
    s_fore_aft_command_ok = !s_arm_test_stop_pending;
}

static void RobotControl_TickArmTest(void)
{
    if (!s_arm_test_active) return;
    if (s_arm_test_stop_pending) { RobotControl_StopArmTest(); return; }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_arm_test_start_tick) >= ROBOT_ARM_TEST_MAX_MS)
    { RobotControl_StopArmTest(); return; }
    if ((uint32_t)(now - s_arm_test_status_tick) < 50U) return;
    s_arm_test_status_tick = now;
    uint8_t flags;
    if (!LiftMotor_ReadStatus(2U, &flags) || (flags & 0x01U) == 0U || (flags & 0x0CU) != 0U)
    { RobotControl_StopArmTest(); return; }
    if ((flags & 0x02U) != 0U) {
        s_arm_test_active = false;
        s_fore_aft_motion = 'Q';
        s_fore_aft_command_ok = true;
    }
}

'''+s[a:]
 s=s.replace('    s_grab_active = false;\n    s_grab_base_rotating = false;', '''    s_arm_test_active = false;
    s_arm_test_stop_pending = false;
    s_last_arm_test_sequence = 0U;
    s_grab_active = false;
    s_grab_base_rotating = false;''')
 s=s.replace('    RobotControl_TickGrab();', '    RobotControl_TickGrab();\n    RobotControl_TickArmTest();')
 s=s.replace("    if ((s_fore_aft_motion != 'Q') &&", "    if (!s_arm_test_active && (s_fore_aft_motion != 'Q') &&")
 a=s.index('    if (strncmp(frame, "MOVE,", 5U) == 0)',s.index('static void RobotControl_HandleFrame(const char *frame)\n{'))
 s=s[:a]+'''    if (strncmp(frame, "ARM,", 4U) == 0) {
        RobotArmAngleCommand command;
        if (!RobotProtocol_ParseArmAngle(frame, &command) ||
            command.sequence == s_last_arm_test_sequence || s_arm_test_active ||
            s_fore_aft_motion != 'Q' || !s_fore_aft_command_ok) return;
        s_last_arm_test_sequence = command.sequence;
        if (ArmMotor_MoveAngle(command.direction, command.angle_tenths,
            (uint16_t)command.speed_rpm, command.pulses_per_revolution) != HAL_OK)
            RobotControl_StopArmTest();
        else {
            s_arm_test_active = true;
            s_arm_test_stop_pending = false;
            s_fore_aft_motion = 'P';
            s_fore_aft_command_ok = true;
            s_arm_test_start_tick = HAL_GetTick();
            s_arm_test_status_tick = s_arm_test_start_tick;
        }
        return;
    }
'''+s[a:]
 s=s.replace("    if (direction == 'E' || direction == 'C' || direction == 'Q') {",'''    if (direction == 'E' || direction == 'C' || direction == 'Q') {
        if (s_arm_test_active) {
            RobotControl_StopArmTest();
            if (s_arm_test_stop_pending) return;
        }''')
 return s
edit(Path('stm32h743/zhukong/Core/Src/robot_control.c'),stm)
def esp(s):
 s=s.replace('    if (strncmp(frame, "MOVE,", 5U) == 0) {','''    if (strncmp(frame, "ARM,", 4U) == 0) {
        RobotArmAngleCommand command;
        if (!RobotProtocol_ParseArmAngle(frame, &command)) return true;
        /* 自主位置运动不能继承之前按住按钮的 250 ms 续期计时。 */
        current_fore_aft = ARM_FORE_AFT_HOLD;
    } else if (strncmp(frame, "MOVE,", 5U) == 0) {''')
 return s
edit(Path('esp32s3/hello_world/main/robot_remote_main.c'),esp)
def client(s):
 a=s.index('    fun sendMotion(')
 return s[:a]+'''    fun sendArmAngle(direction: Char, angleTenths: Int, rpm: Int, pulsesPerRev: Int) {
        if (direction !in "EC" || angleTenths !in 1..36000 || rpm !in 5..60 ||
            pulsesPerRev !in 200..51200) return
        sendFrame("ARM,${nextSequence()},$direction,$angleTenths,$rpm,$pulsesPerRev\\n")
    }

'''+s[a:]
edit(Path('application/app/src/main/java/com/example/app/RobotTcpClient.kt'),client)
def app(s):
 s=s.replace('                    onGrab = {', '''                    onArmAngleTest = { direction, angle, rpm, pulses ->
                        cancelRepeatedForeAftCommand()
                        activeForeAftDirection = null
                        robotClient.sendArmAngle(direction, angle, rpm, pulses)
                    },
                    onGrab = {''')
 s=s.replace('    onGrab: () -> Unit,','    onArmAngleTest: (Char, Int, Int, Int) -> Unit,\n    onGrab: () -> Unit,')
 s=s.replace('        Text("舵机角度（首次拖动后生效）"', '        Text("舵机角度（首次拖动后生效）"')
 s=s.replace('        Spacer(Modifier.height(16.dp))\n        HorizontalDivider()\n        Spacer(Modifier.height(12.dp))\n        Text("固定状态', '''        Spacer(Modifier.height(16.dp))
        ArmAngleTestPanel(connected, onArmAngleTest, onStopForeAft)
        HorizontalDivider()
        Spacer(Modifier.height(12.dp))
        Text("固定状态''')
 s=s.replace('            onGrab = {},', '            onArmAngleTest = { _, _, _, _ -> },\n            onGrab = {},')
 a=s.index('@Composable\nprivate fun MotionButton(')
 s=s[:a]+'''@Composable
private fun ArmAngleTestPanel(
    connected: Boolean,
    onTest: (Char, Int, Int, Int) -> Unit,
    onStop: () -> Unit,
) {
    var angle by rememberSaveable { mutableStateOf("30") }
    var rpm by rememberSaveable { mutableStateOf("10") }
    var pulses by rememberSaveable { mutableStateOf("3200") }
    var measured by rememberSaveable { mutableStateOf("") }
    var note by rememberSaveable { mutableStateOf("等待测试") }
    var testedAngle by rememberSaveable { mutableStateOf(0.0) }
    Text("机械臂前后：步进电机定角测试", style = MaterialTheme.typography.titleMedium)
    Text("控制 USART2 电机 ID2 的相对转角。每次只发一次；无到位回包，停稳后再测试。")
    NumberField("电机转角（°，0.1–3600）", angle, { angle = it }, connected, decimal = true)
    NumberField("速度（RPM，5–60）", rpm, { rpm = it }, connected)
    NumberField("电机每圈脉冲（默认 1.8° / 16 细分：3200）", pulses, { pulses = it }, connected)
    fun test(direction: Char) {
        val degrees = angle.toDoubleOrNull()
        val speed = rpm.toIntOrNull()
        val perRev = pulses.toIntOrNull()
        if (degrees == null || !degrees.isFinite() || degrees !in 0.1..3600.0 ||
            speed == null || speed !in 5..60 || perRev == null || perRev !in 200..51200) {
            note = "请输入有效角度、速度和每圈脉冲"
            return
        }
        val tenths = (degrees * 10).roundToInt()
        if ((tenths.toLong() * perRev + 1800) / 3600 == 0L) {
            note = "转角不足一个脉冲，请增大角度"
            return
        }
        testedAngle = tenths / 10.0
        measured = ""
        onTest(direction, tenths, speed, perRev)
        note = "已提交${if (direction == 'E') "前伸" else "后收"} $testedAngle°；请测量实际位移"
    }
    Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        Button(onClick = { test('E') }, enabled = connected) { Text("前伸测试") }
        Button(onClick = { test('C') }, enabled = connected) { Text("后收测试") }
        Button(onClick = { onStop(); testedAngle = 0.0; note = "已请求停止，本次不用于标定" }, enabled = connected) { Text("停止") }
    }
    Text(note)
    NumberField("本次实测位移（mm，填正数）", measured, { measured = it }, true, decimal = true)
    val mm = measured.toDoubleOrNull()
    if (testedAngle > 0 && mm != null && mm.isFinite() && mm > 0) {
        Text("位移/角度：%.4f mm/°；每转一圈：%.3f mm".format(mm / testedAngle, mm * 360 / testedAngle))
    }
    Text("每圈脉冲需匹配驱动细分；先小角度测试，再重复测量不同角度和方向。")
    Spacer(Modifier.height(16.dp))
}

'''+s[a:]
 return s
edit(Path('application/app/src/main/java/com/example/app/MainActivity.kt'),app)
