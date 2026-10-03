from pathlib import Path

p = Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c')
s = p.read_text(encoding='utf-8')
s = s.replace('ServoControl_SetAngle(', 'RobotControl_SetServoAngle(')
a = s.index('/* App 文本命令')
s = s[:a] + '''/* Track the last successfully commanded base angle across origin, automatic
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

''' + s[a:]
s = s.replace('    s_manual_gripper_active = false; s_manual_gripper_sequence = 0U;', '''    s_manual_gripper_active = false; s_manual_gripper_sequence = 0U;
    s_manual_base_active = false; s_manual_base_sequence = 0U; s_base_angle_known = false;''')
s = s.replace('    RobotControl_TickManualGripper();', '    RobotControl_TickManualGripper();\n    RobotControl_TickManualBase();')
s = s.replace('return !s_manual_gripper_active && !s_pose_active', 'return !s_manual_base_active && !s_manual_gripper_active && !s_pose_active')
s = s.replace('s_arm_origin_valid || s_lift_origin_valid || s_manual_gripper_active', 's_arm_origin_valid || s_lift_origin_valid || s_manual_base_active || s_manual_gripper_active')
s = s.replace('if (s_manual_gripper_active || !s_arm_origin_valid', 'if (s_manual_base_active || s_manual_gripper_active || !s_arm_origin_valid')
a = s.index('    if (strncmp(frame, "GRIP,", 5U)', s.index('static void RobotControl_HandleFrame(const char *frame)\n{'))
s = s[:a] + '''    if (strncmp(frame, "BASE,", 5U) == 0) {
        RobotBaseCommand cmd;
        if (!RobotProtocol_ParseBase(frame, &cmd) || cmd.sequence == s_manual_base_sequence || s_origin_initializing) return;
        RobotControl_CancelPose(); RobotControl_CancelGrab();
        s_manual_gripper_active = false; s_manual_base_active = false;
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
''' + s[a:]
s = s.replace('        RobotControl_CancelPose(); RobotControl_CancelGrab();\n        s_manual_gripper_active = false;', '        RobotControl_CancelPose(); RobotControl_CancelGrab();\n        s_manual_base_active = false;\n        s_manual_gripper_active = false;')
s = s.replace('''        if (RobotControl_ParseServo(frame, &sequence, &direction, &angle)) {
            s_manual_gripper_active = false;''', '''        if (RobotControl_ParseServo(frame, &sequence, &direction, &angle)) {
            s_manual_base_active = false;
            s_manual_gripper_active = false;''')
s = s.replace("if (direction == 'Z') { s_manual_gripper_active = false;", "if (direction == 'Z') { s_manual_base_active = false; s_manual_gripper_active = false;")
p.write_text(s, encoding='utf-8')

p = Path('yundong_part/application/app/src/main/java/com/example/app/MainActivity.kt')
s = p.read_text(encoding='utf-8')
s = s.replace('                    onServoFinished = ::sendServoAngle,', '                    onServoFinished = ::sendServoAngle,\n                    onBaseMove = { target, dps -> cancelPendingServos(); robotClient.sendBase(target, dps) },')
s = s.replace('    onServoFinished: (Char, Int) -> Unit,\n)', '    onServoFinished: (Char, Int) -> Unit,\n    onBaseMove: (Int, Int) -> Unit = { _, _ -> },\n)')
s = s.replace('                        ServoAngleInput("基座 · PC6", \'B\', 360, connected, onServoFinished)', '                        BaseAnglePanel(connected, onBaseMove, onStopArm)')
a = s.index('@Composable\nprivate fun GripperMotionPanel')
s = s[:a] + '''@Composable
private fun BaseAnglePanel(connected: Boolean, onMove: (Int, Int) -> Unit, onStop: () -> Unit) {
    var target by rememberSaveable { mutableStateOf("248") }
    var speed by rememberSaveable { mutableStateOf("60") }
    var error by remember { mutableStateOf<String?>(null) }
    Text("基座 · PC6", style = MaterialTheme.typography.titleMedium)
    DistanceInput("目标角度（0–360 度）", target, true) { target = it }
    DistanceInput("速度（1–360 度/秒）", speed, true) { speed = it }
    Text("先执行原点初始化，可从已记录角度平滑移动。")
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(onClick = {
        val angle = target.toIntOrNull(); val dps = speed.toIntOrNull()
        if (angle == null || angle !in 0..360 || dps == null || dps !in 1..360)
            error = "目标角度：0–360 度；速度：1–360 度/秒"
        else { error = null; onMove(angle, dps) }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("设置基座") }
    Button(onClick = onStop, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("停止机械臂及基座") }
}

''' + s[a:]
p.write_text(s, encoding='utf-8')
