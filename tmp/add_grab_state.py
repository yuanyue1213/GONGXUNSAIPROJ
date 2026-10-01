from pathlib import Path
root = Path('F:/GONGXUNSAIPROJ/yundong_part')
def edit(name, fn):
    p=root/name
    p.write_text(fn(p.read_text(encoding='utf-8')),encoding='utf-8')
def stm(s):
    s=s.replace('#define ROBOT_RX_BUFFER_SIZE', '#define ROBOT_GRAB_STEP_MS             1000U\n#define ROBOT_RX_BUFFER_SIZE')
    a=s.index('static bool RobotControl_Parse(')
    s=s[:a]+'''/* 抓取固定姿态：转盘、夹子、基座。定时推进，不阻塞底盘/停止命令处理。
 * 时间只保证命令间隔，不代表舵机已实际到位；按实车速度调整间隔。 */
static const uint16_t s_grab_angles[][3] = {
    {0U, 60U, 263U},
    {0U, 0U, 263U},
    {0U, 0U, 144U},
    {0U, 60U, 144U},
    {120U, 60U, 144U},
};
static bool s_grab_active;
static uint8_t s_grab_step;
static uint32_t s_grab_tick;
static uint32_t s_last_grab_sequence;

static bool RobotControl_ApplyGrabStep(void)
{
    const uint16_t *angles = s_grab_angles[s_grab_step];
    return ServoControl_SetAngle('T', angles[0]) &&
           ServoControl_SetAngle('G', angles[1]) &&
           ServoControl_SetAngle('B', angles[2]);
}

'''+s[a:]
    s=s.replace("    s_last_move_sequence = 0U;", "    s_last_move_sequence = 0U;\n    s_grab_active = false;\n    s_grab_step = 0U;\n    s_last_grab_sequence = 0U;")
    s=s.replace('void RobotControl_Tick(void)\n{','''void RobotControl_Tick(void)
{
    if (s_grab_active &&
        (uint32_t)(HAL_GetTick() - s_grab_tick) >= ROBOT_GRAB_STEP_MS)
    {
        if (++s_grab_step >= sizeof(s_grab_angles) / sizeof(s_grab_angles[0]))
            s_grab_active = false;
        else
        {
            s_grab_active = RobotControl_ApplyGrabStep();
            s_grab_tick = HAL_GetTick();
        }
    }''')
    s=s.replace("    case 'S':\n", "    case 'A': /* 固定抓取状态 */\n    case 'Z': /* 取消抓取，保持当前舵机角度 */\n    case 'S':\n")
    s=s.replace('''        if (RobotControl_ParseServo(frame, &sequence, &direction, &angle))
            (void)ServoControl_SetAngle(direction, angle);''','''        if (RobotControl_ParseServo(frame, &sequence, &direction, &angle)) {
            s_grab_active = false; /* 手动调角优先，取消自动后续步骤。 */
            (void)ServoControl_SetAngle(direction, angle);
        }''')
    s=s.replace('    if (!RobotControl_Parse(frame, &sequence, &direction)) return;','''    if (!RobotControl_Parse(frame, &sequence, &direction)) return;
    if (direction == 'Z') { s_grab_active = false; return; }
    if (direction == 'A') {
        if (sequence == 0U || sequence == s_last_grab_sequence || s_grab_active) return;
        s_last_grab_sequence = sequence;
        s_grab_step = 0U;
        s_grab_active = RobotControl_ApplyGrabStep();
        s_grab_tick = HAL_GetTick();
        return;
    }''')
    return s
edit(Path('stm32h743/zhukong/Core/Src/robot_control.c'),stm)
def esp(s):
    s=s.replace("    case 'S':", "    case 'A': /* 抓取固定状态 */\n    case 'Z': /* 取消自动舵机步骤 */\n    case 'S':")
    s=s.replace('    (void)send_to_stm32("CMD,0,S\\n");','    (void)send_to_stm32("CMD,0,Z\\n");\n    (void)send_to_stm32("CMD,0,S\\n");')
    return s
edit(Path('esp32s3/hello_world/main/robot_remote_main.c'),esp)
edit(Path('application/app/src/main/java/com/example/app/RobotTcpClient.kt'),lambda s:s.replace('direction !in "SUDHECQ"','direction !in "SUDHECQAZ"'))
def app(s):
    s=s.replace('                    onServoChange = ::queueServoAngle,', '''                    onGrab = { cancelPendingServos(); robotClient.sendMotion('A') },
                    onCancelGrab = { robotClient.sendMotion('Z') },
                    onServoChange = ::queueServoAngle,''')
    s=s.replace('onDisconnect = { stopMotion();', "onDisconnect = { robotClient.sendMotion('Z'); stopMotion();")
    s=s.replace('        super.onStop()','        super.onStop()\n        robotClient.sendMotion(\'Z\')')
    s=s.replace('    onServoChange: (Char, Int) -> Unit,','    onGrab: () -> Unit,\n    onCancelGrab: () -> Unit,\n    onServoChange: (Char, Int) -> Unit,')
    s=s.replace('        Text("舵机角度（首次拖动后生效）"', '''        Text("固定状态：抓取（5 步，每步间隔 1 秒，无到位回包）")
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Button(onClick = onGrab, enabled = connected) { Text("执行抓取") }
            Button(onClick = onCancelGrab, enabled = connected) { Text("取消抓取") }
        }
        Text("取消或手动调角会终止后续步骤，保持当前姿态。")
        Text("舵机角度（首次拖动后生效）"''')
    s=s.replace('            onServoChange = { _, _ -> },','            onGrab = {},\n            onCancelGrab = {},\n            onServoChange = { _, _ -> },')
    return s
edit(Path('application/app/src/main/java/com/example/app/MainActivity.kt'),app)
