from pathlib import Path
import re
root=Path('F:/GONGXUNSAIPROJ/yundong_part')
def edit(name, fn):
    p=root/name
    p.write_text(fn(p.read_text(encoding='utf-8')),encoding='utf-8')
def cut(s,start,end):
    a=s.index(start); b=s.index(end,a); return s[:a]+s[b:]

def client(s):
    s=s.replace('import java.io.BufferedReader\n','').replace('import java.io.InputStreamReader\n','')
    s=s.replace('import java.util.concurrent.ScheduledFuture\n','').replace('import java.util.concurrent.TimeUnit\n','')
    s=s.replace('/** Socket traffic and execution confirmation never depend on the Android UI looper. */','/** One-way commands; no reply, heartbeat, or execution-confirmation timer. */')
    s=s.replace('Executors.newSingleThreadScheduledExecutor()','Executors.newSingleThreadExecutor()')
    s=cut(s,'    private var moveConfirmation:', '    fun connect(')
    s=s.replace('                val reader = BufferedReader(InputStreamReader(newSocket.getInputStream(), Charsets.US_ASCII))','                val input = newSocket.getInputStream()')
    s=s.replace('                    val line = reader.readLine() ?: break\n                    handleNetworkMessage(line)\n                    onMessage(line)','                    // Observe EOF/disconnection only; legacy replies are discarded.\n                    if (input.read() == -1) break')
    s=s.replace('                        cancelMoveConfirmationLocked()\n','').replace('            cancelMoveConfirmationLocked()\n','')
    a=s.index('    fun sendMove('); b=s.index('    fun sendMotion(',a)
    s=s[:a]+'''    fun sendMove(move: DistanceMove): Long {
        val id = nextSequence()
        sendFrame(move.frame(id))
        return id
    }

'''+s[b:]
    s=s.replace("        if (direction == 'S') cancelMoveConfirmation()\n",'')
    s=s.replace('    fun sendDiagnostic() = sendFrame("DIAG,${nextSequence()}\\n")\n\n','')
    return s
edit(Path('application/app/src/main/java/com/example/app/RobotTcpClient.kt'),client)

def activity(s):
    s=re.sub(r'^.*private var (activeMoveSequence|lastMessage|lastStmMessage) .*\n','',s,flags=re.M)
    s=re.sub(r'^.*cancelMoveConfirmation\(\)\s*\n','',s,flags=re.M)
    s=re.sub(r'^.*activeMoveSequence = null\n','',s,flags=re.M)
    s=s.replace('moveStatus = "已断开，底盘自动停车"','moveStatus = "已断开，无法确认底盘状态"')
    a=s.index('                    if (!message.startsWith("TX,")'); b=s.index('                    communicationTrace =',a)
    s=s[:a]+'''                    val command = message.removePrefix("TX,")
                    val entry = "> $command"
                    if (command.startsWith("MOVE,")) {
                        val mm = command.split(',').getOrNull(3)
                        moveStatus = "已发送 $mm mm；无回包，请等待小车停稳后再发下一条"
                    }
'''+s[b:]
    s=s.replace('                    handleMoveMessage(message)\n','')
    for token in ['lastMessage = lastMessage,','lastStmMessage = lastStmMessage,','lastStmMessage = "尚未收到 STM32 回包"','onDiagnose = { robotClient.sendDiagnostic() },']:
        s=re.sub(r'^.*'+re.escape(token)+r'.*\n','',s,flags=re.M)
    s=s.replace('moveRunning = activeMoveSequence != null,','moveRunning = false,')
    a=s.index('    private fun startDistanceMove('); b=s.index('    private fun stopMotion()',a)
    s=s[:a]+'''    private fun startDistanceMove(move: DistanceMove) {
        if (!isConnected) return
        robotClient.sendMove(move)
        moveStatus = "正在提交 ${move.distanceMm} mm 命令"
    }

'''+s[b:]
    # Remove the old cancellation helper left without a signature/body call.
    s=s.replace('        robotClient.cancelMoveConfirmation()\n','')
    s=s.replace('    }\n\n    }\n\n    private fun startLift','    }\n\n    private fun startLift')
    s=re.sub(r'^\s*(lastMessage: String,|lastStmMessage: String,|onDiagnose: \(\) -> Unit,)\n','\n',s,flags=re.M)
    a=s.index('        Button(onClick = onDiagnose'); b=s.index('\n    }\n}\n',a)
    s=s[:a]+s[b:]
    s=s.replace('最近通讯记录（可长按复制）','最近发送记录（可长按复制）')
    s=s.replace('Text("选择方向和距离，点击执行；左/右为平移。")','Text("选择方向和距离，点击执行；左/右为平移。无回包，请停稳后再发下一条。")')
    s=re.sub(r'^.*(lastMessage = "等待连接 ESP32-S3",|lastStmMessage = "尚未收到 STM32 回包",|onDiagnose = \{\},).*\n','',s,flags=re.M)
    return s
edit(Path('application/app/src/main/java/com/example/app/MainActivity.kt'),activity)

def esp(s):
    s=s.replace('#include "../../../shared/robot_line_decoder.h"\n','')
    s=s.replace('static uint32_t current_move_sequence;\n','').replace('static motion_t current_motion = MOTION_STOP;\n','')
    s=cut(s,'/* 只更新底盘状态','/* 初始化桥接串口')
    s=cut(s,'/* TCP send 可能','/* 校验 CMD,')
    s=cut(s,'/* DIAG,seq','/* 在同一个循环内处理一个 TCP 客户端')
    a=s.index('/* 在同一个循环内处理一个 TCP 客户端')
    handler='''/* 单向命令：非法帧直接丢弃，UART 写失败则结束连接并请求停车。
 * ESP32 不维护底盘 BUSY 状态，由 STM32 在本地判断是否接受新 MOVE。 */
static bool handle_frame(const char *frame, int64_t *last_lift_us,
                         int64_t *last_fore_aft_us)
{
    uint32_t sequence;
    motion_t motion;
    if (strncmp(frame, "MOVE,", 5U) == 0) {
        RobotDistanceCommand command;
        if (!RobotProtocol_ParseMove(frame, &command) ||
            RobotProtocol_MovePulses(&command) == 0U) return true;
    } else if (strncmp(frame, "SERVO,", 6U) == 0) {
        char channel;
        uint16_t angle;
        if (!parse_servo(frame, &sequence, &channel, &angle)) return true;
    } else {
        if (!parse_command(frame, &sequence, &motion)) return true;
        char command[MAX_FRAME_LENGTH + 2];
        snprintf(command, sizeof(command), "%s\\n", frame);
        if (!send_to_stm32(command)) return false;
        if (motion == LIFT_UP || motion == LIFT_DOWN || motion == LIFT_HOLD) {
            current_lift = motion;
            *last_lift_us = esp_timer_get_time();
        } else if (motion == ARM_FORWARD || motion == ARM_BACKWARD ||
                   motion == ARM_FORE_AFT_HOLD) {
            current_fore_aft = motion;
            *last_fore_aft_us = esp_timer_get_time();
        }
        return true;
    }
    char command[MAX_FRAME_LENGTH + 2];
    snprintf(command, sizeof(command), "%s\\n", frame);
    return send_to_stm32(command);
}

'''
    s=s[:a]+handler+s[a:]
    s=s.replace('回包转发、命令组帧、超时停车','命令组帧和机械臂超时停车')
    s=re.sub(r'^.*set_motion\(MOTION_STOP\);\n','',s,flags=re.M)
    s=s.replace('；HELLO 为握手消息','')
    s=re.sub(r'    if \(!send_all\(socket_fd, "HELLO,1\\n"\)\) \{\n        return;\n    }\n','',s)
    s=s.replace('    RobotLineDecoder stm32_decoder = {0};\n','')
    s=re.sub(r'        if \(!forward_stm32_messages\(socket_fd, &stm32_decoder\)\) \{\n            break;\n        }\n\n','',s)
    s=s.replace('                    if (dropping_oversized_frame) {\n                        if (!send_all(socket_fd, "ERR,0,TOO_LONG\\n")) {\n                            goto disconnected;\n                        }\n                    } else {','                    if (!dropping_oversized_frame) {')
    s=s.replace('handle_frame(socket_fd, frame,','handle_frame(frame,')
    s=re.sub(r'            if \(!send_all\(socket_fd, "EVENT,[^"\n]+"\)\) \{\n                break;\n            }\n','',s)
    s=s.replace('/* 本地状态表示已转发的运动请求，不是电机实际位置反馈。\n * 底盘序号用于匹配完成回包；机械臂两轴独立维护运动状态。 */','/* 机械臂两轴独立维护续期状态；底盘只转发命令，不等待回包。 */')
    s=s.replace('STM32 --UART1-->','STM32 --UART1-->')
    return s
edit(Path('esp32s3/hello_world/main/robot_remote_main.c'),esp)

def stm(s):
    s=s.replace('#include <stdio.h>\n','')
    for line in ['#define ROBOT_UART_TX_TIMEOUT_MS       20U','static uint32_t s_move_sequence;','static volatile uint32_t s_rx_error_count;','static uint32_t s_tx_error_count;','static void RobotControl_Send(const char *message);','static bool RobotControl_ParseDiag(const char *frame, uint32_t *sequence);']:
        s=s.replace(line+'\n','')
    s=re.sub(r'^.*(s_move_sequence = 0U;|s_rx_error_count = 0U;|s_tx_error_count = 0U;|\+\+s_rx_error_count;).*\n','',s,flags=re.M)
    s=cut(s,'/* 原始文本经 USART3','static bool RobotControl_Parse(const char *frame, uint32_t *sequence,\n                               char *direction)\n{')
    s=cut(s,'static bool RobotControl_ParseDiag(const char *frame, uint32_t *sequence)\n{','static HAL_StatusTypeDef RobotControl_EnableMotors(void)\n{')
    # Remove reply-only formatting statements and sends in Process/Tick/Init.
    s=re.sub(r'\s*snprintf\(response,.*?\);','',s,flags=re.S)
    s=re.sub(r'^\s*RobotControl_Send\([^;]*\);\n','\n',s,flags=re.M)
    s=s.replace('        char response[64];\n','')
    a=s.index('/* 接收的是去掉换行的完整文本'); s=s[:a]+'''/* 单向命令：格式错误、重复或忙碌任务直接忽略，不向 ESP32 回包。
 * 电机底层 ACK、到位查询、故障停车和停车重试仍保留。 */
static void RobotControl_HandleFrame(const char *frame)
{
    uint32_t sequence;
    char direction;
    uint16_t angle;
    if (strncmp(frame, "MOVE,", 5U) == 0) {
        RobotDistanceCommand command;
        if (!RobotProtocol_ParseMove(frame, &command) ||
            RobotProtocol_MovePulses(&command) == 0U ||
            command.sequence == s_last_move_sequence ||
            s_motion != 'S' || s_stop_pending || !s_wheel_uart_ready) return;
        s_last_move_sequence = command.sequence;
        if (RobotControl_ApplyDistance(&command) != HAL_OK) {
            s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        } else {
            s_motion = command.direction;
            s_move_started_tick = HAL_GetTick();
            s_status_tick = s_move_started_tick;
            s_status_wheel = 0U;
            s_reached_mask = 0U;
        }
        return;
    }
    if (strncmp(frame, "SERVO,", 6U) == 0) {
        if (RobotControl_ParseServo(frame, &sequence, &direction, &angle))
            (void)ServoControl_SetAngle(direction, angle);
        return;
    }
    if (!RobotControl_Parse(frame, &sequence, &direction)) return;
    if (direction == 'U' || direction == 'D' || direction == 'H') {
        s_last_lift_tick = HAL_GetTick();
        if (direction == s_lift_motion && s_lift_command_ok) return;
        if ((direction == 'H' ? LiftMotor_Stop() : LiftMotor_Move(direction)) != HAL_OK) {
            s_lift_motion = direction;
            s_lift_command_ok = false;
            if (LiftMotor_Stop() == HAL_OK) s_lift_motion = 'H';
            return;
        }
        s_lift_motion = direction;
        s_lift_command_ok = true;
        return;
    }
    if (direction == 'E' || direction == 'C' || direction == 'Q') {
        s_last_fore_aft_tick = HAL_GetTick();
        if (direction == s_fore_aft_motion && s_fore_aft_command_ok) return;
        if ((direction == 'Q' ? ArmMotor_StopForeAft() : ArmMotor_MoveForeAft(direction)) != HAL_OK) {
            s_fore_aft_motion = direction;
            s_fore_aft_command_ok = false;
            if (ArmMotor_StopForeAft() == HAL_OK) s_fore_aft_motion = 'Q';
            return;
        }
        s_fore_aft_motion = direction;
        s_fore_aft_command_ok = true;
        return;
    }
    s_stop_pending = RobotControl_StopMotors() != HAL_OK;
    if (!s_stop_pending) s_motion = 'S';
}
'''
    s=s.replace('清空任务和接收状态，开启 USART3 中断接收，并发送 READY。\n * 此处不启动电机；是否收到 READY 还取决于 ESP32 当时是否连接并转发。','清空任务和接收状态，开启 USART3 中断接收；不启动电机、不发送回包。')
    s=s.replace('一轮查询完成且四轮均到位才回 DONE；不能只看一个电机。','一轮查询完成且四轮均到位才结束任务；不能只看一个电机。')
    return s
edit(Path('stm32h743/zhukong/Core/Src/robot_control.c'),stm)
print('Removed application replies and confirmation timeout; retained motor ACK/status checks.')
