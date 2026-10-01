from pathlib import Path
root=Path('F:/GONGXUNSAIPROJ/yundong_part')
def edit(name, fn):
 p=root/name; p.write_text(fn(p.read_text(encoding='utf-8')),encoding='utf-8')
def control(s):
 s=s.replace('#include "servo_control.h"', '#include "servo_control.h"\n#include "camera_link.h"')
 s=s.replace('static bool s_stop_pending;', 'static bool s_stop_pending;\nstatic bool s_wheel_move_reached;')
 a=s.index('static bool RobotControl_Parse(')
 s=s[:a]+'''/* 摄像头持续输出坐标。每次移动等待四轮到位、静置 500 ms 后重新定位。 */
#define ROBOT_ALIGN_SAMPLE_MS       3000U
#define ROBOT_ALIGN_TOTAL_MS       60000U
#define ROBOT_ALIGN_SETTLE_MS        500U
#define ROBOT_ALIGN_MAX_MOVES         20U
#define ROBOT_ALIGN_RPM               20U
typedef enum { ALIGN_WAIT, ALIGN_MOVING, ALIGN_SETTLE } AlignPhase;
static bool s_align_active;
static AlignPhase s_align_phase;
static uint32_t s_align_start_tick, s_align_phase_tick, s_last_align_sequence;
static uint32_t s_align_forward_ppm, s_align_lateral_ppm;
static unsigned s_align_moves, s_align_centered;
static HAL_StatusTypeDef RobotControl_ApplyDistance(const RobotDistanceCommand *command);
static HAL_StatusTypeDef RobotControl_StopMotors(void);

static void RobotControl_CancelAlignment(void)
{
    if (s_align_active && s_align_phase == ALIGN_MOVING) {
        s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        if (!s_stop_pending) s_motion = 'S';
    }
    s_align_active = false;
    CameraLink_Discard();
}
static void RobotControl_TickAlignment(void)
{
    if (!s_align_active) return;
    uint32_t now = HAL_GetTick();
    if (s_stop_pending || (uint32_t)(now - s_align_start_tick) >= ROBOT_ALIGN_TOTAL_MS)
    { RobotControl_CancelAlignment(); return; }
    if (s_align_phase == ALIGN_MOVING) {
        CameraCenter ignored; (void)CameraLink_TakeCenter(&ignored);
        if (s_motion == 'S') {
            if (!s_wheel_move_reached) { RobotControl_CancelAlignment(); return; }
            s_align_phase = ALIGN_SETTLE; s_align_phase_tick = now;
        }
        return;
    }
    if (s_align_phase == ALIGN_SETTLE) {
        if ((uint32_t)(now - s_align_phase_tick) >= ROBOT_ALIGN_SETTLE_MS) {
            CameraLink_Discard(); s_align_phase = ALIGN_WAIT; s_align_phase_tick = now;
        }
        return;
    }
    if ((uint32_t)(now - s_align_phase_tick) >= ROBOT_ALIGN_SAMPLE_MS)
    { RobotControl_CancelAlignment(); return; }
    CameraCenter center;
    if (!CameraLink_TakeCenter(&center)) return;
    s_align_phase_tick = now;
    char direction; uint32_t mm;
    if (!CameraProtocol_Correction(&center, &direction, &mm)) {
        if (++s_align_centered >= 2U) s_align_active = false;
        return;
    }
    s_align_centered = 0U;
    if (s_align_moves >= ROBOT_ALIGN_MAX_MOVES)
    { RobotControl_CancelAlignment(); return; }
    RobotDistanceCommand move = {0U, direction, mm, ROBOT_ALIGN_RPM,
        (direction == 'F' || direction == 'B') ? s_align_forward_ppm : s_align_lateral_ppm};
    s_wheel_move_reached = false;
    if (RobotControl_ApplyDistance(&move) != HAL_OK) {
        s_stop_pending = RobotControl_StopMotors() != HAL_OK;
        if (!s_stop_pending) s_motion = 'S';
        s_align_active = false; return;
    }
    s_motion = direction; s_move_started_tick = HAL_GetTick();
    s_status_tick = s_move_started_tick; s_status_wheel = 0U; s_reached_mask = 0U;
    ++s_align_moves; s_align_phase = ALIGN_MOVING; s_align_phase_tick = s_move_started_tick;
}

'''+s[a:]
 s=s.replace('    s_last_move_sequence = 0U;', '    s_last_move_sequence = 0U;\n    s_wheel_move_reached = false;\n    s_align_active = false;\n    s_last_align_sequence = 0U;')
 s=s.replace('void RobotControl_Process(void)\n{\n', 'void RobotControl_Process(void)\n{\n    CameraLink_Process();\n')
 s=s.replace('void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)\n{', 'void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)\n{\n    if (CameraLink_RxCallback(huart)) return;')
 s=s.replace('void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)\n{', 'void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)\n{\n    if (CameraLink_ErrorCallback(huart)) return;')
 s=s.replace("                s_motion = 'S';\n\n", "                s_motion = 'S';\n                s_wheel_move_reached = true;\n")
 s=s.replace("    if ((s_lift_motion != 'H') &&", "    RobotControl_TickAlignment();\n    if ((s_lift_motion != 'H') &&")
 s=s.replace('static HAL_StatusTypeDef RobotControl_StopMotors(void)\n{', 'static HAL_StatusTypeDef RobotControl_StopMotors(void)\n{\n    s_wheel_move_reached = false;')
 a=s.index('    if (strncmp(frame, "ARM_MOVE,", 9U) == 0)',s.index('static void RobotControl_HandleFrame(const char *frame)\n{'))
 s=s[:a]+'''    if (strncmp(frame, "ALIGN,", 6U) == 0) {
        CameraAlignCommand cmd;
        if (!CameraProtocol_ParseAlign(frame, &cmd) || !CameraLink_Ready() ||
            s_align_active || s_grab_active || s_arm_move_active || s_motion != 'S' ||
            s_stop_pending || !s_wheel_uart_ready || cmd.sequence == s_last_align_sequence) return;
        s_last_align_sequence = cmd.sequence; s_align_active = true; s_align_phase = ALIGN_WAIT;
        s_align_forward_ppm = cmd.forward_ppm; s_align_lateral_ppm = cmd.lateral_ppm;
        s_align_start_tick = s_align_phase_tick = HAL_GetTick();
        s_align_moves = s_align_centered = 0U; CameraLink_Discard(); return;
    }
'''+s[a:]
 s=s.replace('command.sequence == s_last_arm_move_sequence || s_arm_move_active || s_grab_active ||', 'command.sequence == s_last_arm_move_sequence || s_arm_move_active || s_grab_active || s_align_active ||')
 s=s.replace("            s_motion != 'S' || s_stop_pending || !s_wheel_uart_ready) return;", "            s_motion != 'S' || s_align_active || s_stop_pending || !s_wheel_uart_ready) return;")
 s=s.replace('        s_last_move_sequence = command.sequence;', '        s_last_move_sequence = command.sequence;\n        s_wheel_move_reached = false;')
 s=s.replace("if (direction == 'Z') { RobotControl_CancelGrab(); return; }", "if (direction == 'Z') { RobotControl_CancelAlignment(); RobotControl_CancelGrab(); return; }")
 s=s.replace('sequence == s_last_grab_sequence || s_grab_active ||', 'sequence == s_last_grab_sequence || s_grab_active || s_align_active ||')
 s=s.replace('    s_stop_pending = RobotControl_StopMotors() != HAL_OK;\n    if (!s_stop_pending) s_motion', '    s_align_active = false;\n    CameraLink_Discard();\n    s_stop_pending = RobotControl_StopMotors() != HAL_OK;\n    if (!s_stop_pending) s_motion')
 return s
edit(Path('stm32h743/zhukong/Core/Src/robot_control.c'),control)
def main(s):
 s=s.replace('#include "servo_control.h"', '#include "servo_control.h"\n#include "camera_link.h"')
 s=s.replace('/* USER CODE BEGIN PV */', '/* USER CODE BEGIN PV */\nUART_HandleTypeDef huart4;')
 s=s.replace('/* USER CODE BEGIN 0 */','''/* USER CODE BEGIN 0 */
/* UART4 PC10 TX / PC11 RX; camera ttyS2, 115200 8N1. No request TX in current protocol. */
static bool CameraUart_Init(void)
{
    RCC_PeriphCLKInitTypeDef clocks = {0};
    clocks.PeriphClkInitTypeDef = 0; /* placeholder removed by script */
    clocks.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    clocks.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clocks) != HAL_OK) return false;
    __HAL_RCC_UART4_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_LOW; gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);
    huart4.Instance = UART4; huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B; huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE; huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    if (HAL_UART_Init(&huart4) != HAL_OK) return false;
    return HAL_UARTEx_DisableFifoMode(&huart4) == HAL_OK;
}
''').replace('    clocks.PeriphClkInitTypeDef = 0; /* placeholder removed by script */\n','')
 s=s.replace('  RobotControl_Init(&huart3, wheel_uart_ready);', '  RobotControl_Init(&huart3, wheel_uart_ready);\n  CameraLink_Init(CameraUart_Init() ? &huart4 : NULL);')
 return s
edit(Path('stm32h743/zhukong/Core/Src/main.c'),main)
edit(Path('stm32h743/zhukong/Core/Src/stm32h7xx_it.c'),lambda s:s.replace('extern UART_HandleTypeDef huart3;', 'extern UART_HandleTypeDef huart3;\nextern UART_HandleTypeDef huart4;\nvoid UART4_IRQHandler(void) { HAL_UART_IRQHandler(&huart4); }'))
edit(Path('stm32h743/zhukong/Core/Inc/stm32h7xx_it.h'),lambda s:s.replace('void USART3_IRQHandler(void);', 'void USART3_IRQHandler(void);\nvoid UART4_IRQHandler(void);'))
edit(Path('stm32h743/zhukong/CMakeLists.txt'),lambda s:s.replace('    Core/Src/servo_control.c', '    Core/Src/servo_control.c\n    Core/Src/camera_link.c'))
def esp(s):
 s=s.replace('#include "../../../shared/robot_distance_protocol.h"', '#include "../../../shared/robot_distance_protocol.h"\n#include "../../../shared/camera_position_protocol.h"')
 s=s.replace('    if (strncmp(frame, "ARM_MOVE,", 9U) == 0) {', '''    if (strncmp(frame, "ALIGN,", 6U) == 0) {
        CameraAlignCommand command;
        if (!CameraProtocol_ParseAlign(frame, &command)) return true;
    } else if (strncmp(frame, "ARM_MOVE,", 9U) == 0) {''')
 return s
edit(Path('esp32s3/hello_world/main/robot_remote_main.c'),esp)
def client(s):
 a=s.index('    fun sendArmDistance(')
 return s[:a]+'''    fun sendAlignment(forwardPpm: Int, lateralPpm: Int) {
        if (forwardPpm !in 1..1000000 || lateralPpm !in 1..1000000) return
        sendFrame("ALIGN,${nextSequence()},$forwardPpm,$lateralPpm\\n")
    }

'''+s[a:]
edit(Path('application/app/src/main/java/com/example/app/RobotTcpClient.kt'),client)
def app(s):
 s=s.replace('                    onMove = ::startDistanceMove,', '''                    onMove = ::startDistanceMove,
                    onAlign = { forward, lateral ->
                        robotClient.sendAlignment(forward, lateral)
                        moveStatus = "已提交位置修正；停止底盘可取消，无到位回包"
                    },''')
 s=s.replace('    onMove: (DistanceMove) -> Unit,', '    onMove: (DistanceMove) -> Unit,\n    onAlign: (Int, Int) -> Unit,')
 s=s.replace('DistanceMovePanel(connected, moveRunning, moveStatus, onMove, onStopMotion)', 'DistanceMovePanel(connected, moveRunning, moveStatus, onMove, onAlign, onStopMotion)')
 s=s.replace('    ) { Text(if (running) "移动中…" else "执行定距移动") }', '''    ) { Text(if (running) "移动中…" else "执行定距移动") }
    Button(
        onClick = {
            val forward = DistanceMove.fromInputs('F', "1000", "20", diameter, wheelPulses, forwardCorrection)
            val lateral = DistanceMove.fromInputs('L', "1000", "20", diameter, wheelPulses, lateralCorrection)
            if (forward == null || lateral == null) error = "请检查轮径、细分和距离修正系数"
            else { error = null; onAlign(forward.pulsesPerMetre, lateral.pulsesPerMetre) }
        }, enabled = connected && !running, modifier = Modifier.fillMaxWidth(),
    ) { Text("执行位置修正") }
    Text("绿色物块对准图像中心 (256,160)，比例 1.090819 mm/px。右→前、下→左；停止底盘可取消。")''')
 s=s.replace('            onMove = {},', '            onMove = {},\n            onAlign = { _, _ -> },')
 return s
edit(Path('application/app/src/main/java/com/example/app/MainActivity.kt'),app)
