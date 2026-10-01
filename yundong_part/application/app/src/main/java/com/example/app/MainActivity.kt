package com.example.app

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.background
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.text.selection.SelectionContainer
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.example.app.ui.theme.AppTheme
import kotlin.math.roundToInt

private const val COMMAND_INTERVAL_MS = 50L

class MainActivity : ComponentActivity() {
    private val commandHandler = Handler(Looper.getMainLooper())
    private var moveStatus by mutableStateOf("等待定距移动")
    private var repeatedLiftCommand: Runnable? = null
    private var activeLiftDirection: Char? = null
    private val pendingServoValues = mutableMapOf<Char, Int>()
    private val pendingServoTasks = mutableMapOf<Char, Runnable>()
    private lateinit var robotClient: RobotTcpClient

    private var connectionText by mutableStateOf("未连接")
    private var communicationTrace by mutableStateOf("")
    private var isConnected by mutableStateOf(false)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        robotClient = RobotTcpClient(
            onConnectionChanged = { connected, message ->
                runOnUiThread {
                    isConnected = connected
                    connectionText = message
                    if (!connected) {
                        moveStatus = "已断开，无法确认底盘状态"
                        cancelRepeatedLiftCommand()
                        cancelPendingServos()
                    }
                }
            },
            onMessage = { message ->
                runOnUiThread {
                    val command = message.removePrefix("TX,")
                    val entry = "> $command"
                    if (command.startsWith("MOVE,")) {
                        val mm = command.split(',').getOrNull(3)
                        moveStatus = "已发送 $mm mm；无回包，请等待小车停稳后再发下一条"
                    }
                    communicationTrace = (communicationTrace.lines().filter { it.isNotBlank() } + entry)
                        .takeLast(16).joinToString("\n")
                }
            },
        )

        enableEdgeToEdge()
        setContent {
            AppTheme {
                RemoteControlScreen(
                    connected = isConnected,
                    connectionText = connectionText,
                    communicationTrace = communicationTrace,
                    onConnect = { host, port ->
                        communicationTrace = ""
                        robotClient.connect(host, port)
                    },
                    onDisconnect = { robotClient.sendMotion('Z'); stopMotion(); stopLift(); stopForeAft(); robotClient.disconnect() },
                    moveRunning = false,
                    moveStatus = moveStatus,
                    onMove = ::startDistanceMove,
                    onAlign = { forward, lateral ->
                        robotClient.sendAlignment(forward, lateral)
                        moveStatus = "已提交位置修正；停止底盘可取消，无到位回包"
                    },
                    onStopMotion = ::stopMotion,
                    onStartLift = ::startLift,
                    onStopLift = ::stopLift,
                    onLiftDistance = { direction, distance, rpm, pulses ->
                        stopLift()
                        robotClient.sendLiftDistance(direction, distance, rpm, pulses)
                    },
                    onStopForeAft = ::stopForeAft,
                    onArmDistance = { direction, distance, rpm, pulses ->
                        robotClient.sendArmDistance(direction, distance, rpm, pulses)
                    },
                    onArmPose = { theta, r, z, rpm, pulses ->
                        cancelPendingServos()
                        robotClient.sendArmPose(theta, r, z, rpm, pulses)
                    },
                    onHome = { cancelPendingServos(); robotClient.sendMotion('O') },
                    onInitializeOrigin = { cancelPendingServos(); robotClient.sendMotion('I') },
                    onStopArm = { robotClient.sendMotion('Z'); stopLift(); stopForeAft() },
                    onGrab = { cancelPendingServos(); robotClient.sendMotion('A') },
                    onCancelGrab = { robotClient.sendMotion('Z') },
                    onServoChange = ::queueServoAngle,
                    onServoFinished = ::sendServoAngle,
                )
            }
        }
    }

    override fun onStop() {
        super.onStop()
        robotClient.sendMotion('Z')
        stopMotion()
        stopLift()
        stopForeAft()
        cancelPendingServos()
    }

    override fun onDestroy() {
        stopMotion()
        stopLift()
        stopForeAft()
        cancelPendingServos()
        robotClient.close()
        super.onDestroy()
    }

    private fun startDistanceMove(move: DistanceMove) {
        if (!isConnected) return
        robotClient.sendMove(move)
        moveStatus = "正在提交 ${move.distanceMm} mm 命令"
    }

    private fun stopMotion() {
        moveStatus = "已请求底盘停车"
        robotClient.sendMotion('S')
    }

    private fun startLift(direction: Char) {
        if (!isConnected) return
        cancelRepeatedLiftCommand()
        activeLiftDirection = direction
        robotClient.sendMotion(direction)
        repeatedLiftCommand = object : Runnable {
            override fun run() {
                if (activeLiftDirection == direction && isConnected) {
                    robotClient.sendMotion(direction)
                    commandHandler.postDelayed(this, COMMAND_INTERVAL_MS)
                }
            }
        }.also { commandHandler.postDelayed(it, COMMAND_INTERVAL_MS) }
    }

    private fun stopLift() {
        cancelRepeatedLiftCommand()
        activeLiftDirection = null
        robotClient.sendMotion('H')
    }

    private fun cancelRepeatedLiftCommand() {
        repeatedLiftCommand?.let(commandHandler::removeCallbacks)
        repeatedLiftCommand = null
    }

    private fun stopForeAft() {
        robotClient.sendMotion('Q')
    }

    private fun queueServoAngle(channel: Char, angle: Int) {
        if (!isConnected) return
        pendingServoValues[channel] = angle
        if (pendingServoTasks.containsKey(channel)) return
        val task = Runnable {
            pendingServoTasks.remove(channel)
            pendingServoValues.remove(channel)?.let { robotClient.sendServo(channel, it) }
        }
        pendingServoTasks[channel] = task
        commandHandler.postDelayed(task, 50L)
    }

    private fun sendServoAngle(channel: Char, angle: Int) {
        pendingServoTasks.remove(channel)?.let(commandHandler::removeCallbacks)
        pendingServoValues.remove(channel)
        if (isConnected) robotClient.sendServo(channel, angle)
    }

    private fun cancelPendingServos() {
        pendingServoTasks.values.forEach(commandHandler::removeCallbacks)
        pendingServoTasks.clear()
        pendingServoValues.clear()
    }
}

@Composable
private fun RemoteControlScreen(
    connected: Boolean,
    connectionText: String,


    communicationTrace: String,
    onConnect: (String, Int) -> Unit,
    onDisconnect: () -> Unit,

    moveRunning: Boolean,
    moveStatus: String,
    onMove: (DistanceMove) -> Unit,
    onAlign: (Int, Int) -> Unit,
    onStopMotion: () -> Unit,
    onStartLift: (Char) -> Unit,
    onStopLift: () -> Unit,
    onLiftDistance: (Char, Int, Int, Int) -> Unit,
    onStopForeAft: () -> Unit,
    onArmDistance: (Char, Int, Int, Int) -> Unit,
    onArmPose: (Int, Int, Int, Int, Int) -> Unit,
    onHome: () -> Unit,
    onInitializeOrigin: () -> Unit,
    onStopArm: () -> Unit,
    onGrab: () -> Unit,
    onCancelGrab: () -> Unit,
    onServoChange: (Char, Int) -> Unit,
    onServoFinished: (Char, Int) -> Unit,
) {
    var host by rememberSaveable { mutableStateOf("192.168.4.1") }
    var portText by rememberSaveable { mutableStateOf("3333") }
    var addressError by remember { mutableStateOf<String?>(null) }

    Column(
        modifier = Modifier.fillMaxSize().verticalScroll(rememberScrollState())
            .padding(horizontal = 20.dp, vertical = 28.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("小车遥控", style = MaterialTheme.typography.headlineMedium, fontWeight = FontWeight.Bold)
        Spacer(Modifier.height(12.dp))
        Text(connectionText, color = if (connected) Color(0xFF16803C) else MaterialTheme.colorScheme.error)
        Spacer(Modifier.height(12.dp))
        OutlinedTextField(
            value = host,
            onValueChange = { host = it },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true,
            label = { Text("ESP32 地址") },
            enabled = !connected,
        )
        Spacer(Modifier.height(8.dp))
        OutlinedTextField(
            value = portText,
            onValueChange = { portText = it },
            modifier = Modifier.fillMaxWidth(),
            singleLine = true,
            label = { Text("端口") },
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            enabled = !connected,
        )
        addressError?.let { Text(it, modifier = Modifier.fillMaxWidth(), color = MaterialTheme.colorScheme.error) }
        Spacer(Modifier.height(8.dp))
        Button(
            onClick = {
                if (connected) {
                    onDisconnect()
                } else {
                    val port = portText.toIntOrNull()
                    if (host.isBlank() || port == null || port !in 1..65535) {
                        addressError = "请输入有效的地址和端口。"
                    } else {
                        addressError = null
                        onConnect(host.trim(), port)
                    }
                }
            },
            modifier = Modifier.fillMaxWidth(),
        ) { Text(if (connected) "断开连接" else "连接 ESP32-S3") }

        Spacer(Modifier.height(20.dp))
        HorizontalDivider()
        Spacer(Modifier.height(20.dp))
        DistanceMovePanel(connected, moveRunning, moveStatus, onMove, onAlign, onStopMotion)
        Spacer(Modifier.height(8.dp))
        Text("最近发送记录（可长按复制）", style = MaterialTheme.typography.labelLarge)
        SelectionContainer {
            Text(communicationTrace.ifBlank { "等待通讯" }, modifier = Modifier.fillMaxWidth(),
                style = MaterialTheme.typography.bodySmall)
        }
        Spacer(Modifier.height(16.dp))
        HorizontalDivider()
        Spacer(Modifier.height(12.dp))
        ArmPosePanel(connected, onArmPose, onInitializeOrigin, onHome, onStopArm)
        HorizontalDivider()
        Spacer(Modifier.height(12.dp))
        Text("固定状态：抓取（8 步）")
        Text("回零→r=110、z=-50→夹紧→回零→基座144°→r=20、z=-40→松爪并回零→基座263°。单位 mm。")
        Text("第 5、8 步基座平滑旋转 3 秒；每轮完成后转盘依次到 120°、240°、0°。两轴到位后继续，无应用回包。")
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Button(onClick = onGrab, enabled = connected) { Text("执行抓取") }
            Button(onClick = onCancelGrab, enabled = connected) { Text("取消抓取") }
        }
        Text("取消或手动调角会终止后续步骤，保持当前姿态。")
        Text("舵机角度（启动姿态：转盘 0°、夹子 15°、基座 263°）", style = MaterialTheme.typography.titleMedium)
        ServoAngleSlider("夹子 · PC8", 'G', 270, connected, onServoChange, onServoFinished)
        ServoAngleSlider("转盘 · PA8", 'T', 270, connected, onServoChange, onServoFinished)
        ServoAngleSlider("基座 · PC6", 'B', 360, connected, onServoChange, onServoFinished)
        Spacer(Modifier.height(16.dp))

    }
}

@Composable
private fun DistanceMovePanel(
    connected: Boolean,
    running: Boolean,
    status: String,
    onMove: (DistanceMove) -> Unit,
    onAlign: (Int, Int) -> Unit,
    onStop: () -> Unit,
) {
    var direction by rememberSaveable { mutableStateOf('F') }
    var distance by rememberSaveable { mutableStateOf("100") }
    var speed by rememberSaveable { mutableStateOf("45") }
    var diameter by rememberSaveable { mutableStateOf("103") }
    var wheelPulses by rememberSaveable { mutableStateOf("3200") }
    var forwardCorrection by rememberSaveable { mutableStateOf("1.0") }
    var lateralCorrection by rememberSaveable { mutableStateOf("1.0") }
    var error by remember { mutableStateOf<String?>(null) }
    Text("底盘定距移动", style = MaterialTheme.typography.titleMedium)
    Text("选择方向和距离，点击执行；左/右为平移。无回包，请停稳后再发下一条。")
    Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
        listOf('F' to "前", 'B' to "后", 'L' to "左", 'R' to "右").forEach { (value, label) ->
            Button(
                onClick = { direction = value },
                enabled = !running,
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (direction == value) MaterialTheme.colorScheme.primary
                    else MaterialTheme.colorScheme.secondaryContainer,
                    contentColor = if (direction == value) MaterialTheme.colorScheme.onPrimary
                    else MaterialTheme.colorScheme.onSecondaryContainer,
                ),
            ) { Text(label) }
        }
    }
    DistanceInput("距离（mm，1–10000）", distance, !running) { distance = it }
    DistanceInput("电机速度（RPM，5–300）", speed, !running) { speed = it }
    Text("距离标定 · 减速比 1:1", style = MaterialTheme.typography.labelLarge)
    DistanceInput("轮径（mm）", diameter, !running, true) { diameter = it }
    DistanceInput("车轮每圈脉冲（默认 1.8° / 16 细分为 3200）", wheelPulses, !running) { wheelPulses = it }
    DistanceInput("前后距离修正系数", forwardCorrection, !running, true) { forwardCorrection = it }
    DistanceInput("左右距离修正系数", lateralCorrection, !running, true) { lateralCorrection = it }
    Text("修正系数默认 1；标定后乘以“设定距离 ÷ 实测距离”。")
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(
        onClick = {
            val correction = if (direction in "LR") lateralCorrection else forwardCorrection
            val move = DistanceMove.fromInputs(direction, distance, speed, diameter, wheelPulses, correction)
            if (move == null) {
                error = "请检查距离、速度及标定参数：轮径 20–500 mm，脉冲 1–1000000，修正系数 0.1–10。"
            } else {
                error = null
                onMove(move)
            }
        },
        enabled = connected && !running,
        modifier = Modifier.fillMaxWidth(),
    ) { Text(if (running) "移动中…" else "执行定距移动") }
    Button(
        onClick = {
            val forward = DistanceMove.fromInputs('F', "1000", "20", diameter, wheelPulses, forwardCorrection)
            val lateral = DistanceMove.fromInputs('L', "1000", "20", diameter, wheelPulses, lateralCorrection)
            if (forward == null || lateral == null) error = "请检查轮径、细分和距离修正系数"
            else { error = null; onAlign(forward.pulsesPerMetre, lateral.pulsesPerMetre) }
        }, enabled = connected && !running, modifier = Modifier.fillMaxWidth(),
    ) { Text("执行位置修正") }
    Text("绿色物块对准图像中心 (256,160)。偏差大于 20 px 时快速修正 50%，接近中心时微调 30%；单步最多 15 mm，停稳 300 ms 后重新识别；停止底盘可取消。")
    Button(
        onClick = onStop,
        enabled = connected,
        modifier = Modifier.fillMaxWidth(),
        colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error),
    ) { Text("停止底盘") }
    Text(status, modifier = Modifier.fillMaxWidth())
}

@Composable
private fun DistanceInput(
    label: String, value: String, enabled: Boolean, decimal: Boolean = false,
    onChange: (String) -> Unit,
) {
    OutlinedTextField(
        value = value,
        onValueChange = onChange,
        label = { Text(label) },
        singleLine = true,
        enabled = enabled,
        modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp),
        keyboardOptions = KeyboardOptions(keyboardType = if (decimal) KeyboardType.Decimal else KeyboardType.Number),
    )
}

@Composable
private fun ServoAngleSlider(
    label: String,
    channel: Char,
    maximum: Int,
    enabled: Boolean,
    onChange: (Char, Int) -> Unit,
    onFinished: (Char, Int) -> Unit,
) {
    var angle by remember(channel) { mutableStateOf(when (channel) { 'G' -> 15; 'T' -> 0; else -> 263 }) }
    Column(modifier = Modifier.fillMaxWidth()) {
        Text("$label：$angle°")
        Slider(
            value = angle.toFloat(),
            onValueChange = {
                val next = it.roundToInt().coerceIn(0, maximum)
                if (next != angle) {
                    angle = next
                    onChange(channel, next)
                }
            },
            onValueChangeFinished = { onFinished(channel, angle) },
            valueRange = 0f..maximum.toFloat(),
            enabled = enabled,
        )
    }
}

@Composable
private fun ArmPosePanel(
    connected: Boolean,
    onMove: (Int, Int, Int, Int, Int) -> Unit,
    onInitialize: () -> Unit,
    onHome: () -> Unit,
    onStop: () -> Unit,
) {
    var theta by rememberSaveable { mutableStateOf("0") }
    var r by rememberSaveable { mutableStateOf("0") }
    var z by rememberSaveable { mutableStateOf("0") }
    var rpm by rememberSaveable { mutableStateOf("5") }
    var pulses by rememberSaveable { mutableStateOf("3200") }
    var note by rememberSaveable { mutableStateOf("等待绝对位置指令") }
    Text("机械臂：柱坐标绝对位置", style = MaterialTheme.typography.titleMedium)
    Text("先点击启动原点状态，将当前两轴位置记为 r=0、z=0；r 正值向前，z 正值上升。")
    Button(onClick = { onInitialize(); note = "已请求原点初始化：转盘0°、爪子15°、基座263°，记录两轴当前位置" },
        enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("启动原点状态") }
    DistanceInput("转盘 θ（度，0–270）", theta, connected) { theta = it }
    OutlinedTextField(value = r, onValueChange = { r = it }, label = { Text("前伸 r（mm，-1000–1000）") },
        enabled = connected, singleLine = true, modifier = Modifier.fillMaxWidth())
    OutlinedTextField(value = z, onValueChange = { z = it }, label = { Text("升降 z（mm，-400–400）") },
        enabled = connected, singleLine = true, modifier = Modifier.fillMaxWidth())
    DistanceInput("速度（RPM，5–60）", rpm, connected) { rpm = it }
    DistanceInput("两轴每圈脉冲（须匹配细分，默认 3200）", pulses, connected) { pulses = it }
    Button(onClick = {
        val angle = theta.toIntOrNull(); val radial = r.toIntOrNull(); val height = z.toIntOrNull()
        val speed = rpm.toIntOrNull(); val ppr = pulses.toIntOrNull()
        if (angle == null || angle !in 0..270 || radial == null || radial !in -1000..1000 ||
            height == null || height !in -400..400 || speed == null || speed !in 5..60 ||
            ppr == null || ppr !in 200..51200) note = "请输入范围内的 θ、r、z、速度和每圈脉冲"
        else { onMove(angle, radial, height, speed, ppr); note = "已提交绝对目标：θ=$angle°，r=$radial mm，z=$height mm" }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("执行绝对位置") }
    Button(onClick = { onHome(); note = "已请求返回启动原点及初始舵机姿态" },
        enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("返回原点状态") }
    Button(onClick = { onStop(); note = "已请求停止机械臂" },
        enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("停止机械臂") }
    Text(note)
    Text("原点初始化不移动两步进轴；每次 STM32 重启后需初始化一次，重复点击不重设原点。返回原点是回到已记录位置，无到位回包。")
    Spacer(Modifier.height(16.dp))
}

@Composable
private fun LiftDistancePanel(
    connected: Boolean,
    onMove: (Char, Int, Int, Int) -> Unit,
    onStop: () -> Unit,
) {
    var distance by rememberSaveable { mutableStateOf("10") }
    var rpm by rememberSaveable { mutableStateOf("5") }
    var pulses by rememberSaveable { mutableStateOf("3200") }
    var note by rememberSaveable { mutableStateOf("等待升降定距移动") }
    Text("机械臂升降：定距移动", style = MaterialTheme.typography.titleMedium)
    Text("实测标定：电机一圈 360° = 40 mm（4 cm），每毫米对应 9°。")
    DistanceInput("移动距离（mm，1–400）", distance, connected) { distance = it }
    DistanceInput("速度（RPM，5–60）", rpm, connected) { rpm = it }
    DistanceInput("电机每圈脉冲（默认 3200，须匹配细分）", pulses, connected) { pulses = it }
    fun move(direction: Char) {
        val mm = distance.toIntOrNull()
        val speed = rpm.toIntOrNull()
        val perRev = pulses.toIntOrNull()
        if (mm == null || mm !in 1..400 || speed == null || speed !in 5..60 ||
            perRev == null || perRev !in 200..51200) {
            note = "请输入有效距离、速度和每圈脉冲"
            return
        }
        onMove(direction, mm, speed, perRev)
        note = "已提交${if (direction == 'U') "上升" else "下降"} $mm mm（电机转角 ${mm * 9}°）"
    }
    Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        Button(onClick = { move('U') }, enabled = connected) { Text("上升") }
        Button(onClick = { move('D') }, enabled = connected) { Text("下降") }
    }
    Button(onClick = { onStop(); note = "已请求停止升降" },
        enabled = connected, modifier = Modifier.fillMaxWidth()) {
        Text("停止升降")
    }
    Text(note)
    Text("每次只发一次，无到位回包；停稳后再发送。请按剩余行程选择距离，每圈脉冲须匹配电机细分。")
    Spacer(Modifier.height(16.dp))
}

@Composable
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

@Composable
private fun MotionButton(
    label: String,
    direction: Char,
    enabled: Boolean,
    onStartMotion: (Char) -> Unit,
    onStopMotion: () -> Unit,
) {
    val touchModifier = if (enabled) {
        Modifier.pointerInput(direction) {
            detectTapGestures(onPress = {
                onStartMotion(direction)
                tryAwaitRelease()
                onStopMotion()
            })
        }
    } else Modifier

    Box(
        modifier = Modifier.size(96.dp).clip(RoundedCornerShape(18.dp))
            .background(if (enabled) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.surfaceVariant)
            .then(touchModifier),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = label,
            style = MaterialTheme.typography.displaySmall,
            color = if (enabled) MaterialTheme.colorScheme.onPrimary else MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

@Preview(showBackground = true)
@Composable
private fun RemoteControlPreview() {
    AppTheme {
        RemoteControlScreen(
            connected = false,
            connectionText = "未连接",
            communicationTrace = "",
            onConnect = { _, _ -> },
            onDisconnect = {},
            moveRunning = false,
            moveStatus = "等待定距移动",
            onMove = {},
            onAlign = { _, _ -> },
            onStopMotion = {},
            onStartLift = {},
            onStopLift = {},
            onLiftDistance = { _, _, _, _ -> },
            onStopForeAft = {},
            onArmDistance = { _, _, _, _ -> },
            onArmPose = { _, _, _, _, _ -> },
            onHome = {},
            onInitializeOrigin = {},
            onStopArm = {},
            onGrab = {},
            onCancelGrab = {},
            onServoChange = { _, _ -> },
            onServoFinished = { _, _ -> },
        )
    }
}
