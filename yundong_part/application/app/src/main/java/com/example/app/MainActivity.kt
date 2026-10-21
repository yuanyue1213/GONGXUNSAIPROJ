package com.example.app

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.relocation.BringIntoViewRequester
import androidx.compose.foundation.relocation.bringIntoViewRequester
import androidx.compose.foundation.layout.WindowInsets
import androidx.compose.foundation.layout.ime
import androidx.compose.foundation.layout.imePadding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.saveable.rememberSaveableStateHolder
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.platform.LocalSoftwareKeyboardController
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import androidx.compose.foundation.clickable
import androidx.compose.runtime.key
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.ui.platform.LocalContext
import java.util.UUID
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.example.app.ui.theme.AppTheme

class MainActivity : ComponentActivity() {
    private var moveStatus by mutableStateOf("等待定距移动")
    private var motorFaults by mutableStateOf<Map<String, String>>(emptyMap())
    private var moveRunning by mutableStateOf(false)
    private var manualMoveId = 0L
    private var manualMoveAccepted = false
    private val linkProbe = LinkProbe()
    private var linkStatus by mutableStateOf("尚未检测STM32回包")
    private var feedbackLines by mutableStateOf(emptyList<String>())
    private var autoStatus by mutableStateOf("等待一键启动")
    private var autoRunning by mutableStateOf(false)
    private var autoCode by mutableStateOf("")
    private lateinit var robotClient: RobotTcpClient

    private var connectionText by mutableStateOf("未连接")
    private var isConnected by mutableStateOf(false)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        robotClient = RobotTcpClient(
            onConnectionChanged = { connected, message ->
                runOnUiThread {
                    isConnected = connected
                    connectionText = message
                    if (!connected) {
                        linkProbe.clear(); linkStatus = "未连接，无法检测STM32回包"
                        moveRunning = false; manualMoveId = 0L
                        autoRunning = false
                        window.clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                        moveStatus = "已断开，无法确认底盘状态"
                    }
                }
            },
            onMessage = { line -> runOnUiThread {
                feedbackLines = (feedbackLines + (if (line.startsWith("TX,")) line else "RX,$line")).takeLast(24)
                linkProbe.receive(line)?.let { linkStatus = it }
                if (line.startsWith("STATUS,")) {
                    val status = line.split(',')
                    if (status.size == 5) {
                        linkStatus = "STM32：${if (status[2] == "IDLE") "空闲" else "忙碌"}；机械臂原点${if (status[3] == "1") "已就绪" else "未初始化"}；转盘目标${status[4]}°"
                    }
                }
                if (line.startsWith("IMU,")) {
                    moveStatus = ImuFeedback.text(line)
                }
                MotorFeedback.text(line)?.let { (key, message) ->
                    motorFaults = motorFaults + (key to message)
                }
                if (line.startsWith("ACTION,")) {
                    val action = line.split(',')
                    if (action.getOrNull(1)?.toLongOrNull() == manualMoveId) {
                        if (action.getOrNull(2) == "ACCEPTED") manualMoveAccepted = true
                        if (action.getOrNull(2) in listOf("DONE", "ERROR", "CANCELLED", "BUSY")) {
                            moveRunning = false; manualMoveId = 0L
                        }
                    }
                    moveStatus = when (action.getOrNull(2)) {
                        "ACCEPTED" -> "动作已接收，等待闭环完成"
                        "DONE" -> "动作完成"
                        "ERROR" -> "动作失败：${action.getOrNull(3)}"
                        "CANCELLED" -> "动作已取消"
                        "BUSY" -> "执行器忙碌：" + when (val reason = action.getOrNull(3)) {
                            "STARTUP_STOP" -> "底盘启动停车未获全部驱动器确认，请检查四轮驱动器"
                            "TASK_ACTIVE", "ACTIVE" -> "上一动作尚未结束"
                            "GYRO_ACTIVE", "CHASSIS_ACTIVE" -> "底盘动作尚未结束"
                            "CHASSIS_STOP_PENDING" -> "底盘停车确认中"
                            "ARM_STOP_PENDING" -> "机械臂前后轴停车未确认"
                            "LIFT_STOP_PENDING" -> "升降轴停车未确认"
                            "ORIGIN_INITIALIZING" -> "机械臂原点初始化中"
                            "ARM_ACTIVE" -> "机械臂前后轴动作中"
                            "LIFT_ACTIVE" -> "升降轴动作中"
                            "GRAB_ACTIVE" -> "抓放动作中"
                            "PLAN_ACTIVE", "POSE_ACTIVE" -> "机械臂路径执行中"
                            "BASE_ACTIVE" -> "机械臂底座动作中"
                            "GRIPPER_ACTIVE" -> "夹爪动作中"
                            "VISION_ACTIVE" -> "视觉校准中"
                            else -> reason ?: "原因未知"
                        }
                        else -> line
                    }
                }
                if (line.startsWith("AUTO,") || line.startsWith("TASK,")) {
                    autoStatus = AutonomousStatus.text(line)
                    if (line.startsWith("TASK,")) autoCode = line.split(',').getOrNull(2) ?: ""
                    val state = line.split(',').getOrNull(2)
                    if (line.startsWith("AUTO,") && state in listOf("DONE", "STOPPED", "ERROR")) autoRunning = false
                    if (line.startsWith("AUTO,") && state in listOf("STARTING", "RUNNING", "TURNING", "SCANNING", "WORK", "CALIBRATING")) autoRunning = true
                    if (autoRunning) window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                    else window.clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                }
            } },
        )

        enableEdgeToEdge()
        setContent {
            AppTheme {
                RemoteControlScreen(
                    connected = isConnected,
                    connectionText = connectionText,
                    onConnect = { host, port ->
                        robotClient.connect(host, port)
                    },
                    onDisconnect = { robotClient.sendMotion('Z'); stopMotion(); stopLift(); stopForeAft(); robotClient.disconnect() },
                    moveRunning = moveRunning,
                    moveStatus = moveStatus,
                    motorFaultText = motorFaults.values.joinToString("\n"),
                    onClearMotorFaults = { motorFaults = emptyMap() },
                    onMove = ::startDistanceMove,
                    onAlign = { forward, lateral, settings ->
                        robotClient.sendConfiguredAlignment(0, forward, lateral, settings)
                        moveStatus = "已提交位置修正；停止底盘可取消，无到位回包"
                    },
                    onRingAlign = { ring, forward, lateral, settings ->
                        robotClient.sendConfiguredAlignment(ring, forward, lateral, settings)
                        moveStatus = "已提交色环位置修正；停止底盘可取消，无到位回包"
                    },
                    onJointAlign = { ring, forward, lateral, settings, geometry -> robotClient.sendJointAlignment(ring, forward, lateral, settings, geometry) },
                    onParallel = { ppm, rpm, step, reverse -> robotClient.sendParallel(ppm, rpm, step, reverse) },
                    onStopMotion = ::stopMotion,
                    onArmPose = { theta, r, z, rpm, pulses ->
                        robotClient.sendArmPose(theta, r, z, rpm, pulses)
                    },
                    onHome = { robotClient.sendMotion('O') },
                    onInitializeOrigin = { base -> robotClient.initializeOrigin(base) },
                    onStopArm = { robotClient.sendMotion('Z'); stopLift(); stopForeAft() },
                    onGrab = { settings -> robotClient.sendSequence('A', settings) },
                    onRelease = { settings -> robotClient.sendSequence('P', settings) },
                    onPlan = { cards -> robotClient.sendPlan(cards) },
                    onCancelGrab = { robotClient.sendMotion('Z') },
                    onGripper = { start, end, dps -> robotClient.sendGripper(start, end, dps) },
                    onStopGripper = { robotClient.stopGripper() },
                    onServoFinished = ::sendServoAngle,
                    onBaseMove = { target, dps -> robotClient.sendBase(target, dps) },
                    autoStatus = autoStatus,
                    autoRunning = autoRunning,
                    autoCode = autoCode,
                    onAutoStart = { settings, parameters ->
                        robotClient.startAutonomous(settings, parameters)
                        autoRunning = true
                        autoCode = ""
                        autoStatus = "已提交，等待ESP32检查配置及STM32状态"
                    },
                    onAutoStop = { robotClient.stopAutonomous() },
                    onRequestImu = { robotClient.requestImu() },
                    linkStatus = linkStatus,
                    feedbackLog = feedbackLines.joinToString("\n"),
                    onCheckLink = {
                        val id = robotClient.checkLink()
                        linkStatus = linkProbe.start(id)
                        window.decorView.postDelayed({
                            if (isConnected) linkProbe.timeout(id)?.let { linkStatus = it }
                        }, 3000)
                    },
                    onRequestStatus = { robotClient.requestStatus() },
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
    }

    override fun onDestroy() {
        stopMotion()
        stopLift()
        stopForeAft()
        robotClient.close()
        super.onDestroy()
    }

    private fun startDistanceMove(move: DistanceMove) {
        if (!isConnected) return
        manualMoveId = robotClient.sendMove(move)
        manualMoveAccepted = false
        moveRunning = true
        val pendingId = manualMoveId
        window.decorView.postDelayed({
            if (manualMoveId == pendingId && !manualMoveAccepted && isConnected)
                moveStatus = "3秒内未收到动作确认，请检测双向通信或停止底盘；不会自动重发移动命令"
        }, 3000)
        moveStatus = move.rotationDegrees?.let {
            "正在提交${if (move.direction == 'C') "左转" else "右转"} $it° 命令"
        } ?: "正在提交 ${move.distanceMm} mm 命令"
    }

    private fun stopMotion() {
        moveRunning = false; manualMoveId = 0L
        moveStatus = "已请求底盘停车"
        robotClient.sendMotion('S')
    }

    private fun stopLift() {
        robotClient.sendMotion('H')
    }

    private fun stopForeAft() {
        robotClient.sendMotion('Q')
    }

    private fun sendServoAngle(channel: Char, angle: Int) {
        if (isConnected) robotClient.sendServo(channel, angle)
    }
}

@Composable
private fun RemoteControlScreen(
    connected: Boolean,
    connectionText: String,


    onConnect: (String, Int) -> Unit,
    onDisconnect: () -> Unit,

    moveRunning: Boolean,
    moveStatus: String,
    onMove: (DistanceMove) -> Unit,
    onAlign: (Int, Int, AlignmentSettings) -> Unit,
    onRingAlign: (Int, Int, Int, AlignmentSettings) -> Unit,
    onJointAlign: (Int, Int, Int, AlignmentSettings, JointAlignmentSettings) -> Unit,
    onParallel: (Int, Int, Int, Boolean) -> Unit,
    onStopMotion: () -> Unit,
    onArmPose: (Int, Int, Int, Int, Int) -> Unit,
    onHome: () -> Unit,
    onInitializeOrigin: (Int) -> Unit,
    onStopArm: () -> Unit,
    onGrab: (SequenceSettings) -> Unit,
    onRelease: (SequenceSettings) -> Unit,
    onPlan: (List<SequenceCard>) -> Unit = {},
    onCancelGrab: () -> Unit,
    onGripper: (Int, Int, Int) -> Unit,
    onStopGripper: () -> Unit,
    onServoFinished: (Char, Int) -> Unit,
    onBaseMove: (Int, Int) -> Unit = { _, _ -> },
    autoStatus: String = "等待一键启动",
    autoRunning: Boolean = false,
    autoCode: String = "",
    onAutoStart: (AutonomousSettings, List<AutonomousArmParameters>) -> Unit = { _, _ -> },
    onAutoStop: () -> Unit = {},
    onRequestImu: () -> Unit = {},
    linkStatus: String = "尚未检测STM32回包",
    feedbackLog: String = "",
    onCheckLink: () -> Unit = {},
    onRequestStatus: () -> Unit = {},
    motorFaultText: String = "",
    onClearMotorFaults: () -> Unit = {},
) {
    var host by rememberSaveable { mutableStateOf("192.168.4.1") }
    var portText by rememberSaveable { mutableStateOf("3333") }
    var addressError by remember { mutableStateOf<String?>(null) }

    var page by rememberSaveable { mutableStateOf(0) }
    val pages = listOf("连接", "底盘", "机械臂", "舵机", "调试", "全流程")
    val pageState = rememberSaveableStateHolder()
    val context = LocalContext.current
    val cardStore = remember(context) { SequenceCardStore(context) }
    var cards by remember { mutableStateOf(cardStore.cards()) }
    var plan by remember { mutableStateOf(cardStore.plan()) }
    fun saveCard(name: String, mode: Char, settings: SequenceSettings) {
        cards = cards + SequenceCard(UUID.randomUUID().toString(), name.trim(), mode, settings)
        cardStore.saveCards(cards)
    }
    val focusManager = LocalFocusManager.current
    val keyboard = LocalSoftwareKeyboardController.current

    // 系统边缘与键盘占用空间交给布局处理，输入页只在剩余高度内滚动。
    Scaffold(
        modifier = Modifier.fillMaxSize().safeDrawingPadding().imePadding(),
        contentWindowInsets = WindowInsets(0, 0, 0, 0),
        topBar = {
            Row(Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 4.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.SpaceBetween) {
                Column {
                    Text(pages[page], style = MaterialTheme.typography.titleLarge)
                    Text(connectionText, style = MaterialTheme.typography.bodySmall,
                        color = if (connected) Color(0xFF16803C) else MaterialTheme.colorScheme.error)
                }
                Button(onClick = { onStopArm(); onStopMotion() }, enabled = connected,
                    colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error)) {
                    Text("全部停止")
                }
            }
        },
        bottomBar = {
            NavigationBar {
                pages.forEachIndexed { index, title ->
                    NavigationBarItem(selected = page == index,
                        onClick = { focusManager.clearFocus(); keyboard?.hide(); page = index },
                        icon = { Text(if (page == index) "●" else "○") },
                        label = { Text(title) })
                }
            }
        },
    ) { padding ->
        // 每页各自保存参数和滚动位置，切走后不会重置。
        pageState.SaveableStateProvider(page) {
            val scroll = rememberScrollState()
            val scope = rememberCoroutineScope()
            Column(
                modifier = Modifier.fillMaxSize().padding(padding)
                    .verticalScroll(scroll).padding(horizontal = 16.dp, vertical = 12.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                if (motorFaultText.isNotEmpty()) {
                    Text("最近电机异常（历史记录）\n$motorFaultText",
                        modifier = Modifier.fillMaxWidth(), color = MaterialTheme.colorScheme.error)
                    TextButton(onClick = onClearMotorFaults) { Text("清除异常记录") }
                }
                when (page) {
                    5 -> AutonomousPanel(connected, autoRunning, autoStatus, autoCode, onAutoStart, onAutoStop)
                    0 -> {
                        VisibleTextField(
                            value = host,
                            onValueChange = { host = it },
                            modifier = Modifier.fillMaxWidth(),
                            singleLine = true,
                            label = { Text("ESP32 地址") },
                            enabled = !connected,
                        )
                        Spacer(Modifier.height(8.dp))
                        VisibleTextField(
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
                        Spacer(Modifier.height(12.dp))
                        Text(linkStatus, modifier = Modifier.fillMaxWidth())
                        Button(onClick = onCheckLink, enabled = connected, modifier = Modifier.fillMaxWidth()) {
                            Text("检测双向通信（不驱动电机）")
                        }
                        Button(onClick = onRequestStatus, enabled = connected, modifier = Modifier.fillMaxWidth()) {
                            Text("查询STM32状态")
                        }
                        Text("串口：ESP TX17→STM PB12；STM PB13→ESP RX18；两板共地，115200。")
                        Text("最近24条通信记录（TX为App发出，RX为收到；TX不代表执行成功）")
                        androidx.compose.foundation.text.selection.SelectionContainer {
                            Text(feedbackLog.ifEmpty { "暂无记录" }, style = MaterialTheme.typography.bodySmall,
                                modifier = Modifier.fillMaxWidth())
                        }
                    }
                    1 -> DistanceMovePanel(connected, moveRunning, moveStatus, onMove, onAlign, onRingAlign, onJointAlign, onParallel, onStopMotion,
                        onSectionSelected = { scope.launch { scroll.scrollTo(0) } }, onRequestImu = onRequestImu)
                    2 -> ArmPosePanel(connected, onArmPose, onInitializeOrigin, onHome, onStopArm)
                    3 -> {
                        ServoAngleInput("夹子 · PC8", 'G', 270, connected, onServoFinished)
                        ServoAngleInput("转盘 · PA8", 'T', 270, connected, onServoFinished)
                        BaseAnglePanel(connected, onBaseMove, onStopArm)
                        GripperMotionPanel(connected, onGripper, onStopGripper)
                    }
                    4 -> {
                        var flow by rememberSaveable { mutableStateOf(0) }
                        val flowState = rememberSaveableStateHolder()
                        TabRow(selectedTabIndex = flow) {
                            listOf("抓取", "放下", "转盘", "卡片编排").forEachIndexed { index, title ->
                                Tab(selected = flow == index, onClick = {
                                    focusManager.clearFocus(); keyboard?.hide(); flow = index
                                    scope.launch { scroll.scrollTo(0) }
                                }, text = { Text(title) })
                            }
                        }
                        Spacer(Modifier.height(12.dp))
                        flowState.SaveableStateProvider(flow) {
                            when (flow) {
                                0 -> SequenceSettingsPanel("抓取", false, connected, onGrab,
                                    onSave = { name, settings -> saveCard(name, 'A', settings) })
                                1 -> SequenceSettingsPanel("放下", true, connected, onRelease,
                                    onSave = { name, settings -> saveCard(name, 'P', settings) })
                                2 -> TurnCardPanel(connected, onSave = { name, turn ->
                                    cards = cards + SequenceCard(UUID.randomUUID().toString(), name, 'T', turn = turn)
                                    cardStore.saveCards(cards)
                                }, onExecute = { turn -> onPlan(listOf(SequenceCard("manual-turn", "转盘", 'T', turn = turn))) })
                                else -> SequenceCardsPanel(cards, plan, connected,
                                    onPlanChange = { plan = it; cardStore.savePlan(it) },
                                    onDeleteCard = { id -> cards = cards.filterNot { it.id == id }; cardStore.saveCards(cards) },
                                    onUpdateCard = { edited ->
                                        cards = cards.map { if (it.id == edited.id) edited else it }
                                        cardStore.saveCards(cards)
                                    },
                                    onEditStarted = {
                                        focusManager.clearFocus(); keyboard?.hide()
                                        scope.launch { scroll.scrollTo(0) }
                                    },
                                    onExecute = { onPlan(plan) })
                            }
                        }
                        Button(onClick = onCancelGrab, enabled = connected, modifier = Modifier.fillMaxWidth()) {
                            Text("取消自动流程")
                        }
                    }
                }
                Spacer(Modifier.height(24.dp))
            }
        }
    }
}

@Composable
private fun DistanceMovePanel(
    connected: Boolean,
    running: Boolean,
    status: String,
    onMove: (DistanceMove) -> Unit,
    onAlign: (Int, Int, AlignmentSettings) -> Unit,
    onRingAlign: (Int, Int, Int, AlignmentSettings) -> Unit,
    onJointAlign: (Int, Int, Int, AlignmentSettings, JointAlignmentSettings) -> Unit,
    onParallel: (Int, Int, Int, Boolean) -> Unit,
    onStop: () -> Unit,
    onSectionSelected: () -> Unit = {},
    onRequestImu: () -> Unit = {},
) {
    var direction by rememberSaveable { mutableStateOf('F') }
    var distance by rememberSaveable { mutableStateOf("100") }
    var speed by rememberSaveable { mutableStateOf("45") }
    var diameter by rememberSaveable { mutableStateOf("103") }
    var wheelPulses by rememberSaveable { mutableStateOf("3200") }
    var forwardCorrection by rememberSaveable { mutableStateOf("1.0") }
    var lateralCorrection by rememberSaveable { mutableStateOf("1.0") }
    var error by remember { mutableStateOf<String?>(null) }
    var ring by rememberSaveable { mutableStateOf(2) }
    var turnMm by rememberSaveable { mutableStateOf("5") }
    var turnDegrees by rememberSaveable { mutableStateOf("90") }
    var turnRpm by rememberSaveable { mutableStateOf("10") }
    var parallelStep by rememberSaveable { mutableStateOf("3") }
    var reverseTurn by rememberSaveable { mutableStateOf(false) }
    var fineRpm by rememberSaveable { mutableStateOf("10") }
    var coarseRpm by rememberSaveable { mutableStateOf("20") }
    var xOffset by rememberSaveable { mutableStateOf("10") }
    var centerX by rememberSaveable { mutableStateOf("256") }
    var centerY by rememberSaveable { mutableStateOf("160") }
    fun alignmentSettings() = AlignmentSettings.parse(fineRpm, coarseRpm, xOffset, centerX, centerY)
    var wheelbase by rememberSaveable { mutableStateOf("190") }
    var track by rememberSaveable { mutableStateOf("251.4") }
    var cameraForward by rememberSaveable { mutableStateOf("-7") }
    var cameraLeft by rememberSaveable { mutableStateOf("-291.27") }
    var section by rememberSaveable { mutableStateOf(0) }
    val focusManager = LocalFocusManager.current
    val keyboard = LocalSoftwareKeyboardController.current
    TabRow(selectedTabIndex = section) {
        listOf("移动", "视觉", "转向", "标定").forEachIndexed { index, title ->
            Tab(selected = section == index, onClick = {
                focusManager.clearFocus(); keyboard?.hide(); section = index; error = null
                onSectionSelected()
            }, text = { Text(title) })
        }
    }
    Spacer(Modifier.height(12.dp))
    Text(status, modifier = Modifier.fillMaxWidth())
    if (section == 0) {
        Text("底盘定距移动", style = MaterialTheme.typography.titleMedium)
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
        Text("前进/后退：JY901S保持起步朝向，四轮编码器控制距离。需陀螺仪在线，且每圈脉冲与电机设置一致。")
        DistanceInput("电机速度（RPM，5–300）", speed, !running) { speed = it }
        Button(
            onClick = {
                val correction = if (direction in "LR") lateralCorrection else forwardCorrection
                val move = DistanceMove.fromInputs(direction, distance, speed, diameter, wheelPulses, correction)
                if (move == null) {
                    error = "请检查距离、速度及标定参数：轮径 20–500 mm，每圈脉冲 200–51200，修正系数 0.1–10。"
                } else {
                    error = null
                    onMove(move)
                }
            },
            enabled = connected && !running,
            modifier = Modifier.fillMaxWidth(),
        ) { Text(if (running) "移动中…" else "执行定距移动") }
    }
    if (section == 1) {
        Text("位置修正参数", style = MaterialTheme.typography.titleMedium)
        Text("画面分辨率 512×320，左上角为 (0,0)，支持一位小数。")
        DistanceInput("目标中心 X（px，0–511.9）", centerX, !running, true) { centerX = it }
        DistanceInput("目标中心 Y（px，0–319.9）", centerY, !running, true) { centerY = it }
        Button(onClick = { centerX = "256"; centerY = "160" }, enabled = !running) { Text("恢复画面中心 (256,160)") }
        DistanceInput("精调及补偿速度（RPM，5–60）", fineRpm, !running) { fineRpm = it }
        DistanceInput("粗调速度（RPM，5–60）", coarseRpm, !running) { coarseRpm = it }
        VisibleTextField(value = xOffset, onValueChange = { xOffset = it },
            label = { Text("结束 x 补偿（mm，-100–100，0关闭）") },
            singleLine = true, modifier = Modifier.fillMaxWidth())
        Button(
            onClick = {
                val forward = DistanceMove.fromInputs('F', "1000", "20", diameter, wheelPulses, forwardCorrection)
                val lateral = DistanceMove.fromInputs('L', "1000", "20", diameter, wheelPulses, lateralCorrection)
                val settings = alignmentSettings()
                if (forward == null || lateral == null || settings == null) error = "请检查目标中心范围及精度、位置修正参数和底盘标定"
                else { error = null; onAlign(forward.pulsesPerMetre, lateral.pulsesPerMetre, settings) }
            }, enabled = connected && !running, modifier = Modifier.fillMaxWidth(),
        ) { Text("执行位置修正") }
        Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
            listOf(1 to "左色环", 2 to "中色环", 3 to "右色环").forEach { (value, label) ->
                Button(onClick = { ring = value }, enabled = !running) {
                    Text(if (ring == value) "✓ $label" else label)
                }
            }
        }
        Button(onClick = {
            val forward = DistanceMove.fromInputs('F', "1000", "20", diameter, wheelPulses, forwardCorrection)
            val lateral = DistanceMove.fromInputs('L', "1000", "20", diameter, wheelPulses, lateralCorrection)
            val settings = alignmentSettings()
            if (forward == null || lateral == null || settings == null) error = "请检查目标中心范围及精度、位置修正参数和底盘标定"
            else { error = null; onRingAlign(ring, forward.pulsesPerMetre, lateral.pulsesPerMetre, settings) }
        }, enabled = connected && !running, modifier = Modifier.fillMaxWidth()) {
            Text("执行色环位置修正")
        }
        HorizontalDivider(Modifier.padding(vertical = 12.dp))
        Text("中心＋平行联合校准", style = MaterialTheme.typography.titleMedium)
        DistanceInput("前后轮中心间距（mm，50–2000）", wheelbase, !running, true) { wheelbase = it }
        DistanceInput("左右轮中心间距（mm，50–2000）", track, !running, true) { track = it }
        VisibleTextField(value = cameraForward, onValueChange = { cameraForward = it },
            label = { Text("画面中心相对底盘中心：前方偏移（mm）") }, singleLine = true, modifier = Modifier.fillMaxWidth())
        VisibleTextField(value = cameraLeft, onValueChange = { cameraLeft = it },
            label = { Text("画面中心相对底盘中心：左方偏移（mm）") }, singleLine = true, modifier = Modifier.fillMaxWidth())
        Button(onClick = { reverseTurn = !reverseTurn }) {
            Text(if (reverseTurn) "联合转向：反向" else "联合转向：默认")
        }
        Button(onClick = {
            val forward = DistanceMove.fromInputs('F', "1000", "20", diameter, wheelPulses, forwardCorrection)
            val lateral = DistanceMove.fromInputs('L', "1000", "20", diameter, wheelPulses, lateralCorrection)
            val settings = alignmentSettings()
            val values = listOf(wheelbase, track, cameraForward, cameraLeft).map { it.trim().toDoubleOrNull() }
            val geometry = if (values.all { it != null }) JointAlignmentSettings(values[0]!!, values[1]!!, values[2]!!, values[3]!!, reverseTurn) else null
            if (forward == null || lateral == null || settings == null || geometry == null || !geometry.valid())
                error = "请检查目标中心范围及精度、实际轮距、摄像头偏移和底盘标定参数"
            else { error = null; onJointAlign(ring, forward.pulsesPerMetre, lateral.pulsesPerMetre, settings, geometry) }
        }, enabled = connected && !running, modifier = Modifier.fillMaxWidth()) { Text("执行中心＋平行校准") }
    }
    if (section == 2) {
        Text("JY901S · PD闭环旋转", style = MaterialTheme.typography.titleMedium)
        Text("X前、Y右、Z下；USART3固定9600，启动配置10Hz角度/角速度输出，需接通PB11与PB10双向串口。")
        Button(onClick = onRequestImu, enabled = connected) { Text("读取JY901S角度") }
        DistanceInput("旋转角度（度，0.1–360）", turnDegrees, !running, true) { turnDegrees = it }
        DistanceInput("定角度转速（RPM，5–60）", turnRpm, !running) { turnRpm = it }
        Text("相对当前朝向旋转；需要JY901S在线并静止，角度由传感器闭环判断。")
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            listOf('C' to "定角度左转", 'W' to "定角度右转").forEach { (dir, label) ->
                Button(onClick = {
                    val turn = DistanceMove.gyroRotationFromInputs(dir, turnDegrees, turnRpm)
                    if (turn == null) error = "请输入0.1–360度（一位小数）和5–60 RPM速度上限"
                    else { error = null; onMove(turn) }
                }, enabled = connected && !running) { Text(label) }
            }
        }
        HorizontalDivider(Modifier.padding(vertical = 12.dp))
        Text("原始脉冲转向测试 / 圆心连线校准（不使用陀螺仪闭环）", style = MaterialTheme.typography.titleMedium)
        DistanceInput("转动车轮距离（mm）", turnMm, true) { turnMm = it }
        DistanceInput("转向速度（RPM，5–60）", turnRpm, true) { turnRpm = it }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            listOf('C' to "左转", 'W' to "右转").forEach { (dir, label) ->
                Button(onClick = {
                    val turn = DistanceMove.fromInputs(dir, turnMm, turnRpm, diameter, wheelPulses, forwardCorrection)
                    if (turn == null || turn.speedRpm !in 5..60) error = "请检查转向距离、速度和底盘标定"
                    else { error = null; onMove(turn) }
                }, enabled = connected && !running) { Text(label) }
            }
        }
        DistanceInput("自动转向单步上限（mm，1–10）", parallelStep, true) { parallelStep = it }
        Button(onClick = { reverseTurn = !reverseTurn }) {
            Text(if (reverseTurn) "校准转向：反向" else "校准转向：默认")
        }
        Button(onClick = {
            val calibration = DistanceMove.fromInputs('F', "1000", "10", diameter, wheelPulses, forwardCorrection)
            val rpm = turnRpm.toIntOrNull(); val step = parallelStep.toIntOrNull()
            if (calibration == null || rpm == null || rpm !in 5..60 || step == null || step !in 1..10)
                error = "转向速度：5–60 RPM；单步：1–10 mm"
            else { error = null; onParallel(calibration.pulsesPerMetre, rpm, step, reverseTurn) }
        }, enabled = connected && !running, modifier = Modifier.fillMaxWidth()) { Text("执行平行校准") }
    }
    if (section == 3) {
        Text("距离标定 · 减速比 1:1", style = MaterialTheme.typography.labelLarge)
        DistanceInput("轮径（mm）", diameter, !running, true) { diameter = it }
        DistanceInput("车轮每圈脉冲", wheelPulses, !running) { wheelPulses = it }
        DistanceInput("前后距离修正系数", forwardCorrection, !running, true) { forwardCorrection = it }
        DistanceInput("左右距离修正系数", lateralCorrection, !running, true) { lateralCorrection = it }
    }
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(
        onClick = onStop,
        enabled = connected,
        modifier = Modifier.fillMaxWidth(),
        colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error),
    ) { Text("停止底盘") }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
private fun VisibleTextField(
    value: String,
    onValueChange: (String) -> Unit,
    modifier: Modifier = Modifier,
    label: @Composable (() -> Unit)? = null,
    singleLine: Boolean = true,
    enabled: Boolean = true,
    keyboardOptions: KeyboardOptions = KeyboardOptions.Default,
) {
    val requester = remember { BringIntoViewRequester() }
    var focused by remember { mutableStateOf(false) }
    val keyboardHeight = WindowInsets.ime.getBottom(LocalDensity.current)
    LaunchedEffect(focused, keyboardHeight) {
        if (focused) {
            // 等待键盘动画和页面重排后，将整个输入框滚入可见区域。
            delay(150)
            requester.bringIntoView()
        }
    }
    OutlinedTextField(value = value, onValueChange = onValueChange,
        modifier = modifier.bringIntoViewRequester(requester).onFocusChanged { focused = it.isFocused },
        label = label, singleLine = singleLine, enabled = enabled, keyboardOptions = keyboardOptions)
}

@Composable
private fun DistanceInput(
    label: String, value: String, enabled: Boolean, decimal: Boolean = false,
    onChange: (String) -> Unit,
) {
    VisibleTextField(
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
private fun ServoAngleInput(label: String, channel: Char, maximum: Int,
                            enabled: Boolean, onFinished: (Char, Int) -> Unit) {
    var angle by rememberSaveable(channel) { mutableStateOf(when (channel) { 'G' -> "15"; 'T' -> "0"; else -> "248" }) }
    var error by remember { mutableStateOf<String?>(null) }
    DistanceInput("$label 角度（0–$maximum 度）", angle, true) { angle = it }
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(onClick = {
        val value = angle.toIntOrNull()
        if (value == null || value !in 0..maximum) error = "请输入 0–$maximum 度"
        else { error = null; onFinished(channel, value) }
    }, enabled = enabled, modifier = Modifier.fillMaxWidth()) { Text("设置$label") }
}

@Composable
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

@Composable
private fun GripperMotionPanel(connected: Boolean, onExecute: (Int, Int, Int) -> Unit, onStop: () -> Unit) {
    var start by rememberSaveable { mutableStateOf("0") }
    var end by rememberSaveable { mutableStateOf("60") }
    var dps by rememberSaveable { mutableStateOf("60") }
    var error by remember { mutableStateOf<String?>(null) }
    Text("夹子开合", style = MaterialTheme.typography.titleMedium)
    DistanceInput("起始角度（0–270 度）", start, true) { start = it }
    DistanceInput("结束角度（0–270 度）", end, true) { end = it }
    DistanceInput("速度（6–300 度/秒）", dps, true) { dps = it }
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(onClick = {
        val from = start.toIntOrNull(); val to = end.toIntOrNull(); val speed = dps.toIntOrNull()
        if (from == null || from !in 0..270 || to == null || to !in 0..270 || speed == null || speed !in 6..300)
            error = "角度：0–270 度；速度：6–300 度/秒"
        else { error = null; onExecute(from, to, speed) }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("执行夹子开合") }
    Button(onClick = onStop, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("停止夹子") }
}

@Composable
private fun SequenceSettingsPanel(
    title: String, release: Boolean, connected: Boolean, onExecute: (SequenceSettings) -> Unit,
    onSave: (String, SequenceSettings) -> Unit,
    initialCard: SequenceCard? = null, editing: Boolean = false,
) {
    val initial = initialCard?.settings
    var r1 by rememberSaveable { mutableStateOf(initial?.let { (it.r1 / 10.0).toString() } ?: if (release) "20" else "110") }
    var z1 by rememberSaveable { mutableStateOf(initial?.z1?.toString() ?: if (release) "-40" else "-50") }
    var r2 by rememberSaveable { mutableStateOf(initial?.let { (it.r2 / 10.0).toString() } ?: if (release) "130" else "20") }
    var z2 by rememberSaveable { mutableStateOf(initial?.z2?.toString() ?: if (release) "-110" else "-40") }
    var combined by rememberSaveable { mutableStateOf(initial?.combinedRpm?.toString() ?: "20") }
    var gripper by rememberSaveable { mutableStateOf(initial?.gripperDps?.toString() ?: "60") }
    var openAngle by rememberSaveable { mutableStateOf(initial?.openAngle?.toString() ?: "60") }
    var closeAngle by rememberSaveable { mutableStateOf(initial?.closeAngle?.toString() ?: "0") }
    var baseHome by rememberSaveable { mutableStateOf(initial?.baseHome?.toString() ?: "248") }
    var baseTilt by rememberSaveable { mutableStateOf(initial?.baseTilt?.toString() ?: "140") }
    var cardName by rememberSaveable { mutableStateOf(initialCard?.name ?: title) }
    var saved by remember { mutableStateOf(false) }
    var error by remember { mutableStateOf<String?>(null) }
    Text("${title}参数", style = MaterialTheme.typography.titleMedium)
    val first = if (release) "第3步取物" else "第2步抓取"
    val second = "第6步放置"
    VisibleTextField(value = r1, onValueChange = { r1 = it }, label = { Text("$first r（mm）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())
    VisibleTextField(value = z1, onValueChange = { z1 = it }, label = { Text("$first z（mm）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())
    VisibleTextField(value = r2, onValueChange = { r2 = it }, label = { Text("$second r（mm）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())
    VisibleTextField(value = z2, onValueChange = { z2 = it }, label = { Text("$second z（mm）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())
    DistanceInput("r/z 合成速度（RPM，5–600）", combined, true) { combined = it }
    Text("下降时先并行到目标上方10 mm，r到位后z以300 RPM完成末段；下降不足10 mm时全部留到末段。")
    DistanceInput("夹子张开速度（度/秒，6–300）", gripper, true) { gripper = it }
    DistanceInput("夹子张开角度（0–270 度）", openAngle, true) { openAngle = it }
    DistanceInput("夹子夹紧角度（0–270 度）", closeAngle, true) { closeAngle = it }
    DistanceInput(if (release) "放置角度（基座，0–360 度）" else "抓取角度（基座，0–360 度）", baseHome, true) { baseHome = it }
    DistanceInput(if (release) "转盘取物角度（基座，0–360 度）" else "转盘存放角度（基座，0–360 度）", baseTilt, true) { baseTilt = it }
    VisibleTextField(value = cardName, onValueChange = { cardName = it; saved = false },
        label = { Text("卡片名称（最多40字）") }, modifier = Modifier.fillMaxWidth())
    Button(onClick = {
        val settings = SequenceSettings.parse(r1, z1, r2, z2, combined, gripper, (initial?.theta ?: 0).toString(), openAngle, closeAngle, baseHome, baseTilt)
        if (settings == null) error = "请检查位置、速度和角度参数"
        else if (cardName.isBlank() || cardName.trim().length > 40) error = "请填写1–40字的卡片名称"
        else { error = null; onSave(cardName.trim(), settings); saved = true }
    }, modifier = Modifier.fillMaxWidth()) { Text(if (editing) "保存修改" else "保存为卡片") }
    if (saved) Text("已保存，可在卡片编排中选择")
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    if (!editing) Button(onClick = {
        val settings = SequenceSettings.parse(r1, z1, r2, z2, combined, gripper, "0", openAngle, closeAngle, baseHome, baseTilt)
        if (settings == null) error = "r：±1000 mm（最多1位小数）；z：±400 mm（整数）；合成速度：5–600 RPM；夹子：6–300 度/秒"
        else { error = null; onExecute(settings) }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("执行$title") }
    Spacer(Modifier.height(12.dp))
}

/** Plan entries are snapshots, so deleting a library card does not alter a saved plan. */
@Composable
private fun SequenceCardsPanel(
    cards: List<SequenceCard>, plan: List<SequenceCard>, connected: Boolean,
    onPlanChange: (List<SequenceCard>) -> Unit, onDeleteCard: (String) -> Unit, onExecute: () -> Unit,
    onUpdateCard: (SequenceCard) -> Unit,
    onEditStarted: () -> Unit,
) {
    var editingId by rememberSaveable { mutableStateOf<String?>(null) }
    var editingPlanIndex by rememberSaveable { mutableStateOf<Int?>(null) }
    val editingCard = editingPlanIndex?.let { plan.getOrNull(it) } ?: cards.find { it.id == editingId }
    if (editingCard != null) {
        Text("编辑卡片", style = MaterialTheme.typography.titleMedium)
        Button(onClick = { editingId = null; editingPlanIndex = null }) { Text("取消 / 返回卡片列表") }
        fun saveEdited(edited: SequenceCard) {
            val index = editingPlanIndex
            if (index != null) onPlanChange(plan.mapIndexed { i, card -> if (i == index) edited else card })
            else onUpdateCard(edited)
            editingId = null
            editingPlanIndex = null
        }
        key(editingId, editingPlanIndex) {
            if (editingCard.mode == 'T') {
                TurnCardPanel(false, onSave = { name, turn -> saveEdited(editingCard.copy(name = name, turn = turn)) },
                    onExecute = {}, initialCard = editingCard, editing = true)
            } else {
                SequenceSettingsPanel(if (editingCard.mode == 'P') "放下" else "抓取", editingCard.mode == 'P', false,
                    onExecute = {}, onSave = { name, settings -> saveEdited(editingCard.copy(name = name, settings = settings)) },
                    initialCard = editingCard, editing = true)
            }
        }
        return
    }
    Text("当前编排 ${plan.size}/${SequencePlan.MAX_ITEMS}", style = MaterialTheme.typography.titleMedium)
    if (plan.isEmpty()) Text("从下方卡片库加入抓取、放下或转盘卡片")
    plan.forEachIndexed { index, card ->
        SequenceCardView(card, "${index + 1}. ", onClick = { editingPlanIndex = index; editingId = card.id; onEditStarted() }) {
            Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                Button(onClick = {
                    val next = plan.toMutableList()
                    next[index] = next[index - 1]; next[index - 1] = card; onPlanChange(next)
                }, enabled = index > 0) { Text("上移") }
                Button(onClick = {
                    val next = plan.toMutableList()
                    next[index] = next[index + 1]; next[index + 1] = card; onPlanChange(next)
                }, enabled = index < plan.lastIndex) { Text("下移") }
                Button(onClick = { onPlanChange(plan.filterIndexed { i, _ -> i != index }) }) { Text("移除") }
            }
        }
    }
    Button(onClick = onExecute, enabled = connected && plan.isNotEmpty(), modifier = Modifier.fillMaxWidth()) {
        Text("执行编排")
    }
    Button(onClick = { onPlanChange(emptyList()) }, enabled = plan.isNotEmpty(), modifier = Modifier.fillMaxWidth()) {
        Text("清空编排")
    }
    HorizontalDivider(Modifier.padding(vertical = 12.dp))
    Text("卡片库", style = MaterialTheme.typography.titleMedium)
    if (cards.isEmpty()) Text("在抓取、放下或转盘页填写参数，点击保存为卡片")
    cards.forEach { card ->
        SequenceCardView(card, onClick = { editingId = card.id; editingPlanIndex = null; onEditStarted() }) {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { onPlanChange(plan + card) }, enabled = plan.size < SequencePlan.MAX_ITEMS) { Text("加入流程") }
                Button(onClick = { onDeleteCard(card.id) }) { Text("删除卡片") }
            }
        }
    }
}

@Composable
private fun SequenceCardView(card: SequenceCard, prefix: String = "", onClick: (() -> Unit)? = null, actions: @Composable () -> Unit) {
    val p = card.settings
    Card(Modifier.fillMaxWidth().padding(vertical = 6.dp).clickable(enabled = onClick != null) { onClick?.invoke() }) {
        Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("$prefix${card.name} · ${when (card.mode) { 'A' -> "抓取"; 'T' -> "转盘"; else -> "放下" }}", fontWeight = FontWeight.Bold)
            if (card.mode == 'T') {
                val t = card.turn!!
                Text("转盘 ${t.start}° → ${t.end}°；速度 ${t.dps}°/秒")
            } else {
            Text("目标1 r=${p.r1 / 10.0} mm  z=${p.z1} mm；目标2 r=${p.r2 / 10.0} mm  z=${p.z2} mm")
            Text("r/z合成速度 ${p.combinedRpm} RPM；夹子 ${p.gripperDps}°/s")
            Text("夹子开/合 ${p.openAngle}/${p.closeAngle}°")
            Text(if (card.mode == 'P') "基座：转盘取物 ${p.baseTilt}° → 放置 ${p.baseHome}°"
                else "基座：抓取 ${p.baseHome}° → 转盘存放 ${p.baseTilt}°")
            }
            actions()
        }
    }
}

@Composable
private fun ArmPosePanel(
    connected: Boolean,
    onMove: (Int, Int, Int, Int, Int) -> Unit,
    onInitialize: (Int) -> Unit,
    onHome: () -> Unit,
    onStop: () -> Unit,
) {
    var theta by rememberSaveable { mutableStateOf("0") }
    var r by rememberSaveable { mutableStateOf("0") }
    var z by rememberSaveable { mutableStateOf("0") }
    var rpm by rememberSaveable { mutableStateOf("20") }
    var pulses by rememberSaveable { mutableStateOf("3200") }
    var note by rememberSaveable { mutableStateOf("等待绝对位置指令") }
    Text("机械臂：柱坐标绝对位置", style = MaterialTheme.typography.titleMedium)
    var originBase by rememberSaveable { mutableStateOf("248") }
    DistanceInput("启动基座角度（0–360 度）", originBase, true) { originBase = it }
    Text("首次记录r/z零点；之后可随时更新启动基座角度，零点不变。")
    Button(onClick = {
        val base = originBase.toIntOrNull()
        if (base == null || base !in 0..360) note = "请输入0–360度的初始化基座角度"
        else { onInitialize(base); note = "已提交启动基座角度$base°；首次初始化记录零点，之后保留零点" }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("初始化 / 更新启动基座角度") }
    DistanceInput("转盘 θ（度，0–270）", theta, connected) { theta = it }
    VisibleTextField(value = r, onValueChange = { r = it }, label = { Text("前伸 r（mm，-1000–1000）") },
        enabled = connected, singleLine = true, modifier = Modifier.fillMaxWidth())
    VisibleTextField(value = z, onValueChange = { z = it }, label = { Text("升降 z（mm，-400–400）") },
        enabled = connected, singleLine = true, modifier = Modifier.fillMaxWidth())
    DistanceInput("速度（RPM，5–300）", rpm, connected) { rpm = it }
    DistanceInput("两轴每圈脉冲", pulses, connected) { pulses = it }
    Button(onClick = {
        val angle = theta.toIntOrNull(); val radial = r.toIntOrNull(); val height = z.toIntOrNull()
        val speed = rpm.toIntOrNull(); val ppr = pulses.toIntOrNull()
        if (angle == null || angle !in 0..270 || radial == null || radial !in -1000..1000 ||
            height == null || height !in -400..400 || speed == null || speed !in 5..300 ||
            ppr == null || ppr !in 200..51200) note = "请输入范围内的 θ、r、z、速度和每圈脉冲"
        else { onMove(angle, radial, height, speed, ppr); note = "已提交绝对目标：θ=$angle°，r=$radial mm，z=$height mm" }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("执行绝对位置") }
    Button(onClick = { onHome(); note = "已请求返回启动原点及初始舵机姿态" },
        enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("返回原点状态") }
    Button(onClick = { onStop(); note = "已请求停止机械臂" },
        enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("停止机械臂") }
    if (note.startsWith("请输入") || note.startsWith("请检查")) Text(note, color = MaterialTheme.colorScheme.error)
    Spacer(Modifier.height(16.dp))
}

@Preview(showBackground = true)
@Composable
private fun RemoteControlPreview() {
    AppTheme {
        RemoteControlScreen(
            connected = false,
            connectionText = "未连接",
            onConnect = { _, _ -> },
            onDisconnect = {},
            moveRunning = false,
            moveStatus = "等待定距移动",
            onMove = {},
            onAlign = { _, _, _ -> },
            onRingAlign = { _, _, _, _ -> },
            onJointAlign = { _, _, _, _, _ -> },
            onParallel = { _, _, _, _ -> },
            onStopMotion = {},
            onArmPose = { _, _, _, _, _ -> },
            onHome = {},
            onInitializeOrigin = {},
            onStopArm = {},
            onGrab = {},
            onRelease = {},
            onCancelGrab = {},
            onGripper = { _, _, _ -> },
            onStopGripper = {},
            onServoFinished = { _, _ -> },
        )
    }
}

@Composable
private fun TurnCardPanel(connected: Boolean, onSave: (String, TurnSettings) -> Unit, onExecute: (TurnSettings) -> Unit,
                          initialCard: SequenceCard? = null, editing: Boolean = false) {
    var start by rememberSaveable { mutableStateOf(initialCard?.turn?.start?.toString() ?: "0") }
    var end by rememberSaveable { mutableStateOf(initialCard?.turn?.end?.toString() ?: "120") }
    var speed by rememberSaveable { mutableStateOf(initialCard?.turn?.dps?.toString() ?: "60") }
    var name by rememberSaveable { mutableStateOf(initialCard?.name ?: "转盘") }
    var message by remember { mutableStateOf<String?>(null) }
    fun values(): TurnSettings? = try { TurnSettings(start.trim().toInt(), end.trim().toInt(), speed.trim().toInt()).takeIf { it.valid() } } catch (_: IllegalArgumentException) { null }
    DistanceInput("起始角度（0–270度）", start, true) { start = it }
    DistanceInput("结束角度（0–270度）", end, true) { end = it }
    DistanceInput("速度（1–360度/秒）", speed, true) { speed = it }
    VisibleTextField(value = name, onValueChange = { name = it }, label = { Text("卡片名称") }, modifier = Modifier.fillMaxWidth())
    Button(onClick = {
        val v = values()
        if (v == null || name.trim().length !in 1..40) message = "请检查角度、速度和名称"
        else { onSave(name.trim(), v); message = "已保存" }
    }, modifier = Modifier.fillMaxWidth()) { Text(if (editing) "保存修改" else "保存转盘卡片") }
    if (!editing) Button(onClick = { val v = values(); if (v == null) message = "请检查角度和速度" else onExecute(v) }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("执行转盘") }
    message?.let { Text(it) }
}
