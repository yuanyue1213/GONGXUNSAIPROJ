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
import java.io.BufferedReader
import java.io.BufferedWriter
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.net.InetSocketAddress
import java.net.Socket
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.roundToInt

private const val COMMAND_INTERVAL_MS = 50L

class MainActivity : ComponentActivity() {
    private val commandHandler = Handler(Looper.getMainLooper())
    private var repeatedCommand: Runnable? = null
    private var activeDirection: Char? = null
    private var repeatedLiftCommand: Runnable? = null
    private var activeLiftDirection: Char? = null
    private var repeatedForeAftCommand: Runnable? = null
    private var activeForeAftDirection: Char? = null
    private val pendingServoValues = mutableMapOf<Char, Int>()
    private val pendingServoTasks = mutableMapOf<Char, Runnable>()
    private lateinit var robotClient: RobotTcpClient

    private var connectionText by mutableStateOf("未连接")
    private var lastMessage by mutableStateOf("等待连接 ESP32-S3")
    private var lastStmMessage by mutableStateOf("尚未收到 STM32 回包")
    private var isConnected by mutableStateOf(false)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        robotClient = RobotTcpClient(
            onConnectionChanged = { connected, message ->
                runOnUiThread {
                    isConnected = connected
                    connectionText = message
                    if (!connected) {
                        cancelRepeatedCommand()
                        cancelRepeatedLiftCommand()
                        cancelRepeatedForeAftCommand()
                        cancelPendingServos()
                    }
                }
            },
            onMessage = { message ->
                runOnUiThread {
                    lastMessage = message
                    if (message.startsWith("STM,")) lastStmMessage = message
                }
            },
        )

        enableEdgeToEdge()
        setContent {
            AppTheme {
                RemoteControlScreen(
                    connected = isConnected,
                    connectionText = connectionText,
                    lastMessage = lastMessage,
                    lastStmMessage = lastStmMessage,
                    onConnect = { host, port ->
                        lastStmMessage = "尚未收到 STM32 回包"
                        robotClient.connect(host, port)
                    },
                    onDisconnect = { stopMotion(); stopLift(); stopForeAft(); robotClient.disconnect() },
                    onDiagnose = { robotClient.sendDiagnostic() },
                    onStartMotion = ::startMotion,
                    onStopMotion = ::stopMotion,
                    onStartLift = ::startLift,
                    onStopLift = ::stopLift,
                    onStartForeAft = ::startForeAft,
                    onStopForeAft = ::stopForeAft,
                    onServoChange = ::queueServoAngle,
                    onServoFinished = ::sendServoAngle,
                )
            }
        }
    }

    override fun onStop() {
        super.onStop()
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

    private fun startMotion(direction: Char) {
        if (!isConnected) return
        cancelRepeatedCommand()
        activeDirection = direction
        robotClient.sendMotion(direction)
        repeatedCommand = object : Runnable {
            override fun run() {
                if (activeDirection == direction && isConnected) {
                    robotClient.sendMotion(direction)
                    commandHandler.postDelayed(this, COMMAND_INTERVAL_MS)
                }
            }
        }.also { commandHandler.postDelayed(it, COMMAND_INTERVAL_MS) }
    }

    private fun stopMotion() {
        cancelRepeatedCommand()
        activeDirection = null
        robotClient.sendMotion('S')
    }

    private fun cancelRepeatedCommand() {
        repeatedCommand?.let(commandHandler::removeCallbacks)
        repeatedCommand = null
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

    private fun startForeAft(direction: Char) {
        if (!isConnected) return
        cancelRepeatedForeAftCommand()
        activeForeAftDirection = direction
        robotClient.sendMotion(direction)
        repeatedForeAftCommand = object : Runnable {
            override fun run() {
                if (activeForeAftDirection == direction && isConnected) {
                    robotClient.sendMotion(direction)
                    commandHandler.postDelayed(this, COMMAND_INTERVAL_MS)
                }
            }
        }.also { commandHandler.postDelayed(it, COMMAND_INTERVAL_MS) }
    }

    private fun stopForeAft() {
        cancelRepeatedForeAftCommand()
        activeForeAftDirection = null
        robotClient.sendMotion('Q')
    }

    private fun cancelRepeatedForeAftCommand() {
        repeatedForeAftCommand?.let(commandHandler::removeCallbacks)
        repeatedForeAftCommand = null
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
    lastMessage: String,
    lastStmMessage: String,
    onConnect: (String, Int) -> Unit,
    onDisconnect: () -> Unit,
    onDiagnose: () -> Unit,
    onStartMotion: (Char) -> Unit,
    onStopMotion: () -> Unit,
    onStartLift: (Char) -> Unit,
    onStopLift: () -> Unit,
    onStartForeAft: (Char) -> Unit,
    onStopForeAft: () -> Unit,
    onServoChange: (Char, Int) -> Unit,
    onServoFinished: (Char, Int) -> Unit,
) {
    var host by mutableStateOf("192.168.4.1")
    var portText by mutableStateOf("3333")
    var addressError by mutableStateOf<String?>(null)

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
        Text("按住方向键运动，松手停车", style = MaterialTheme.typography.titleMedium)
        Spacer(Modifier.height(16.dp))
        MotionButton("↑", 'F', connected, onStartMotion, onStopMotion)
        Spacer(Modifier.height(8.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            MotionButton("←", 'L', connected, onStartMotion, onStopMotion)
            Button(
                onClick = onStopMotion,
                enabled = connected,
                modifier = Modifier.size(96.dp),
                colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error),
            ) { Text("停止", color = MaterialTheme.colorScheme.onError) }
            MotionButton("→", 'R', connected, onStartMotion, onStopMotion)
        }
        Spacer(Modifier.height(8.dp))
        MotionButton("↓", 'B', connected, onStartMotion, onStopMotion)
        Spacer(Modifier.height(16.dp))
        HorizontalDivider()
        Spacer(Modifier.height(12.dp))
        Text("机械臂升降：按住运动，松手停止", style = MaterialTheme.typography.titleMedium)
        Spacer(Modifier.height(8.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            MotionButton("升", 'U', connected, onStartLift, onStopLift)
            MotionButton("降", 'D', connected, onStartLift, onStopLift)
        }
        Spacer(Modifier.height(12.dp))
        Text("机械臂前后：按住运动，松手停止", style = MaterialTheme.typography.titleMedium)
        Spacer(Modifier.height(8.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            MotionButton("前", 'E', connected, onStartForeAft, onStopForeAft)
            MotionButton("后", 'C', connected, onStartForeAft, onStopForeAft)
        }
        Spacer(Modifier.height(16.dp))
        HorizontalDivider()
        Spacer(Modifier.height(12.dp))
        Text("舵机角度（首次拖动后生效）", style = MaterialTheme.typography.titleMedium)
        ServoAngleSlider("夹子 · PC8", 'G', 270, connected, onServoChange, onServoFinished)
        ServoAngleSlider("转盘 · PA8", 'T', 270, connected, onServoChange, onServoFinished)
        ServoAngleSlider("基座 · PC6", 'B', 360, connected, onServoChange, onServoFinished)
        Spacer(Modifier.height(16.dp))
        Button(onClick = onDiagnose, enabled = connected, modifier = Modifier.fillMaxWidth()) {
            Text("检测 USART2 与电机 ID 1/2")
        }
        Spacer(Modifier.height(8.dp))
        Text("STM32 执行 / 诊断", style = MaterialTheme.typography.labelLarge)
        Text(lastStmMessage, modifier = Modifier.fillMaxWidth())
        Spacer(Modifier.height(8.dp))
        Text("ESP32 回复", style = MaterialTheme.typography.labelLarge)
        Text(lastMessage, modifier = Modifier.fillMaxWidth())
    }
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
    var angle by remember(channel) { mutableStateOf(maximum / 2) }
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

private class RobotTcpClient(
    private val onConnectionChanged: (Boolean, String) -> Unit,
    private val onMessage: (String) -> Unit,
) : AutoCloseable {
    private val connectionExecutor = Executors.newSingleThreadExecutor()
    private val sendExecutor = Executors.newSingleThreadExecutor()
    private val sequence = AtomicLong(0)
    private val lock = Any()

    @Volatile private var socket: Socket? = null
    @Volatile private var writer: BufferedWriter? = null

    fun connect(host: String, port: Int) {
        disconnect()
        onConnectionChanged(false, "正在连接 $host:$port…")
        connectionExecutor.execute {
            val newSocket = Socket()
            try {
                newSocket.connect(InetSocketAddress(host, port), 3_000)
                val newWriter = BufferedWriter(OutputStreamWriter(newSocket.getOutputStream(), Charsets.US_ASCII))
                val reader = BufferedReader(InputStreamReader(newSocket.getInputStream(), Charsets.US_ASCII))
                synchronized(lock) {
                    socket = newSocket
                    writer = newWriter
                }
                onConnectionChanged(true, "已连接 $host:$port")
                while (true) {
                    val line = reader.readLine() ?: break
                    onMessage(line)
                }
            } catch (error: Exception) {
                onConnectionChanged(false, "连接错误：${error.message ?: error.javaClass.simpleName}")
            } finally {
                synchronized(lock) {
                    if (socket === newSocket) {
                        socket = null
                        writer = null
                    }
                }
                newSocket.close()
                onConnectionChanged(false, "已断开")
            }
        }
    }

    fun sendMotion(direction: Char) {
        if (direction !in charArrayOf('F', 'B', 'L', 'R', 'S', 'U', 'D', 'H', 'E', 'C', 'Q')) return
        val command = "CMD,${sequence.incrementAndGet()},$direction\n"
        sendExecutor.execute {
            val activeWriter = synchronized(lock) { writer } ?: return@execute
            try {
                synchronized(activeWriter) {
                    activeWriter.write(command)
                    activeWriter.flush()
                }
            } catch (_: Exception) {
                disconnect()
            }
        }
    }

    fun sendServo(channel: Char, angle: Int) {
        val maximum = if (channel == 'B') 360 else 270
        if (channel !in charArrayOf('G', 'T', 'B') || angle !in 0..maximum) return
        val command = "SERVO,${sequence.incrementAndGet()},$channel,$angle\n"
        sendExecutor.execute {
            val activeWriter = synchronized(lock) { writer } ?: return@execute
            try {
                synchronized(activeWriter) {
                    activeWriter.write(command)
                    activeWriter.flush()
                }
            } catch (_: Exception) {
                disconnect()
            }
        }
    }

    fun sendDiagnostic() {
        val command = "DIAG,${sequence.incrementAndGet()}\n"
        sendExecutor.execute {
            val activeWriter = synchronized(lock) { writer } ?: return@execute
            try {
                synchronized(activeWriter) {
                    activeWriter.write(command)
                    activeWriter.flush()
                }
            } catch (_: Exception) {
                disconnect()
            }
        }
    }

    fun disconnect() {
        val activeSocket = synchronized(lock) {
            writer = null
            socket.also { socket = null }
        }
        activeSocket?.close()
    }

    override fun close() {
        disconnect()
        connectionExecutor.shutdownNow()
        sendExecutor.shutdownNow()
    }
}

@Preview(showBackground = true)
@Composable
private fun RemoteControlPreview() {
    AppTheme {
        RemoteControlScreen(
            connected = false,
            connectionText = "未连接",
            lastMessage = "等待连接 ESP32-S3",
            lastStmMessage = "尚未收到 STM32 回包",
            onConnect = { _, _ -> },
            onDisconnect = {},
            onDiagnose = {},
            onStartMotion = {},
            onStopMotion = {},
            onStartLift = {},
            onStopLift = {},
            onStartForeAft = {},
            onStopForeAft = {},
            onServoChange = { _, _ -> },
            onServoFinished = { _, _ -> },
        )
    }
}
