package com.example.app

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.background
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
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
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

private const val COMMAND_INTERVAL_MS = 50L

class MainActivity : ComponentActivity() {
    private val commandHandler = Handler(Looper.getMainLooper())
    private var repeatedCommand: Runnable? = null
    private var activeDirection: Char? = null
    private lateinit var robotClient: RobotTcpClient

    private var connectionText by mutableStateOf("未连接")
    private var lastMessage by mutableStateOf("等待连接 ESP32-S3")
    private var isConnected by mutableStateOf(false)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        robotClient = RobotTcpClient(
            onConnectionChanged = { connected, message ->
                runOnUiThread {
                    isConnected = connected
                    connectionText = message
                    if (!connected) cancelRepeatedCommand()
                }
            },
            onMessage = { message -> runOnUiThread { lastMessage = message } },
        )

        enableEdgeToEdge()
        setContent {
            AppTheme {
                RemoteControlScreen(
                    connected = isConnected,
                    connectionText = connectionText,
                    lastMessage = lastMessage,
                    onConnect = { host, port -> robotClient.connect(host, port) },
                    onDisconnect = { stopMotion(); robotClient.disconnect() },
                    onStartMotion = ::startMotion,
                    onStopMotion = ::stopMotion,
                )
            }
        }
    }

    override fun onStop() {
        super.onStop()
        stopMotion()
    }

    override fun onDestroy() {
        stopMotion()
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
}

@Composable
private fun RemoteControlScreen(
    connected: Boolean,
    connectionText: String,
    lastMessage: String,
    onConnect: (String, Int) -> Unit,
    onDisconnect: () -> Unit,
    onStartMotion: (Char) -> Unit,
    onStopMotion: () -> Unit,
) {
    var host by mutableStateOf("192.168.4.1")
    var portText by mutableStateOf("3333")
    var addressError by mutableStateOf<String?>(null)

    Column(
        modifier = Modifier.fillMaxSize().padding(horizontal = 20.dp, vertical = 28.dp),
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
        Spacer(Modifier.height(24.dp))
        Text("ESP32 回复", style = MaterialTheme.typography.labelLarge)
        Text(lastMessage, modifier = Modifier.fillMaxWidth())
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
        if (direction !in charArrayOf('F', 'B', 'L', 'R', 'S')) return
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
            onConnect = { _, _ -> },
            onDisconnect = {},
            onStartMotion = {},
            onStopMotion = {},
        )
    }
}
