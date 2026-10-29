/* 启动连接管理：后台线程处理 Socket，主线程更新界面。
 * ViewModel 保留屏幕旋转期间的连接；App 只发启动帧，运动由 STM32 独立执行。 */
package com.example.myapplication

import android.app.Application
import android.os.Handler
import android.os.Looper
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.AndroidViewModel
import java.net.InetSocketAddress
import java.net.Socket
import java.util.concurrent.Executors

class StartViewModel(application: Application) : AndroidViewModel(application) {
    var busy by mutableStateOf(false)
        private set
    var status by mutableStateOf("等待启动")
        private set
    private val main = Handler(Looper.getMainLooper())
    private val worker = Executors.newSingleThreadExecutor()
    @Volatile private var connection: Socket? = null

    fun start() {
        if (busy) return
        busy = true
        status = "正在连接控制器…"
        val prefs = getApplication<Application>().getSharedPreferences("start", 0)
        /* 持久化单调递增编号，避免快速点击或 App 重启后被 STM32 判断为重复启动。 */
        val request = maxOf(System.currentTimeMillis() / 1000, prefs.getLong("request", 0) + 1)
        if (request !in 1..0xFFFFFFFFL || !prefs.edit().putLong("request", request).commit()) {
            busy = false
            status = "无法保存启动编号，请重试"
            return
        }
        /* 网络操作放到单线程执行器，避免阻塞 UI；界面状态通过 main.post 回到主线程更新。 */
        worker.execute {
            var sent = false
            try {
                Socket().use { socket ->
                    connection = socket
                    socket.connect(InetSocketAddress("192.168.4.1", 3333), 5000)
                    socket.soTimeout = 10000
                    socket.getOutputStream().apply {
                        write(StartProtocol.frame(request).toByteArray(Charsets.US_ASCII))
                        flush()
                    }
                    sent = true
                    main.post { status = "已发送启动请求，等待 STM32 确认…" }
                    val input = socket.getInputStream()
                    val line = StringBuilder()
                    var dropping = false
                    /* 首次跟踪本次请求；收到 BUSY 后可跟随 STM32 正在执行的任务编号。 */
                    var observedRun = request
                    while (true) {
                        val byte = input.read()
                        if (byte < 0) error("控制器断开连接")
                        /* 按换行组帧，超长帧丢弃到下一换行，防止残留片段被当作新状态。 */
                        if (byte != 10) {
                            if (!dropping) {
                                if (line.length < 191) line.append(byte.toChar()) else dropping = true
                            }
                            continue
                        }
                        val frame = if (dropping) "" else line.toString().removeSuffix("\r")
                        line.clear(); dropping = false
                        val state = StartProtocol.status(frame, observedRun) ?: continue
                        state.runId?.let { observedRun = it }
                        socket.soTimeout = 180000
                        main.post { status = state.text }
                        if (state.finished) break
                    }
                }
            /* 已发送后的断链只能说明状态未知，不能据此认为 STM32 已停止或自动重发启动。 */
            } catch (e: Exception) {
                main.post {
                    status = if (sent) "通信中断，任务状态未知；STM32 会独立执行。${e.message.orEmpty()}"
                        else "连接失败，请确认已连接机器人热点。${e.message.orEmpty()}"
                }
            } finally {
                connection = null
                main.post { busy = false }
            }
        }
    }
    override fun onCleared() {
        connection?.close()
        worker.shutdownNow()
    }
}
