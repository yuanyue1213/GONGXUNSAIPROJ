/* App 启动协议边界：构造 START，解析 AUTO/TASK 为显示状态，不生成运动指令。 */
package com.example.myapplication

internal object StartProtocol {
    data class Status(val text: String, val finished: Boolean, val runId: Long? = null)
    fun frame(id: Long): String {
        require(id in 1..0xFFFFFFFFL)
        return "START,$id\n"
    }
    fun status(line: String, id: Long): Status? {
        if (line == "BRIDGE_ERROR,UART_TX") return Status("启动信号串口发送失败", true)
        val f = line.split(',')
        if (f.size == 3 && f[0] == "TASK" && f[1].toLongOrNull() == id)
            return Status("任务码：${f[2]}", false)
        if (f.size != 8 || f[0] != "AUTO") return null
        /* 过滤其他任务的旧反馈；BUSY 是例外，用于获知控制器当前运行的任务。 */
        if (f[1].toLongOrNull() != id && f[2] != "BUSY") return null
        val state = when (f[2]) {
            "STARTING" -> "正在初始化执行姿态"
            "RUNNING" -> "底盘行驶中"
            "TURNING" -> "底盘转向中"
            "SCANNING" -> "读取任务二维码"
            "WORK" -> "视觉定位与物料取放中"
            "DONE" -> "全流程完成"
            "STOPPED" -> "全流程已停止"
            "BUSY" -> "STM32 正在执行全流程"
            "ERROR" -> "启动失败"
            else -> return null
        }
        val terminal = f[2] in listOf("DONE", "STOPPED", "ERROR")
        return Status("$state · 路线步骤 ${f[3]} · 抓取 ${f[5]} / 放置 ${f[6]}\n${f[7]}", terminal, f[1].toLongOrNull())
    }
}
