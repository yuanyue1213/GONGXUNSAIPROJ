package com.example.app

internal object ImuFeedback {
    fun text(line: String): String {
        val f = line.split(',')
        if (f.size == 5 && f[2] == "READY") return "JY901S：展开角度 ${f[3]}°，Z角速度 ${f[4]}°/s"
        val status = when (f.getOrNull(3)) {
            "INIT_SEARCH" -> "正在等待模块启动并以9600查询参数（约5秒）"
            "INIT_CONFIG" -> "正在配置10Hz角度和角速度输出，保持9600"
            "INIT_VERIFY" -> "正在读回设置并验证数据"
            "INIT_NO_REPLY" -> "9600查询结束，未获得可确认波特率的寄存器回应"
            "INIT_UART_ERROR" -> "STM32 USART3初始化或切换波特率失败"
            "INIT_TX_ERROR" -> "STM32 USART3发送配置指令失败"
            "INIT_VERIFY_TIMEOUT" -> "修改输出参数后未收到参数读回"
            "INIT_PARAM_MISMATCH" -> "模块返回的参数与目标设置不一致"
            "INIT_SAVE_TIMEOUT" -> "保存设置后未收到参数读回"
            "INIT_SAVE_MISMATCH" -> "保存后读回参数不一致"
            "INIT_STREAM_TIMEOUT" -> "参数已验证，但角度或角速度数据不足/超时"
            "INIT_FAILED" -> "初始化失败，需进一步检查串口收发"
            else -> "未获得有效的新鲜数据"
        }
        val detail = if (f.size == 16) "\n当前波特率${f[4]}；累计接收${f[5]}字节；有效帧${f[6]}；寄存器回应${f[7]}；角度帧${f[8]}；角速度帧${f[9]}；串口错误${f[10]}；校验失败${f[11]}。\n最近有效帧波特率${f[12]}；读回RSW=${f[13]}、RRATE=${f[14]}、BAUD=${f[15]}（目标12、6、2）。"
            else ""
        return "JY901S：$status$detail"
    }
}
