package com.example.app

/** A TCP connection alone does not prove STM32 has answered. Old or unrelated
 * sequence IDs must not satisfy the current diagnostic request. */
internal class LinkProbe {
    private var id = 0L
    private var esp = false
    private var stm = false
    fun start(sequence: Long): String {
        id = sequence; esp = false; stm = false
        return "正在检测 App → ESP32 → STM32 回包…"
    }
    fun receive(line: String): String? {
        val f = line.split(',')
        if (id == 0L || f.size != 3 || f[1].toLongOrNull() != id) return null
        if (f[0] == "PONG" && f[2] == "STM32") {
            stm = true
            return "双向通信正常：STM32回应已通过ESP32返回App（不代表动作完成）"
        }
        if (f[0] == "LINK" && f[2] == "ESP_OK" && !stm) {
            esp = true
            return "ESP32已回应，正在等待STM32回包…"
        }
        return null
    }
    fun timeout(sequence: Long): String? {
        if (sequence != id || stm) return null
        return if (esp) "ESP32已回应，3秒内未收到STM32：检查PB13→GPIO18回线、共地和两端固件"
            else "3秒内未收到ESP32检测回应：检查Wi-Fi/TCP及ESP32固件是否更新"
    }
    fun clear() { id = 0; esp = false; stm = false }
}
