package com.example.app

/** Reports an observed communication/control failure, not proof of a damaged motor. */
object MotorFeedback {
    fun text(line: String): Pair<String, String>? {
        val fields = line.split(',')
        if (fields.size != 4 || fields[0] != "MOTOR_DIAG") return null
        val id = fields[2].toIntOrNull() ?: return null
        val motor = when (fields[1]) {
            "CHASSIS" -> when (id) {
                0 -> "底盘总线（无法确定单个电机）"
                1 -> "底盘1号·左前轮"
                2 -> "底盘2号·右前轮"
                3 -> "底盘3号·右后轮"
                4 -> "底盘4号·左后轮"
                else -> return null
            }
            "ARM" -> when (id) {
                1 -> "机械臂1号·升降轴"
                2 -> "机械臂2号·伸缩轴"
                else -> return null
            }
            else -> return null
        }
        val operation = when (fields[3]) {
            "POSITION_READ" -> "位置读取失败"
            "ORIGIN_READ" -> "原点位置读取失败"
            "FIRMWARE_READ" -> "驱动器型号读取失败"
            "STATUS_READ" -> "状态读取失败"
            "DRIVER_FAULT" -> "驱动状态异常（未使能或堵转保护）"
            "ENABLE" -> "使能失败"
            "STOP" -> "停车指令应答失败"
            "SPEED_COMMAND" -> "组合速度指令失败"
            "POSITION_COMMAND" -> "组合位置指令失败"
            "COMMAND" -> "控制指令失败"
            "MOVE_TIMEOUT" -> "运动超时"
            else -> return null
        }
        return "${fields[1]}:$id" to "$motor：$operation"
    }
}
