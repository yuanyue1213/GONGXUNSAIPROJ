package com.example.app

internal const val DEFAULT_TURN_CORRECTION = 0.96

internal data class AutonomousRoutePoint(val x: Int, val y: Int) {
    fun valid(): Boolean {
        if (x !in 150..2250 || y !in 150..2250) return false
        return !(((x > 400 && x < 1150) || (x > 1250 && x < 2000)) &&
            ((y > 400 && y < 1150) || (y > 1250 && y < 2000)))
    }
}
internal object AutonomousRoute {
    val names = listOf("起步前进", "右上途经点NE", "扫码工位Q", "返回NE", "第一批原料区R",
        "前往粗加工：中心C", "第一批粗加工P", "前往暂存：中心C", "第一批暂存T", "左上途经点NW",
        "第二批原料区R", "前往粗加工：中心C", "第二批粗加工P", "前往码垛：中心C", "第二批暂存码垛T",
        "驶向终点：中心C", "右侧通道Q", "右下途经点SE", "终点接近点", "终点S2")
    val defaults = listOf(2250 to 2200, 2200 to 2200, 2200 to 1200, 2200 to 2200, 1200 to 2200,
        1200 to 1200, 1200 to 300, 1200 to 1200, 300 to 1200, 300 to 2200,
        1200 to 2200, 1200 to 1200, 1200 to 300, 1200 to 1200, 300 to 1200,
        1200 to 1200, 2200 to 1200, 2200 to 300, 2250 to 300, 2250 to 150).map { AutonomousRoutePoint(it.first, it.second) }
}
internal data class AutonomousArmParameters(val role: Char, val slot: Int, val settings: SequenceSettings) {
    fun valid() = role in "RGPTK" && slot in 1..3 && settings.valid()
    fun frame(config: Long) = "AUTO_ARM,$config,$role,$slot," + settings.frame(config, if (role in "RG") 'A' else 'P')
}
internal fun validAutonomousCode(code: String): Boolean {
    val groups = code.split('+')
    return groups.size == 4 && groups.all { it.length == 3 } &&
        groups[0].all { it in '1'..'6' } && groups[2].all { it in '1'..'6' } &&
        groups[0].toSet().size == 3 && groups[0].toSet() == groups[2].toSet() &&
        groups[1].toList().sorted() == "123".toList() && groups[3].toList().sorted() == "123".toList()
}

internal data class AutonomousSettings(
    val forwardPpm: Int, val lateralPpm: Int, val rpm: Int, val fineRpm: Int,
    val targetX: Float, val targetY: Float, val processMap: String, val storageMap: String,
    val slots: List<Int>,
    val visionEnabled: Boolean = false, val armEnabled: Boolean = false, val manualCode: String = "",
    val route: List<AutonomousRoutePoint> = AutonomousRoute.defaults,
    val wheelbaseMm: Double = 190.0, val trackMm: Double = 251.4,
    val turnCorrection: Double = DEFAULT_TURN_CORRECTION, val turnRpm: Int = 20,
    val wheelPulses: Int = 3200,
) {
    fun turnPulses(): Int? {
        if (!wheelbaseMm.isFinite() || wheelbaseMm !in 50.0..2000.0 ||
            !trackMm.isFinite() || trackMm !in 50.0..2000.0 ||
            !turnCorrection.isFinite() || turnCorrection !in 0.1..10.0 || turnRpm !in 5..60) return null
        val pulses = Math.round(Math.PI * (wheelbaseMm + trackMm) / 4.0 * forwardPpm / 1000.0 * turnCorrection)
        return pulses.takeIf { it in 1L..500000L }?.toInt()
    }
    fun valid() = wheelPulses in 200..51200 && forwardPpm in 1..1000000 && lateralPpm in 1..1000000 && rpm in 5..300 && (!visionEnabled || (fineRpm in 5..300 &&
        targetX.isFinite() && targetY.isFinite() && targetX >= 0 && targetX <= 511.9f && targetY >= 0 && targetY <= 319.9f &&
        processMap.toList().sorted() == "123".toList() && storageMap.toList().sorted() == "123".toList())) &&
        (!armEnabled || (slots.size == 3 && slots.all { it in 0..270 } && slots.distinct().size == 3)) &&
        (visionEnabled || !armEnabled || validAutonomousCode(manualCode)) &&
        route.size == 20 && route.all { it.valid() } && route.last() == AutonomousRoute.defaults.last() && turnPulses() != null
    fun frames(config: Long, run: Long, parameters: List<AutonomousArmParameters>): List<String> {
        val required = "RGPTK".flatMap { role -> (1..3).map { role to it } }.toSet()
        if (!valid() || config !in 1..0xFFFFFFFFL || run !in 1..0xFFFFFFFFL ||
            (armEnabled && (parameters.size != 15 || parameters.any { !it.valid() } ||
                parameters.map { it.role to it.slot }.toSet() != required))) return emptyList()
        val x = java.lang.String.format(java.util.Locale.US, "%.1f", if (visionEnabled) targetX else 256f)
        val y = java.lang.String.format(java.util.Locale.US, "%.1f", if (visionEnabled) targetY else 160f)
        val fine = if (visionEnabled) fineRpm else 10
        val mp = if (visionEnabled) processMap else "321"
        val mt = if (visionEnabled) storageMap else "321"
        val angles = if (armEnabled) slots else listOf(0,90,180)
        val code = if (!visionEnabled && armEnabled) manualCode else "-"
        return listOf("AUTO_CFG,$config,$forwardPpm,$lateralPpm,$rpm,$fine,$x,$y,$mp,$mt,${angles.joinToString(",")}\n",
            "AUTO_OPTIONS,$config,${if (visionEnabled) 1 else 0},${if (armEnabled) 1 else 0},$code\n",
            "AUTO_DRIVE,$config,${turnPulses()},$turnRpm,$wheelPulses\n") +
            route.mapIndexed { index, point -> "AUTO_ROUTE,$config,$index,${point.x},${point.y}\n" } +
            (if (armEnabled) parameters.map { it.frame(config) } else emptyList()) + "AUTO_START,$run,$config\n"
    }
}

internal object AutonomousStatus {
    fun text(line: String): String {
        val fields = line.split(',')
        if (fields.size == 3 && fields[0] == "TASK") return "任务码：${fields[2]}"
        if (fields.size != 8 || fields[0] != "AUTO") return line
        val state = when (fields[2]) {
            "STARTING" -> "正在检查执行状态"
            "RUNNING" -> "正在行驶"
            "TURNING" -> "正在原地转向"
            "DRIVE" -> "已接收行驶与转向配置"
            "SCANNING" -> "正在读取二维码"
            "WORK" -> "正在取放物料"
            "CALIBRATING" -> "正在视觉校准"
            "DONE" -> "任务完成，已到S2"
            "STOPPED" -> "自动任务已停止"
            "BUSY" -> "自动任务正在执行"
            "CONFIGURED" -> "已接收路线参数"
            "PROFILE", "PARAMETERS" -> "已接收机械臂参数"
            "OPTIONS" -> "已接收执行模式"
            "ROUTE" -> "已接收路线坐标"
            else -> "任务配置失败"
        }
        val reason = when (fields[7]) {
            "STM_NOT_READY_INITIALIZE_ORIGIN" -> "请先初始化机械臂零点并确认空闲"
            "STM_NOT_IDLE" -> "执行器忙碌，请先停止并等待空闲"
            "NOT_CONFIGURED_OR_DUPLICATE" -> "配置不完整或重复启动"
            "TASK_CODE_REQUIRED", "TASK_CODE" -> "请填写有效的四组任务码"
            "ROUTE" -> "路线坐标无效或超出车道"
            "OPTIONS" -> "执行模式配置无效"
            "DRIVE" -> "行驶细分脉冲或转向参数无效"
            "VISION_TIMEOUT" -> "等待视觉识别超时"
            "VISION_OUT_OF_APPROACH_RANGE" -> "目标超出对准范围或未能收敛"
            "LANE_BOUNDARY" -> "预计运动超出灰色车道"
            "STM_ACTION_FAILED" -> "STM32报告动作失败或忙碌"
            "ACTION_TIMEOUT" -> "等待动作完成超时"
            "IMU_NOT_READY" -> "JY901S未就绪或车辆未静止，请检查接线和串口配置"
            "IMU_TIMEOUT" -> "JY901S数据超时，已停车"
            "IMU_RESET" -> "JY901S连接或角度参考已变化，请重新启动任务"
            "TURN_NO_PROGRESS" -> "转向没有检测到角度进展"
            "IMU_NO_REFERENCE" -> "尚未记录启动参考朝向"
            "IMU_DIRECTION" -> "航向变化与指令不一致，请检查传感器安装与轮子方向"
            "TURN_TIMEOUT" -> "陀螺仪转向超时"
            "DRIVE_TIMEOUT" -> "直行距离控制超时"
            "DRIVE_NO_PROGRESS" -> "直行未检测到编码器距离进展"
            "ODOM_READ" -> "读取底盘电机编码器失败，已停车"
            "ODOM_DIRECTION" -> "编码器方向与前后指令不一致"
            "MOTOR_FAULT", "MOTOR_COMMAND", "MOTOR_ENABLE", "MOTOR_STOP" -> "底盘电机故障或通信失败"
            "DISCONNECTED" -> "连接已断开"
            "USER", "USER_STOP" -> "手动停止"
            "0", "S1_HEAD_DOWN", "CARD_DONE", "S2" -> ""
            else -> fields[7]
        }
        return "$state · 抓取动作${fields[5]} / 放置动作${fields[6]}${if (reason.isEmpty()) "" else " · $reason"}"
    }
}
