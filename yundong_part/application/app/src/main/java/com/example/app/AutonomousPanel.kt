package com.example.app

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp

private val armFieldLabels = linkedMapOf("r1" to "第一定位点 R（mm，一位小数）", "z1" to "第一定位点 Z（mm）",
    "r2" to "第二定位点 R（mm，一位小数）", "z2" to "第二定位点 Z（mm）", "rpm" to "合成速度 RPM（5–600）",
    "gripper" to "夹爪速度（°/s）", "open" to "夹爪张开角度（°）", "close" to "夹爪闭合角度（°）",
    "home" to "工位基座角度（°）", "tilt" to "车载存放基座角度（°）")
private fun armDefaults(role: Char) = mapOf("r1" to if (role in "RG") "110.0" else "20.0",
    "z1" to if (role in "RG") "-50" else "-40", "r2" to if (role in "RG") "20.0" else "110.0",
    "z2" to if (role == 'K') "10" else if (role in "RG") "-40" else "-50", "rpm" to "20",
    "gripper" to "60", "open" to "60", "close" to "0", "home" to "248", "tilt" to "140")

@Composable
internal fun AutonomousPanel(connected: Boolean, running: Boolean, status: String, taskCode: String,
                             onStart: (AutonomousSettings, List<AutonomousArmParameters>) -> Unit,
                             onStop: () -> Unit) {
    val context = LocalContext.current
    val preferences = remember {
        context.getSharedPreferences("full_process_parameters", 0).also { prefs ->
            if (!prefs.getBoolean("wheel_77_defaults_migrated", false)) {
                val edit = prefs.edit().putBoolean("wheel_77_defaults_migrated", true)
                for (key in listOf("forward", "lateral")) {
                    if (prefs.getString(key, null)?.toIntOrNull() == 9889)
                        edit.putString(key, "13228")
                }
                edit.apply()
            }
            if (!prefs.getBoolean("forward_15_to_11_migrated", false)) {
                val edit = prefs.edit().putBoolean("forward_15_to_11_migrated", true)
                if (prefs.getString("forward", null)?.toIntOrNull() == 13228)
                    edit.putString("forward", "18039")
                edit.apply()
            }
            if (!prefs.getBoolean("forward_30_to_43_migrated", false)) {
                val edit = prefs.edit().putBoolean("forward_30_to_43_migrated", true)
                prefs.getString("forward", null)?.toIntOrNull()?.takeIf { it in 1..1000000 }?.let {
                    edit.putString("forward", ((it.toLong() * 30L + 21L) / 43L).coerceAtLeast(1L).toString())
                }
                edit.apply()
            }
            if (!prefs.getBoolean("turn_calibration_096_migrated", false)) {
                val edit = prefs.edit().putBoolean("turn_calibration_096_migrated", true)
                if (prefs.getString("turnCorrection", null)?.toDoubleOrNull() == 1.0)
                    edit.putString("turnCorrection", DEFAULT_TURN_CORRECTION.toString())
                edit.apply()
            }
            if (!prefs.getBoolean("turn_speed_40_migrated", false)) {
                val edit = prefs.edit().putBoolean("turn_speed_40_migrated", true)
                if (prefs.getString("turnRpm", null)?.toIntOrNull() == 20) edit.putString("turnRpm", "40")
                edit.apply()
            }
        }
    }
    val values = remember { mutableStateMapOf<String, String>().apply {
        preferences.all.forEach { (key, value) -> if (value is String) put(key, value) }
    } }
    fun value(key: String, fallback: String) = values[key] ?: fallback
    fun put(key: String, text: String) { values[key] = text; preferences.edit().putString(key, text).apply() }
    var vision by remember { mutableStateOf(preferences.getBoolean("vision", false)) }
    var arm by remember { mutableStateOf(preferences.getBoolean("arm", false)) }
    var section by rememberSaveable { mutableIntStateOf(0) }
    var routeIndex by rememberSaveable { mutableIntStateOf(0) }
    var roleIndex by rememberSaveable { mutableIntStateOf(0) }
    var slot by rememberSaveable { mutableIntStateOf(1) }
    var error by remember { mutableStateOf<String?>(null) }
    val roles = listOf("原料抓取", "粗加工取回", "粗加工放置", "第一批暂存", "第二批码垛")

    fun armParameters(role: Char, slot: Int): AutonomousArmParameters? {
        val defaults = armDefaults(role)
        fun p(field: String) = value("arm_${role}${slot}_$field", defaults.getValue(field))
        return SequenceSettings.parse(p("r1"), p("z1"), p("r2"), p("z2"), p("rpm"), p("gripper"),
            open = p("open"), close = p("close"), baseHome = p("home"), baseTilt = p("tilt"))
            ?.let { AutonomousArmParameters(role, slot, it) }
    }

    Text("全流程 · 车身坐标路线 · 两批搬运", style = MaterialTheme.typography.titleMedium)
    Text("开始时车尾朝向另一个启停区。所有前后左右均以车自身为准，左右转为相对当前朝向转90°。中断重启前请恢复起始位置和朝向。")
    Text("按下列25步顺序执行，工位保持当前朝向；两次加工区均先放置三件，再抓回三件，最后堆叠。动作完成后间隔100ms。")
    Text(status, modifier = Modifier.fillMaxWidth())
    Text("任务码：${taskCode.ifEmpty { if (vision) "等待扫码" else if (arm) "使用下方手动任务码" else "底盘模式不使用" }}")
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text("视觉校准")
        Switch(checked = vision, enabled = !running, onCheckedChange = {
            vision = it; preferences.edit().putBoolean("vision", it).apply()
        })
    }
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text("执行机械臂动作")
        Switch(checked = arm, enabled = !running, onCheckedChange = {
            arm = it; preferences.edit().putBoolean("arm", it).apply()
        })
    }
    Text(when {
        vision && arm -> "完整流程：扫码、工位校准、两批取放及码垛。"
        vision -> "底盘＋视觉：扫码并校准工位，不操作机械臂、夹爪或转盘。"
        arm -> "底盘＋机械臂：使用手动任务码，按配置位置取放，不进行视觉校准。"
        else -> "仅底盘：沿路线行驶，不扫码、不校准、不执行机械臂动作，无需初始化机械臂零点。"
    })
    if (arm) Text("启用机械臂前，请在机械臂页初始化零点，并确认工位与车载槽位参数。")
    if (arm && !vision) OutlinedTextField(value("taskCode", "156+123+516+231"), { put("taskCode", it) },
        label = { Text("手动任务码（四组）") }, enabled = !running, modifier = Modifier.fillMaxWidth())
    TabRow(selectedTabIndex = section) {
        listOf("路线", "视觉", "机械臂").forEachIndexed { index, label ->
            Tab(selected = section == index, enabled = !running, onClick = { section = index }, text = { Text(label) })
        }
    }
    Spacer(Modifier.height(8.dp))
    when (section) {
        0 -> {
            listOf(Triple("forward", "前进每米脉冲", "12585"),
                Triple("lateral", "起步横移每米脉冲", "13228"),
                Triple("rpm", "路线速度 RPM（5–1200）", "45"),
                Triple("wheelPulses", "底盘电机每圈脉冲（须与电机细分一致）", "3200"),
                Triple("turnRpm", "陀螺仪转向速度上限 RPM（5–120）", "40")).forEach { (key, label, initial) ->
                OutlinedTextField(value(key, initial), { put(key, it) }, label = { Text(label) }, enabled = !running, modifier = Modifier.fillMaxWidth())
            }
            Text("JY901S只用于转向（固定9600、10Hz），启动时需在线并静止。前后左右按标定脉冲执行四轮同步位置运动，不用陀螺仪纠偏；确认四轮到位后衔接下一步。每圈脉冲须与电机设置一致。")
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                OutlinedButton(onClick = { routeIndex-- }, enabled = !running && routeIndex > 0) { Text("上一点") }
                OutlinedButton(onClick = { routeIndex++ }, enabled = !running && routeIndex < AutonomousRoute.names.lastIndex) { Text("下一点") }
            }
            Text("${routeIndex + 1}/${AutonomousRoute.names.size} · ${AutonomousRoute.names[routeIndex]}")
        }
        1 -> {
            Text(if (vision) "按实车相机与夹爪位置填写。" else "视觉校准已关闭，此处参数不参与本次执行。")
            listOf(Triple("fine", "对准速度 RPM（5–300）", "10"), Triple("targetX", "目标图像 X", "256.0"),
                Triple("targetY", "目标图像 Y", "160.0"), Triple("processMap", "粗加工画面左中右编号", "321"),
                Triple("storageMap", "暂存画面左中右编号", "321")).forEach { (key, label, initial) ->
                OutlinedTextField(value(key, initial), { put(key, it) }, label = { Text(label) }, enabled = vision && !running, modifier = Modifier.fillMaxWidth())
            }
        }
        2 -> {
            Text(if (arm) "直接填写各工位、各槽位参数，与调试卡片无关联。" else "机械臂动作已关闭，此处参数不参与本次执行。")
            OutlinedTextField(value("slots", "0,90,180"), { put("slots", it) }, label = { Text("车载槽1/2/3转盘角度") },
                enabled = arm && !running, modifier = Modifier.fillMaxWidth())
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                OutlinedButton(onClick = { roleIndex = (roleIndex + 4) % 5 }, enabled = !running) { Text("上一工位") }
                OutlinedButton(onClick = { roleIndex = (roleIndex + 1) % 5 }, enabled = !running) { Text("下一工位") }
            }
            Text(roles[roleIndex], style = MaterialTheme.typography.titleMedium)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                (1..3).forEach { number -> FilterChip(selected = slot == number, enabled = !running,
                    onClick = { slot = number }, label = { Text("车载槽$number") }) }
            }
            Text(if (roleIndex < 2) "第一点：外部取料；第二点：车载存放。" else "第一点：车载取料；第二点：外部放置。")
            val role = "RGPTK"[roleIndex]
            val defaults = armDefaults(role)
            armFieldLabels.forEach { (field, label) ->
                val key = "arm_${role}${slot}_$field"
                OutlinedTextField(value(key, defaults.getValue(field)), { put(key, it) }, label = { Text(label) },
                    enabled = arm && !running, modifier = Modifier.fillMaxWidth())
            }
        }
    }
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(enabled = connected && !running, modifier = Modifier.fillMaxWidth(), onClick = start@ {
        val parameters = mutableListOf<AutonomousArmParameters>()
        if (arm) for ((index, role) in "RGPTK".withIndex()) for (number in 1..3) {
            val parsed = armParameters(role, number)
            if (parsed == null) { roleIndex = index; slot = number; section = 2; error = "${roles[index]} · 槽$number 参数无效"; return@start }
            parameters.add(parsed)
        }
        val settings = try {
            AutonomousSettings(value("forward","12585").toInt(), value("lateral","13228").toInt(), value("rpm","45").toInt(),
                if (vision) value("fine","10").toInt() else 10,
                if (vision) value("targetX","256.0").toFloat() else 256f, if (vision) value("targetY","160.0").toFloat() else 160f,
                if (vision) value("processMap","321") else "321", if (vision) value("storageMap","321") else "321",
                if (arm) value("slots","0,90,180").split(',').map { it.trim().toInt() } else listOf(0,90,180),
                vision, arm, if (arm && !vision) value("taskCode","156+123+516+231").trim() else "",
                turnRpm = value("turnRpm","40").toInt(), wheelPulses = value("wheelPulses","3200").toInt())
        } catch (_: IllegalArgumentException) { null }
        if (settings == null || settings.frames(1,2,parameters).isEmpty()) error = "请检查速度、脉冲、任务码、视觉或槽位参数"
        else { error = null; onStart(settings, parameters) }
    }) { Text("一键启动全流程") }
    OutlinedButton(onClick = onStop, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("停止全流程") }
}
