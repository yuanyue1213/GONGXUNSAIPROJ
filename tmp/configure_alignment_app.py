from pathlib import Path
base=Path('yundong_part/application/app/src/main/java/com/example/app')
(base/'AlignmentSettings.kt').write_text('''package com.example.app

import java.math.BigDecimal

internal data class AlignmentSettings(
    val x: Int = 256, val y: Int = 160, val scale: Int = 1090819,
    val fineGain: Int = 30, val coarseGain: Int = 50,
    val fineRpm: Int = 10, val coarseRpm: Int = 20,
    val maxMm: Int = 15, val offsetMm: Int = 10,
) {
    fun valid() = x in 0..511 && y in 0..319 && scale in 10000..10000000 &&
        fineGain in 1..100 && coarseGain in 1..100 && fineRpm in 5..60 && coarseRpm in 5..60 &&
        maxMm in 1..100 && offsetMm in -100..100
    fun frame(sequence: Long, ring: Int, forward: Int, lateral: Int) =
        "ALIGN_CFG,$sequence,$ring,$forward,$lateral,$x,$y,$scale,$fineGain,$coarseGain,$fineRpm,$coarseRpm,$maxMm,$offsetMm\\n"
    companion object {
        fun parse(x: String, y: String, scale: String, fineGain: String, coarseGain: String,
                  fineRpm: String, coarseRpm: String, maxMm: String, offsetMm: String): AlignmentSettings? = try {
            AlignmentSettings(x.trim().toInt(), y.trim().toInt(),
                BigDecimal(scale.trim()).movePointRight(6).intValueExact(), fineGain.trim().toInt(),
                coarseGain.trim().toInt(), fineRpm.trim().toInt(), coarseRpm.trim().toInt(),
                maxMm.trim().toInt(), offsetMm.trim().toInt()).takeIf { it.valid() }
        } catch (_: IllegalArgumentException) { null }
          catch (_: ArithmeticException) { null }
    }
}
''',encoding='utf-8')
p=base/'RobotTcpClient.kt';s=p.read_text(encoding='utf-8');mark='    fun sendRingAlignment(';pos=s.index(mark);s=s[:pos]+'''    fun sendConfiguredAlignment(ring: Int, forwardPpm: Int, lateralPpm: Int, settings: AlignmentSettings) {
        if (ring !in 0..3 || forwardPpm !in 1..1000000 || lateralPpm !in 1..1000000 || !settings.valid()) return
        sendFrame(settings.frame(nextSequence(), ring, forwardPpm, lateralPpm))
    }

'''+s[pos:];p.write_text(s,encoding='utf-8')
p=base/'MainActivity.kt';s=p.read_text(encoding='utf-8').replace('onAlign = { forward, lateral ->','onAlign = { forward, lateral, settings ->').replace('robotClient.sendAlignment(forward, lateral)','robotClient.sendConfiguredAlignment(0, forward, lateral, settings)').replace('onRingAlign = { ring, forward, lateral ->','onRingAlign = { ring, forward, lateral, settings ->').replace('robotClient.sendRingAlignment(ring, forward, lateral)','robotClient.sendConfiguredAlignment(ring, forward, lateral, settings)').replace('onAlign: (Int, Int) -> Unit','onAlign: (Int, Int, AlignmentSettings) -> Unit').replace('onRingAlign: (Int, Int, Int) -> Unit','onRingAlign: (Int, Int, Int, AlignmentSettings) -> Unit').replace('onAlign = { _, _ -> }','onAlign = { _, _, _ -> }').replace('onRingAlign = { _, _, _ -> }','onRingAlign = { _, _, _, _ -> }')
a='    var ring by rememberSaveable { mutableStateOf(2) }';s=s.replace(a,a+'''
    var targetX by rememberSaveable { mutableStateOf("256") }
    var targetY by rememberSaveable { mutableStateOf("160") }
    var pixelScale by rememberSaveable { mutableStateOf("1.090819") }
    var fineGain by rememberSaveable { mutableStateOf("30") }
    var coarseGain by rememberSaveable { mutableStateOf("50") }
    var fineRpm by rememberSaveable { mutableStateOf("10") }
    var coarseRpm by rememberSaveable { mutableStateOf("20") }
    var maxStep by rememberSaveable { mutableStateOf("15") }
    var xOffset by rememberSaveable { mutableStateOf("10") }
    fun alignmentSettings() = AlignmentSettings.parse(targetX, targetY, pixelScale, fineGain,
        coarseGain, fineRpm, coarseRpm, maxStep, xOffset)''')
a='    ) { Text(if (running) "移动中…" else "执行定距移动") }';s=s.replace(a,a+'''
    Text("位置修正参数", style = MaterialTheme.typography.titleMedium)
    DistanceInput("目标中心 x（px，0–511）", targetX, !running) { targetX = it }
    DistanceInput("目标中心 y（px，0–319）", targetY, !running) { targetY = it }
    DistanceInput("比例（mm/px，0.01–10）", pixelScale, !running, true) { pixelScale = it }
    DistanceInput("精调比例（%，1–100）", fineGain, !running) { fineGain = it }
    DistanceInput("粗调比例（%，1–100）", coarseGain, !running) { coarseGain = it }
    DistanceInput("精调及补偿速度（RPM，5–60）", fineRpm, !running) { fineRpm = it }
    DistanceInput("粗调速度（RPM，5–60）", coarseRpm, !running) { coarseRpm = it }
    DistanceInput("单步上限（mm，1–100）", maxStep, !running) { maxStep = it }
    OutlinedTextField(value = xOffset, onValueChange = { xOffset = it },
        label = { Text("结束 x 补偿（mm，-100–100，0关闭）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())''')
s=s.replace('            if (forward == null || lateral == null) error = "请检查轮径、细分和距离修正系数"\n            else { error = null; onAlign(forward.pulsesPerMetre, lateral.pulsesPerMetre) }','''            val settings = alignmentSettings()
            if (forward == null || lateral == null || settings == null) error = "请检查位置修正参数及底盘标定"
            else { error = null; onAlign(forward.pulsesPerMetre, lateral.pulsesPerMetre, settings) }''')
s=s.replace('        if (forward == null || lateral == null) error = "请检查轮径、细分和距离修正系数"\n        else { error = null; onRingAlign(ring, forward.pulsesPerMetre, lateral.pulsesPerMetre) }','''        val settings = alignmentSettings()
        if (forward == null || lateral == null || settings == null) error = "请检查位置修正参数及底盘标定"
        else { error = null; onRingAlign(ring, forward.pulsesPerMetre, lateral.pulsesPerMetre, settings) }''')
# Put validation errors after the settings so they are visible near execution.
s=s.replace('    ) { Text("执行位置修正") }','    ) { Text("执行位置修正") }\n    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }');p.write_text(s,encoding='utf-8')
