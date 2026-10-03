from pathlib import Path
base=Path('yundong_part/application/app/src/main/java/com/example/app')
p=base/'RobotTcpClient.kt';s=p.read_text(encoding='utf-8').replace('rpm !in 5..60','rpm !in 5..120');pos=s.index('    fun sendServo(');s=s[:pos]+'''    fun sendGripper(start: Int, end: Int, dps: Int) {
        if (start !in 0..270 || end !in 0..270 || dps !in 6..300) return
        sendFrame("GRIP,${nextSequence()},$start,$end,$dps\\n")
    }
    fun stopGripper() { sendFrame("GRIP_STOP,${nextSequence()}\\n") }

'''+s[pos:];p.write_text(s,encoding='utf-8')
p=base/'SequenceSettings.kt';s=p.read_text(encoding='utf-8').replace('in 5..60','in 5..120');p.write_text(s,encoding='utf-8')
p=base/'MainActivity.kt';s=p.read_text(encoding='utf-8').replace('import androidx.compose.material3.Slider\n','').replace('263','248').replace('speed !in 5..60','speed !in 5..120').replace('RPM，5–60','RPM，5–120').replace('速度：5–60 RPM','速度：5–120 RPM');# Restore alignment labels, whose bound remains 60.
s=s.replace('精调及补偿速度（RPM，5–120）','精调及补偿速度（RPM，5–60）').replace('粗调速度（RPM，5–120）','粗调速度（RPM，5–60）')
s=s.replace('                    onServoChange = ::queueServoAngle,','''                    onGripper = { start, end, dps -> cancelPendingServos(); robotClient.sendGripper(start, end, dps) },
                    onStopGripper = { robotClient.stopGripper() },
                    onServoChange = ::queueServoAngle,''').replace('    onServoChange: (Char, Int) -> Unit,','    onGripper: (Int, Int, Int) -> Unit,\n    onStopGripper: () -> Unit,\n    onServoChange: (Char, Int) -> Unit,').replace('            onServoChange = { _, _ -> },','            onGripper = { _, _, _ -> },\n            onStopGripper = {},\n            onServoChange = { _, _ -> },')
s=s.replace('ServoAngleSlider(', 'ServoAngleInput(').replace('connected, onServoChange, onServoFinished)','connected, onServoFinished)')
s=s.replace("        ServoAngleInput(\"基座 · PC6\", 'B', 360, connected, onServoFinished)","        ServoAngleInput(\"基座 · PC6\", 'B', 360, connected, onServoFinished)\n        GripperMotionPanel(connected, onGripper, onStopGripper)")
a=s.index('@Composable\nprivate fun ServoAngleInput(');b=s.index('@Composable\nprivate fun SequenceSettingsPanel',a)
s=s[:a]+'''@Composable
private fun ServoAngleInput(label: String, channel: Char, maximum: Int,
                            enabled: Boolean, onFinished: (Char, Int) -> Unit) {
    var angle by rememberSaveable(channel) { mutableStateOf(when (channel) { 'G' -> "15"; 'T' -> "0"; else -> "248" }) }
    var error by remember { mutableStateOf<String?>(null) }
    DistanceInput("$label 角度（0–$maximum 度）", angle, true) { angle = it }
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(onClick = {
        val value = angle.toIntOrNull()
        if (value == null || value !in 0..maximum) error = "请输入 0–$maximum 度"
        else { error = null; onFinished(channel, value) }
    }, enabled = enabled, modifier = Modifier.fillMaxWidth()) { Text("设置$label") }
}

@Composable
private fun GripperMotionPanel(connected: Boolean, onExecute: (Int, Int, Int) -> Unit, onStop: () -> Unit) {
    var start by rememberSaveable { mutableStateOf("0") }
    var end by rememberSaveable { mutableStateOf("60") }
    var dps by rememberSaveable { mutableStateOf("60") }
    var error by remember { mutableStateOf<String?>(null) }
    Text("夹子开合", style = MaterialTheme.typography.titleMedium)
    DistanceInput("起始角度（0–270 度）", start, true) { start = it }
    DistanceInput("结束角度（0–270 度）", end, true) { end = it }
    DistanceInput("速度（6–300 度/秒）", dps, true) { dps = it }
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(onClick = {
        val from = start.toIntOrNull(); val to = end.toIntOrNull(); val speed = dps.toIntOrNull()
        if (from == null || from !in 0..270 || to == null || to !in 0..270 || speed == null || speed !in 6..300)
            error = "角度：0–270 度；速度：6–300 度/秒"
        else { error = null; onExecute(from, to, speed) }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("执行夹子开合") }
    Button(onClick = onStop, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("停止夹子") }
}

'''+s[b:];p.write_text(s,encoding='utf-8')
