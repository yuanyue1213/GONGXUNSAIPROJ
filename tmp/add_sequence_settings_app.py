from pathlib import Path
root=Path('F:/GONGXUNSAIPROJ')
base=root/'yundong_part/application/app/src/main/java/com/example/app'
(base/'SequenceSettings.kt').write_text('''package com.example.app

import java.math.BigDecimal

internal data class SequenceSettings(
    val r1: Int, val z1: Int, val r2: Int, val z2: Int,
    val radialRpm: Int, val upRpm: Int, val downRpm: Int,
) {
    fun valid() = r1 in -10000..10000 && r2 in -10000..10000 &&
        z1 in -400..400 && z2 in -400..400 &&
        radialRpm in 5..60 && upRpm in 5..60 && downRpm in 5..60
    fun frame(sequence: Long, mode: Char) =
        "STATE,$sequence,$mode,$r1,$z1,$r2,$z2,$radialRpm,$upRpm,$downRpm\\n"
    companion object {
        fun parse(r1: String, z1: String, r2: String, z2: String,
                  radial: String, up: String, down: String): SequenceSettings? = try {
            SequenceSettings(BigDecimal(r1.trim()).movePointRight(1).intValueExact(),
                z1.trim().toInt(), BigDecimal(r2.trim()).movePointRight(1).intValueExact(),
                z2.trim().toInt(), radial.trim().toInt(), up.trim().toInt(), down.trim().toInt())
                .takeIf { it.valid() }
        } catch (_: IllegalArgumentException) { null }
          catch (_: ArithmeticException) { null }
    }
}
''',encoding='utf-8')
p=base/'RobotTcpClient.kt';s=p.read_text(encoding='utf-8');s=s.replace('    fun sendMotion(direction: Char) {','''    fun sendSequence(mode: Char, settings: SequenceSettings) {
        if (mode !in "AP" || !settings.valid()) return
        sendFrame(settings.frame(nextSequence(), mode))
    }

    fun sendMotion(direction: Char) {''');p.write_text(s,encoding='utf-8')
p=base/'MainActivity.kt';s=p.read_text(encoding='utf-8');s=s.replace('import androidx.compose.foundation.text.selection.SelectionContainer\n','').replace('    private var communicationTrace by mutableStateOf("")\n','')
a=s.index('            onMessage = { message ->');b=s.index('\n        )',a);s=s[:a]+'            onMessage = {},'+s[b:]
s=s.replace('                    communicationTrace = communicationTrace,\n','').replace('                        communicationTrace = ""\n','').replace('    communicationTrace: String,\n','').replace('            communicationTrace = "",\n','')
s=s.replace("onGrab = { cancelPendingServos(); robotClient.sendMotion('A') }", "onGrab = { settings -> cancelPendingServos(); robotClient.sendSequence('A', settings) }").replace("onRelease = { cancelPendingServos(); robotClient.sendMotion('P') }", "onRelease = { settings -> cancelPendingServos(); robotClient.sendSequence('P', settings) }")
s=s.replace('    onGrab: () -> Unit,','    onGrab: (SequenceSettings) -> Unit,').replace('    onRelease: () -> Unit,','    onRelease: (SequenceSettings) -> Unit,')
a=s.index('        Text("最近发送记录');b=s.index('        Spacer(Modifier.height(16.dp))',a);s=s[:a]+s[b:]
a=s.index('        Text("固定状态：抓取');b=s.index('        Text("舵机角度',a)
s=s[:a]+'''        SequenceSettingsPanel("抓取", false, connected, onGrab)
        SequenceSettingsPanel("放下", true, connected, onRelease)
        Button(onClick = onCancelGrab, enabled = connected, modifier = Modifier.fillMaxWidth()) {
            Text("取消自动流程")
        }
'''+s[b:]
remove_prefixes=['        Text("舵机角度（','    Text("选择方向和距离','    Text("修正系数默认','    Text("色环按图像','    Text("绿色物块对准','    Text("先点击启动原点','    Text("原点初始化不移动']
lines=[]
for line in s.splitlines():
 if line.startswith(remove_prefixes[0]): lines.append('        Text("舵机角度", style = MaterialTheme.typography.titleMedium)')
 elif any(line.startswith(x) for x in remove_prefixes[1:]): continue
 else: lines.append(line)
s='\n'.join(lines)+'\n'
s=s.replace('两轴每圈脉冲（须匹配细分，默认 3200）','两轴每圈脉冲').replace('车轮每圈脉冲（默认 1.8° / 16 细分为 3200）','车轮每圈脉冲')
pos=s.index('@Composable\nprivate fun ArmPosePanel')
form='''@Composable
private fun SequenceSettingsPanel(
    title: String, release: Boolean, connected: Boolean, onExecute: (SequenceSettings) -> Unit,
) {
    var r1 by rememberSaveable { mutableStateOf(if (release) "20" else "110") }
    var z1 by rememberSaveable { mutableStateOf(if (release) "-40" else "-50") }
    var r2 by rememberSaveable { mutableStateOf(if (release) "130" else "20") }
    var z2 by rememberSaveable { mutableStateOf(if (release) "-110" else "-40") }
    var radial by rememberSaveable { mutableStateOf("20") }
    var up by rememberSaveable { mutableStateOf("20") }
    var down by rememberSaveable { mutableStateOf("50") }
    var error by remember { mutableStateOf<String?>(null) }
    Text("$title参数", style = MaterialTheme.typography.titleMedium)
    val first = if (release) "第3步取物" else "第2步抓取"
    val second = "第6步放置"
    OutlinedTextField(value = r1, onValueChange = { r1 = it }, label = { Text("$first r（mm）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())
    OutlinedTextField(value = z1, onValueChange = { z1 = it }, label = { Text("$first z（mm）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())
    OutlinedTextField(value = r2, onValueChange = { r2 = it }, label = { Text("$second r（mm）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())
    OutlinedTextField(value = z2, onValueChange = { z2 = it }, label = { Text("$second z（mm）") },
        singleLine = true, modifier = Modifier.fillMaxWidth())
    DistanceInput("伸缩速度（RPM）", radial, true) { radial = it }
    DistanceInput("上升速度（RPM）", up, true) { up = it }
    DistanceInput("下降速度（RPM）", down, true) { down = it }
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    Button(onClick = {
        val settings = SequenceSettings.parse(r1, z1, r2, z2, radial, up, down)
        if (settings == null) error = "r：±1000 mm（最多1位小数）；z：±400 mm（整数）；速度：5–60 RPM"
        else { error = null; onExecute(settings) }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("执行$title") }
    Spacer(Modifier.height(12.dp))
}

'''
s=s[:pos]+form+s[pos:];p.write_text(s,encoding='utf-8')
