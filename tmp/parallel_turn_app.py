from pathlib import Path
base=Path('yundong_part/application/app/src/main/java/com/example/app')
p=base/'DistanceMove.kt';s=p.read_text(encoding='utf-8').replace('direction !in "FBLR"','direction !in "FBLRCW"');p.write_text(s,encoding='utf-8')
p=base/'RobotTcpClient.kt';s=p.read_text(encoding='utf-8');pos=s.index('    fun sendGripper(');s=s[:pos]+'''    fun sendParallel(ppm: Int, rpm: Int, stepMm: Int, reverse: Boolean) {
        if (ppm !in 1..1000000 || rpm !in 5..60 || stepMm !in 1..10) return
        sendFrame("PARALLEL,${nextSequence()},$ppm,$rpm,$stepMm,${if (reverse) 1 else 0}\\n")
    }

'''+s[pos:];p.write_text(s,encoding='utf-8')
p=base/'MainActivity.kt';s=p.read_text(encoding='utf-8').replace('                    onStopMotion = ::stopMotion,','                    onParallel = { ppm, rpm, step, reverse -> robotClient.sendParallel(ppm, rpm, step, reverse) },\n                    onStopMotion = ::stopMotion,').replace('    onRingAlign: (Int, Int, Int, AlignmentSettings) -> Unit,','    onRingAlign: (Int, Int, Int, AlignmentSettings) -> Unit,\n    onParallel: (Int, Int, Int, Boolean) -> Unit,').replace('onAlign, onRingAlign, onStopMotion)','onAlign, onRingAlign, onParallel, onStopMotion)').replace('            onStopMotion = {},','            onParallel = { _, _, _, _ -> },\n            onStopMotion = {},')
a='    var ring by rememberSaveable { mutableStateOf(2) }';s=s.replace(a,a+'''
    var turnMm by rememberSaveable { mutableStateOf("5") }
    var turnRpm by rememberSaveable { mutableStateOf("10") }
    var parallelStep by rememberSaveable { mutableStateOf("3") }
    var reverseTurn by rememberSaveable { mutableStateOf(false) }''')
a='    Button(\n        onClick = onStop,\n        enabled = connected,';pos=s.index(a,s.index('private fun DistanceMovePanel'))
form='''    Text("左右转 / 圆心连线校准", style = MaterialTheme.typography.titleMedium)
    DistanceInput("转动车轮距离（mm）", turnMm, true) { turnMm = it }
    DistanceInput("转向速度（RPM，5–60）", turnRpm, true) { turnRpm = it }
    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        listOf('C' to "左转", 'W' to "右转").forEach { (dir, label) ->
            Button(onClick = {
                val turn = DistanceMove.fromInputs(dir, turnMm, turnRpm, diameter, wheelPulses, forwardCorrection)
                if (turn == null || turn.speedRpm !in 5..60) error = "请检查转向距离、速度和底盘标定"
                else { error = null; onMove(turn) }
            }, enabled = connected && !running) { Text(label) }
        }
    }
    DistanceInput("自动转向单步上限（mm，1–10）", parallelStep, true) { parallelStep = it }
    Button(onClick = { reverseTurn = !reverseTurn }) {
        Text(if (reverseTurn) "校准转向：反向" else "校准转向：默认")
    }
    Button(onClick = {
        val calibration = DistanceMove.fromInputs('F', "1000", "10", diameter, wheelPulses, forwardCorrection)
        val rpm = turnRpm.toIntOrNull(); val step = parallelStep.toIntOrNull()
        if (calibration == null || rpm == null || rpm !in 5..60 || step == null || step !in 1..10)
            error = "转向速度：5–60 RPM；单步：1–10 mm"
        else { error = null; onParallel(calibration.pulsesPerMetre, rpm, step, reverseTurn) }
    }, enabled = connected && !running, modifier = Modifier.fillMaxWidth()) { Text("执行平行校准") }
''';s=s[:pos]+form+s[pos:];p.write_text(s,encoding='utf-8')
