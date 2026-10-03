from pathlib import Path
root=Path('F:/GONGXUNSAIPROJ')
def edit(path, old, new):
 p=root/path;s=p.read_text(encoding='utf-8');assert old in s,path;p.write_text(s.replace(old,new),encoding='utf-8')
for path in ['yundong_part/stm32h743/zhukong/Core/Src/robot_control.c','yundong_part/esp32s3/hello_world/main/robot_remote_main.c']:
 edit(path,'strncmp(frame, "ALIGN,", 6U) == 0','(strncmp(frame, "ALIGN,", 6U) == 0 || strncmp(frame, "ALIGN_RING,", 11U) == 0)')
edit('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c','s_align_moves = s_align_centered = 0U; CameraLink_Discard(); return;','s_align_moves = s_align_centered = 0U; CameraLink_SelectTarget(cmd.ring_index); return;')
p='yundong_part/application/app/src/main/java/com/example/app/RobotTcpClient.kt'
edit(p,'    fun sendArmDistance(','''    fun sendRingAlignment(ring: Int, forwardPpm: Int, lateralPpm: Int) {
        if (ring !in 1..3 || forwardPpm !in 1..1000000 || lateralPpm !in 1..1000000) return
        sendFrame("ALIGN_RING,${nextSequence()},$ring,$forwardPpm,$lateralPpm\\n")
    }

    fun sendArmDistance(''')
p='yundong_part/application/app/src/main/java/com/example/app/MainActivity.kt'
edit(p,'    onAlign: (Int, Int) -> Unit,','    onAlign: (Int, Int) -> Unit,\n    onRingAlign: (Int, Int, Int) -> Unit,')
edit(p,'                    onStopMotion = ::stopMotion,','''                    onRingAlign = { ring, forward, lateral ->
                        robotClient.sendRingAlignment(ring, forward, lateral)
                        moveStatus = "已提交色环位置修正；停止底盘可取消，无到位回包"
                    },
                    onStopMotion = ::stopMotion,''')
edit(p,'onMove, onAlign, onStopMotion)','onMove, onAlign, onRingAlign, onStopMotion)')
edit(p,'            onAlign = { _, _ -> },','            onAlign = { _, _ -> },\n            onRingAlign = { _, _, _ -> },')
edit(p,'    Text("底盘定距移动",','    var ring by rememberSaveable { mutableStateOf(2) }\n    Text("底盘定距移动",')
edit(p,'    ) { Text("执行位置修正") }','''    ) { Text("执行位置修正") }
    Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
        listOf(1 to "左色环", 2 to "中色环", 3 to "右色环").forEach { (value, label) ->
            Button(onClick = { ring = value }, enabled = !running) {
                Text(if (ring == value) "✓ $label" else label)
            }
        }
    }
    Button(onClick = {
        val forward = DistanceMove.fromInputs('F', "1000", "20", diameter, wheelPulses, forwardCorrection)
        val lateral = DistanceMove.fromInputs('L', "1000", "20", diameter, wheelPulses, lateralCorrection)
        if (forward == null || lateral == null) error = "请检查轮径、细分和距离修正系数"
        else { error = null; onRingAlign(ring, forward.pulsesPerMetre, lateral.pulsesPerMetre) }
    }, enabled = connected && !running, modifier = Modifier.fillMaxWidth()) {
        Text("执行色环位置修正")
    }
    Text("色环按图像从左到右编号，默认中间色环；摄像头须运行 circle.py。")''')
