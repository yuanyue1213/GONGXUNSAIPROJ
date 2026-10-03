from pathlib import Path

p = Path('yundong_part/application/app/src/main/java/com/example/app/MainActivity.kt')
s = p.read_text(encoding='utf-8')
s = s.replace('import androidx.compose.foundation.background', '''import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.relocation.BringIntoViewRequester
import androidx.compose.foundation.relocation.bringIntoViewRequester
import androidx.compose.foundation.layout.WindowInsets
import androidx.compose.foundation.layout.ime
import androidx.compose.foundation.layout.imePadding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.saveable.rememberSaveableStateHolder
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.platform.LocalSoftwareKeyboardController
import kotlinx.coroutines.delay
import androidx.compose.foundation.background''')
start = s.index('    Column(\n', s.index('private fun RemoteControlScreen('))
end = s.index('\n@Composable\nprivate fun DistanceMovePanel', start)
connection_start = s.index('        OutlinedTextField(', start)
connection_end = s.index('\n        Spacer(Modifier.height(20.dp))', connection_start)
connection = s[connection_start:connection_end]
replacement = '''    var page by rememberSaveable { mutableStateOf(0) }
    val pages = listOf("连接", "底盘", "机械臂", "舵机", "流程")
    val pageState = rememberSaveableStateHolder()
    val focusManager = LocalFocusManager.current
    val keyboard = LocalSoftwareKeyboardController.current

    // 系统边缘与键盘占用空间交给布局处理，输入页只在剩余高度内滚动。
    Scaffold(
        modifier = Modifier.fillMaxSize().safeDrawingPadding().imePadding(),
        contentWindowInsets = WindowInsets(0, 0, 0, 0),
        topBar = {
            Row(Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 4.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.SpaceBetween) {
                Column {
                    Text(pages[page], style = MaterialTheme.typography.titleLarge)
                    Text(connectionText, style = MaterialTheme.typography.bodySmall,
                        color = if (connected) Color(0xFF16803C) else MaterialTheme.colorScheme.error)
                }
                Button(onClick = { onStopArm(); onStopMotion() }, enabled = connected,
                    colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error)) {
                    Text("全部停止")
                }
            }
        },
        bottomBar = {
            NavigationBar {
                pages.forEachIndexed { index, title ->
                    NavigationBarItem(selected = page == index,
                        onClick = { focusManager.clearFocus(); keyboard?.hide(); page = index },
                        icon = { Text(if (page == index) "●" else "○") },
                        label = { Text(title) })
                }
            }
        },
    ) { padding ->
        // 每页各自保存参数和滚动位置，切走后不会重置。
        pageState.SaveableStateProvider(page) {
            Column(
                modifier = Modifier.fillMaxSize().padding(padding)
                    .verticalScroll(rememberScrollState()).padding(horizontal = 16.dp, vertical = 12.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                when (page) {
                    0 -> {
CONNECTION
                    }
                    1 -> DistanceMovePanel(connected, moveRunning, moveStatus, onMove, onAlign, onRingAlign, onParallel, onStopMotion)
                    2 -> ArmPosePanel(connected, onArmPose, onInitializeOrigin, onHome, onStopArm)
                    3 -> {
                        ServoAngleInput("夹子 · PC8", 'G', 270, connected, onServoFinished)
                        ServoAngleInput("转盘 · PA8", 'T', 270, connected, onServoFinished)
                        ServoAngleInput("基座 · PC6", 'B', 360, connected, onServoFinished)
                        GripperMotionPanel(connected, onGripper, onStopGripper)
                    }
                    4 -> {
                        var flow by rememberSaveable { mutableStateOf(0) }
                        val flowState = rememberSaveableStateHolder()
                        TabRow(selectedTabIndex = flow) {
                            listOf("抓取", "放下").forEachIndexed { index, title ->
                                Tab(selected = flow == index, onClick = {
                                    focusManager.clearFocus(); keyboard?.hide(); flow = index
                                }, text = { Text(title) })
                            }
                        }
                        Spacer(Modifier.height(12.dp))
                        flowState.SaveableStateProvider(flow) {
                            if (flow == 0) SequenceSettingsPanel("抓取", false, connected, onGrab)
                            else SequenceSettingsPanel("放下", true, connected, onRelease)
                        }
                        Button(onClick = onCancelGrab, enabled = connected, modifier = Modifier.fillMaxWidth()) {
                            Text("取消自动流程")
                        }
                    }
                }
                Spacer(Modifier.height(24.dp))
            }
        }
    }
}
'''.replace('CONNECTION', connection)
s = s[:start] + replacement + s[end:]
# 底盘标定值与移动/视觉/转向共享状态，各分区仅显示自身控件。
start = s.index('    Text("底盘定距移动"')
s = s[:start] + '''    var section by rememberSaveable { mutableStateOf(0) }
    val focusManager = LocalFocusManager.current
    val keyboard = LocalSoftwareKeyboardController.current
    TabRow(selectedTabIndex = section) {
        listOf("移动", "视觉", "转向", "标定").forEachIndexed { index, title ->
            Tab(selected = section == index, onClick = {
                focusManager.clearFocus(); keyboard?.hide(); section = index
            }, text = { Text(title) })
        }
    }
    Spacer(Modifier.height(12.dp))
    if (section == 0) {
''' + s[start:]
cal_start = s.index('    Text("距离标定 ·')
cal_end = s.index('    error?.let', cal_start)
calibration = s[cal_start:cal_end]
s = s[:cal_start] + s[cal_end:]
visual_start = s.index('    Text("位置修正参数"')
s = s[:visual_start] + '    }\n    if (section == 1) {\n' + s[visual_start:]
turn_start = s.index('    Text("左右转 /')
s = s[:turn_start] + '    }\n    if (section == 2) {\n' + s[turn_start:]
stop_start = s.index('    Button(\n        onClick = onStop,', turn_start)
s = s[:stop_start] + '    }\n    if (section == 3) {\n' + calibration + '    }\n' + s[stop_start:]
# 所有输入统一跟随键盘高度重新请求可见区域，含底部的流程参数。
s = s.replace('OutlinedTextField(', 'VisibleTextField(')
pos = s.index('@Composable\nprivate fun DistanceInput(')
s = s[:pos] + '''@OptIn(ExperimentalFoundationApi::class)
@Composable
private fun VisibleTextField(
    value: String,
    onValueChange: (String) -> Unit,
    modifier: Modifier = Modifier,
    label: @Composable (() -> Unit)? = null,
    singleLine: Boolean = true,
    enabled: Boolean = true,
    keyboardOptions: KeyboardOptions = KeyboardOptions.Default,
) {
    val requester = remember { BringIntoViewRequester() }
    var focused by remember { mutableStateOf(false) }
    val keyboardHeight = WindowInsets.ime.getBottom(LocalDensity.current)
    LaunchedEffect(focused, keyboardHeight) {
        if (focused) {
            // 等待键盘动画和页面重排后，将整个输入框滚入可见区域。
            delay(150)
            requester.bringIntoView()
        }
    }
    OutlinedTextField(value = value, onValueChange = onValueChange,
        modifier = modifier.bringIntoViewRequester(requester).onFocusChanged { focused = it.isFocused },
        label = label, singleLine = singleLine, enabled = enabled, keyboardOptions = keyboardOptions)
}

''' + s[pos:]
p.write_text(s, encoding='utf-8')
