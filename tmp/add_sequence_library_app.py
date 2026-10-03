from pathlib import Path

p = Path('yundong_part/application/app/src/main/java/com/example/app/MainActivity.kt')
s = p.read_text(encoding='utf-8')
s = s.replace('import androidx.compose.material3.Button\n', 'import androidx.compose.material3.Button\nimport androidx.compose.material3.Card\nimport androidx.compose.ui.platform.LocalContext\nimport java.util.UUID\n')
s = s.replace("onInitializeOrigin = { cancelPendingServos(); robotClient.sendMotion('I') },", 'onInitializeOrigin = { base -> cancelPendingServos(); robotClient.initializeOrigin(base) },')
s = s.replace('                    onRelease = { settings -> cancelPendingServos(); robotClient.sendSequence(\'P\', settings) },', '''                    onRelease = { settings -> cancelPendingServos(); robotClient.sendSequence('P', settings) },
                    onPlan = { cards -> cancelPendingServos(); robotClient.sendPlan(cards) },''')
s = s.replace('    onInitializeOrigin: () -> Unit,', '    onInitializeOrigin: (Int) -> Unit,')
s = s.replace('    onRelease: (SequenceSettings) -> Unit,', '    onRelease: (SequenceSettings) -> Unit,\n    onPlan: (List<SequenceCard>) -> Unit = {},')
s = s.replace('    val pageState = rememberSaveableStateHolder()', '''    val pageState = rememberSaveableStateHolder()
    val context = LocalContext.current
    val cardStore = remember(context) { SequenceCardStore(context) }
    var cards by remember { mutableStateOf(cardStore.cards()) }
    var plan by remember { mutableStateOf(cardStore.plan()) }
    fun saveCard(name: String, mode: Char, settings: SequenceSettings) {
        cards = cards + SequenceCard(UUID.randomUUID().toString(), name.trim(), mode, settings)
        cardStore.saveCards(cards)
    }''', 1)
s = s.replace('listOf("抓取", "放下").forEachIndexed', 'listOf("抓取", "放下", "卡片编排").forEachIndexed')
s = s.replace('''                            if (flow == 0) SequenceSettingsPanel("抓取", false, connected, onGrab)
                            else SequenceSettingsPanel("放下", true, connected, onRelease)''', '''                            when (flow) {
                                0 -> SequenceSettingsPanel("抓取", false, connected, onGrab,
                                    onSave = { name, settings -> saveCard(name, 'A', settings) })
                                1 -> SequenceSettingsPanel("放下", true, connected, onRelease,
                                    onSave = { name, settings -> saveCard(name, 'P', settings) })
                                else -> SequenceCardsPanel(cards, plan, connected,
                                    onPlanChange = { plan = it; cardStore.savePlan(it) },
                                    onDeleteCard = { id -> cards = cards.filterNot { it.id == id }; cardStore.saveCards(cards) },
                                    onExecute = { onPlan(plan) })
                            }''')
s = s.replace('    title: String, release: Boolean, connected: Boolean, onExecute: (SequenceSettings) -> Unit,\n)', '    title: String, release: Boolean, connected: Boolean, onExecute: (SequenceSettings) -> Unit,\n    onSave: (String, SequenceSettings) -> Unit,\n)')
a = s.index('private fun SequenceSettingsPanel(')
b = s.index('@Composable\nprivate fun ArmPosePanel', a)
part = s[a:b]
part = part.replace('    var error by remember', '    var cardName by rememberSaveable { mutableStateOf(title) }\n    var saved by remember { mutableStateOf(false) }\n    var error by remember', 1)
part = part.replace('    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }', '''    VisibleTextField(value = cardName, onValueChange = { cardName = it; saved = false },
        label = { Text("卡片名称（最多40字）") }, modifier = Modifier.fillMaxWidth())
    Button(onClick = {
        val settings = SequenceSettings.parse(r1, z1, r2, z2, radial, up, down, gripper, theta, openAngle, closeAngle, baseHome, baseTilt)
        if (settings == null) error = "请检查位置、速度和角度参数"
        else if (cardName.isBlank() || cardName.trim().length > 40) error = "请填写1–40字的卡片名称"
        else { error = null; onSave(cardName.trim(), settings); saved = true }
    }, modifier = Modifier.fillMaxWidth()) { Text("保存为卡片") }
    if (saved) Text("已保存，可在卡片编排中选择")
    error?.let { Text(it, color = MaterialTheme.colorScheme.error) }''')
s = s[:a] + part + s[b:]
s = s.replace('    onInitialize: () -> Unit,', '    onInitialize: (Int) -> Unit,')
s = s.replace('''    Button(onClick = { onInitialize(); note = "已请求原点初始化：转盘0°、爪子15°、基座248°，记录两轴当前位置" },
        enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("启动原点状态") }''', '''    var originBase by rememberSaveable { mutableStateOf("248") }
    DistanceInput("初始化基座角度（0–360 度）", originBase, true) { originBase = it }
    Button(onClick = {
        val base = originBase.toIntOrNull()
        if (base == null || base !in 0..360) note = "请输入0–360度的初始化基座角度"
        else { onInitialize(base); note = "已请求原点初始化：基座$base°，记录两轴当前位置" }
    }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("启动原点状态") }''')
index = s.index('@Composable\nprivate fun ArmPosePanel')
s = s[:index] + '''/** Plan entries are snapshots, so deleting a library card does not alter a saved plan. */
@Composable
private fun SequenceCardsPanel(
    cards: List<SequenceCard>, plan: List<SequenceCard>, connected: Boolean,
    onPlanChange: (List<SequenceCard>) -> Unit, onDeleteCard: (String) -> Unit, onExecute: () -> Unit,
) {
    Text("当前编排 ${plan.size}/${SequencePlan.MAX_ITEMS}", style = MaterialTheme.typography.titleMedium)
    if (plan.isEmpty()) Text("从下方卡片库加入抓取或放下卡片")
    plan.forEachIndexed { index, card ->
        SequenceCardView(card, "${index + 1}. ") {
            Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                Button(onClick = {
                    val next = plan.toMutableList()
                    next[index] = next[index - 1]; next[index - 1] = card; onPlanChange(next)
                }, enabled = index > 0) { Text("上移") }
                Button(onClick = {
                    val next = plan.toMutableList()
                    next[index] = next[index + 1]; next[index + 1] = card; onPlanChange(next)
                }, enabled = index < plan.lastIndex) { Text("下移") }
                Button(onClick = { onPlanChange(plan.filterIndexed { i, _ -> i != index }) }) { Text("移除") }
            }
        }
    }
    Button(onClick = onExecute, enabled = connected && plan.isNotEmpty(), modifier = Modifier.fillMaxWidth()) {
        Text("执行编排")
    }
    Button(onClick = { onPlanChange(emptyList()) }, enabled = plan.isNotEmpty(), modifier = Modifier.fillMaxWidth()) {
        Text("清空编排")
    }
    HorizontalDivider(Modifier.padding(vertical = 12.dp))
    Text("卡片库", style = MaterialTheme.typography.titleMedium)
    if (cards.isEmpty()) Text("在抓取或放下页填写参数，点击保存为卡片")
    cards.forEach { card ->
        SequenceCardView(card) {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { onPlanChange(plan + card) }, enabled = plan.size < SequencePlan.MAX_ITEMS) { Text("加入流程") }
                Button(onClick = { onDeleteCard(card.id) }) { Text("删除卡片") }
            }
        }
    }
}

@Composable
private fun SequenceCardView(card: SequenceCard, prefix: String = "", actions: @Composable () -> Unit) {
    val p = card.settings
    Card(Modifier.fillMaxWidth().padding(vertical = 6.dp)) {
        Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("$prefix${card.name} · ${if (card.mode == 'A') "抓取" else "放下"}", fontWeight = FontWeight.Bold)
            Text("目标1 r=${p.r1 / 10.0} mm  z=${p.z1} mm；目标2 r=${p.r2 / 10.0} mm  z=${p.z2} mm")
            Text("伸缩/上升/下降 ${p.radialRpm}/${p.upRpm}/${p.downRpm} RPM；夹子 ${p.gripperDps}°/s")
            Text("转盘 ${p.theta}°；夹子开/合 ${p.openAngle}/${p.closeAngle}°；基座 ${p.baseHome}/${p.baseTilt}°")
            actions()
        }
    }
}

''' + s[index:]
p.write_text(s, encoding='utf-8')
