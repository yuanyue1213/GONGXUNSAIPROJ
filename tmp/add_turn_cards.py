from pathlib import Path
def edit(path, fn):
 p=Path(path); s=p.read_text(encoding='utf-8'); p.write_text(fn(s),encoding='utf-8',newline='\r\n')
def protocol(s):
 s=s.replace("(*p != 'A' && *p != 'P')", "(*p != 'A' && *p != 'P' && *p != 'T')")
 anchor='    cmd->mode = *p; p += 2;\n'
 s=s.replace(anchor,anchor+'''    /* T card: start/end angles and angular speed, independent of arm coordinates. */
    if (cmd->mode == 'T') {
        return RobotProtocol_U32(&p, &cmd->theta, ',') &&
            RobotProtocol_U32(&p, &cmd->open_angle, ',') &&
            RobotProtocol_U32(&p, &cmd->gripper_dps, '\\0') && cmd->sequence != 0U &&
            cmd->theta <= 270U && cmd->open_angle <= 270U &&
            cmd->gripper_dps >= 1U && cmd->gripper_dps <= 360U;
    }
''')
 return s
edit('yundong_part/shared/robot_distance_protocol.h',protocol)
def control(s):
 s=s.replace('static bool s_release_mode;', '''static bool s_release_mode;
static bool s_turn_card_active, s_release_origin_pending;
static uint32_t s_turn_card_tick, s_turn_card_update, s_turn_card_ms;
static int32_t s_turn_card_start, s_turn_card_end;''')
 s=s.replace('static bool RobotControl_FinishSequencePosition(void)\n{', '''static bool RobotControl_FinishSequencePosition(void)
{
    /* Release starts by returning r/z to their recorded origins, before changing posture. */
    if (s_release_origin_pending) {
        s_release_origin_pending = false;
        if (!RobotControl_SetServoAngle('G', s_sequence_steps[0].gripper) ||
            !RobotControl_SetServoAngle('B', s_sequence_steps[0].base)) {
            RobotControl_CancelGrab(); return false;
        }
    }''')
 # forward declaration must precede new use
 s=s.replace('static bool RobotControl_FinishSequencePosition(void)', 'static void RobotControl_CancelGrab(void);\nstatic bool RobotControl_FinishSequencePosition(void)',1)
 s=s.replace('    s_grab_active = false;\n    if (s_grab_arm_wait', '    s_grab_active = false; s_turn_card_active = false; s_release_origin_pending = false;\n    if (s_grab_arm_wait')
 s=s.replace("if (!RobotControl_SetServoAngle('T', s_sequence_theta) || !RobotControl_SetServoAngle('G', gripper) ||", "if ((!s_release_mode && !RobotControl_SetServoAngle('T', s_sequence_theta)) || !RobotControl_SetServoAngle('G', gripper) ||")
 anchor='static void RobotControl_TickGrab(void)\n{'
 helper='''static void RobotControl_CompleteSequence(void)
{
    if (s_plan_active && ++s_plan_index < s_plan_count) {
        s_grab_step = 8U; s_grab_tick = HAL_GetTick();
    } else {
        s_grab_active = false;
        s_plan_active = false; s_plan_count = 0U; s_plan_mask = 0U;
    }
}
'''
 s=s.replace(anchor,helper+anchor)
 s=s.replace('    if (!s_grab_active) return;\n    if (s_grab_step', '''    if (!s_grab_active) return;
    if (s_turn_card_active) {
        uint32_t now = HAL_GetTick(), elapsed = now - s_turn_card_tick;
        if (elapsed < s_turn_card_ms && now - s_turn_card_update < 20U) return;
        uint32_t progress = elapsed < s_turn_card_ms ? elapsed : s_turn_card_ms;
        uint16_t angle = s_turn_card_ms == 0U ? (uint16_t)s_turn_card_end :
            (uint16_t)(s_turn_card_start + (s_turn_card_end - s_turn_card_start) *
                (int32_t)progress / (int32_t)s_turn_card_ms);
        if (!RobotControl_SetServoAngle('T', angle)) { RobotControl_CancelGrab(); return; }
        s_turn_card_update = now;
        if (elapsed >= s_turn_card_ms) { s_turn_card_active = false; RobotControl_CompleteSequence(); }
        return;
    }
    if (s_grab_step''')
 a=s.index('        if (s_plan_active && ++s_plan_index < s_plan_count)',s.index('static void RobotControl_TickGrab'))
 b=s.index('\n    } else if',a)
 s=s[:a]+'        RobotControl_CompleteSequence();'+s[b:]
 s=s.replace('    s_grab_active = false;\n    s_manual_gripper_active', '    s_grab_active = false; s_turn_card_active = false; s_release_origin_pending = false;\n    s_manual_gripper_active')
 anchor='    if (!RobotControl_SequenceIdle()) return false;\n'
 s=s.replace(anchor,anchor+'''    if (direction == 'T' && settings != NULL) {
        if (!RobotControl_SetServoAngle('T', (uint16_t)settings->theta)) return false;
        s_turn_card_start = (int32_t)settings->theta; s_turn_card_end = (int32_t)settings->open_angle;
        uint32_t distance = (uint32_t)(s_turn_card_end > s_turn_card_start ?
            s_turn_card_end - s_turn_card_start : s_turn_card_start - s_turn_card_end);
        s_turn_card_ms = (distance * 1000U + settings->gripper_dps - 1U) / settings->gripper_dps;
        s_turn_card_tick = s_turn_card_update = HAL_GetTick();
        s_turn_card_active = s_grab_active = true; return true;
    }
''')
 s=s.replace('    s_grab_active = RobotControl_ApplyGrabStep();', '''    s_release_origin_pending = s_release_mode;
    s_grab_active = s_release_mode ? RobotControl_StartSequencePosition() : RobotControl_ApplyGrabStep();''')
 s=s.replace("if (direction == 'A' || direction == 'P') {", "if (direction == 'A' || direction == 'P' || (configured && direction == 'T')) {")
 return s
edit('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c',control)
def cards(s):
 s=s.replace('/** A saved card', '''internal data class TurnSettings(val start: Int, val end: Int, val dps: Int) {
    fun valid() = start in 0..270 && end in 0..270 && dps in 1..360
}

/** A saved card''')
 s=s.replace('    val settings: SequenceSettings,', '    val settings: SequenceSettings = SequenceSettings(0,0,0,0,20,20,50),\n    val turn: TurnSettings? = null,')
 s=s.replace('mode in "AP" && settings.valid()', '(if (mode == \'T\') turn?.valid() == true else mode in "AP" && settings.valid())\n    fun frame(sequence: Long) = if (mode == \'T\') "STATE,$sequence,T,${turn!!.start},${turn.end},${turn.dps}\\n" else settings.frame(sequence, mode)')
 s=s.replace('it.settings.frame(1, it.mode)', 'it.frame(1)')
 anchor="            val fields = parts[2].split(',')\n"
 s=s.replace(anchor,anchor+'''            if (fields.size == 4 && fields[0] == "T") {
                return@mapNotNull SequenceCard(URLDecoder.decode(parts[0], "UTF-8"),
                    URLDecoder.decode(parts[1], "UTF-8"), 'T',
                    turn = TurnSettings(fields[1].toInt(), fields[2].toInt(), fields[3].toInt())).takeIf { it.valid() }
            }
''')
 s=s.replace('card.settings.frame(sequence, card.mode)', 'card.frame(sequence)')
 return s
edit('yundong_part/application/app/src/main/java/com/example/app/SequenceCards.kt',cards)
def ui(s):
 s=s.replace('listOf("抓取", "放下", "卡片编排")','listOf("抓取", "放下", "转盘", "卡片编排")')
 s=s.replace('                                else -> SequenceCardsPanel', '''                                2 -> TurnCardPanel(connected, onSave = { name, turn ->
                                    cards = cards + SequenceCard(UUID.randomUUID().toString(), name, 'T', turn = turn)
                                    cardStore.saveCards(cards)
                                }, onExecute = { turn -> onPlan(listOf(SequenceCard("manual-turn", "转盘", 'T', turn = turn))) })
                                else -> SequenceCardsPanel''')
 s=s.replace('    DistanceInput("转盘角度（0–270 度）", theta, true) { theta = it }', '    if (!release) DistanceInput("转盘角度（0–270 度）", theta, true) { theta = it }')
 s=s.replace('SequenceSettings.parse(r1, z1, r2, z2, radial, up, down, gripper, theta,', 'SequenceSettings.parse(r1, z1, r2, z2, radial, up, down, gripper, if (release) "0" else theta,')
 s=s.replace('从下方卡片库加入抓取或放下卡片','从下方卡片库加入抓取、放下或转盘卡片').replace('在抓取或放下页填写参数','在抓取、放下或转盘页填写参数')
 s=s.replace('if (card.mode == \'A\') "抓取" else "放下"', 'when (card.mode) { \'A\' -> "抓取"; \'T\' -> "转盘"; else -> "放下" }')
 a=s.index('            Text("目标1',s.index('private fun SequenceCardView')); b=s.index('            actions()',a)
 old=s[a:b].replace('            Text("转盘 ${p.theta}°；', '            Text("${if (card.mode == \'P\') "" else "转盘 ${p.theta}°；"}')
 s=s[:a]+'''            if (card.mode == 'T') {
                val t = card.turn!!
                Text("转盘 ${t.start}° → ${t.end}°；速度 ${t.dps}°/秒")
            } else {
'''+old+'''            }
'''+s[b:]
 s += '''
@Composable
private fun TurnCardPanel(connected: Boolean, onSave: (String, TurnSettings) -> Unit, onExecute: (TurnSettings) -> Unit) {
    var start by rememberSaveable { mutableStateOf("0") }
    var end by rememberSaveable { mutableStateOf("120") }
    var speed by rememberSaveable { mutableStateOf("60") }
    var name by rememberSaveable { mutableStateOf("转盘") }
    var message by remember { mutableStateOf<String?>(null) }
    fun values(): TurnSettings? = try { TurnSettings(start.trim().toInt(), end.trim().toInt(), speed.trim().toInt()).takeIf { it.valid() } } catch (_: IllegalArgumentException) { null }
    DistanceInput("起始角度（0–270度）", start, true) { start = it }
    DistanceInput("结束角度（0–270度）", end, true) { end = it }
    DistanceInput("速度（1–360度/秒）", speed, true) { speed = it }
    VisibleTextField(value = name, onValueChange = { name = it }, label = { Text("卡片名称") }, modifier = Modifier.fillMaxWidth())
    Button(onClick = {
        val v = values()
        if (v == null || name.trim().length !in 1..40) message = "请检查角度、速度和名称"
        else { onSave(name.trim(), v); message = "已保存" }
    }, modifier = Modifier.fillMaxWidth()) { Text("保存转盘卡片") }
    Button(onClick = { val v = values(); if (v == null) message = "请检查角度和速度" else onExecute(v) }, enabled = connected, modifier = Modifier.fillMaxWidth()) { Text("执行转盘") }
    message?.let { Text(it) }
}
'''
 return s
edit('yundong_part/application/app/src/main/java/com/example/app/MainActivity.kt',ui)
