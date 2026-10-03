from pathlib import Path
p=Path('yundong_part/shared/robot_distance_protocol.h');s=p.read_text(encoding='utf-8').replace('down_rpm, gripper_dps;','down_rpm, gripper_dps, theta, open_angle, close_angle, base_home, base_tilt;');a="    if (!RobotProtocol_U32(&p, &cmd->down_rpm, has_gripper_speed ? ',' : '\\0') ||\n        (has_gripper_speed && !RobotProtocol_U32(&p, &cmd->gripper_dps, '\\0'))) return false;";b="""    cmd->theta = 0U; cmd->open_angle = 60U; cmd->close_angle = 0U;
    cmd->base_home = 248U; cmd->base_tilt = 140U;
    if (!RobotProtocol_U32(&p, &cmd->down_rpm, has_gripper_speed ? ',' : '\\0')) return false;
    if (has_gripper_speed) {
        bool angles = strchr(p, ',') != NULL;
        if (!RobotProtocol_U32(&p, &cmd->gripper_dps, angles ? ',' : '\\0')) return false;
        if (angles && (!RobotProtocol_U32(&p, &cmd->theta, ',') ||
            !RobotProtocol_U32(&p, &cmd->open_angle, ',') || !RobotProtocol_U32(&p, &cmd->close_angle, ',') ||
            !RobotProtocol_U32(&p, &cmd->base_home, ',') || !RobotProtocol_U32(&p, &cmd->base_tilt, '\\0'))) return false;
    }""";assert a in s;s=s.replace(a,b).replace('cmd->gripper_dps >= 6U && cmd->gripper_dps <= 300U;','cmd->gripper_dps >= 6U && cmd->gripper_dps <= 300U && cmd->theta <= 270U &&\n        cmd->open_angle <= 270U && cmd->close_angle <= 270U && cmd->base_home <= 360U && cmd->base_tilt <= 360U;');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c');s=p.read_text(encoding='utf-8').replace('static uint32_t s_gripper_open_ms;','static uint32_t s_gripper_open_ms, s_sequence_gripper_dps;\nstatic uint16_t s_sequence_theta;\nstatic int32_t s_sequence_gripper_start, s_sequence_gripper_end;')
s=s.replace("s_grab_step == 2U && !ServoControl_SetAngle('G', 0U)","s_grab_step == 2U && !ServoControl_SetAngle('G', s_sequence_steps[2].gripper)")
s=s.replace('s_release_mode && s_grab_step == 2U ? 60U : step->gripper','s_release_mode && s_grab_step == 2U ? steps[1].gripper : step->gripper')
a='''    s_grab_gripper_opening = s_grab_step > 0U &&
        steps[s_grab_step - 1U].gripper == 0U && gripper == 60U;
    if (s_grab_gripper_opening) gripper = 0U;''';b='''    s_grab_gripper_opening = s_grab_step == 6U && steps[5].gripper != gripper;
    if (s_grab_gripper_opening) {
        s_sequence_gripper_start = steps[5].gripper; s_sequence_gripper_end = gripper;
        uint32_t distance = (uint32_t)(s_sequence_gripper_end > s_sequence_gripper_start ?
            s_sequence_gripper_end - s_sequence_gripper_start : s_sequence_gripper_start - s_sequence_gripper_end);
        s_gripper_open_ms = (distance * 1000U + s_sequence_gripper_dps - 1U) / s_sequence_gripper_dps;
        gripper = (uint16_t)s_sequence_gripper_start;
    }''';assert a in s;s=s.replace(a,b).replace("!ServoControl_SetAngle('T', 0U) || !ServoControl_SetAngle('G', gripper)","!ServoControl_SetAngle('T', s_sequence_theta) || !ServoControl_SetAngle('G', gripper)")
s=s.replace('uint16_t angle = (uint16_t)((60U * progress + s_gripper_open_ms / 2U) / s_gripper_open_ms);','int32_t delta = s_sequence_gripper_end - s_sequence_gripper_start;\n        uint16_t angle = (uint16_t)(s_sequence_gripper_start + delta * (int32_t)progress / (int32_t)s_gripper_open_ms);')
s=s.replace('        s_gripper_open_ms = ROBOT_GRIPPER_OPEN_MS;', '        s_gripper_open_ms = ROBOT_GRIPPER_OPEN_MS;\n        s_sequence_gripper_dps = 60U; s_sequence_theta = 0U;')
s=s.replace('            s_gripper_open_ms = (60000U + settings.gripper_dps - 1U) / settings.gripper_dps;','''            s_sequence_gripper_dps = settings.gripper_dps;
            s_sequence_theta = (uint16_t)settings.theta;
            for (unsigned i = 0; i < 8U; ++i) {
                s_sequence_steps[i].gripper = (uint16_t)(s_sequence_steps[i].gripper == 60U ? settings.open_angle : settings.close_angle);
                s_sequence_steps[i].base = (uint16_t)(s_sequence_steps[i].base == 248U ? settings.base_home : settings.base_tilt);
            }''');p.write_text(s,encoding='utf-8')
base=Path('yundong_part/application/app/src/main/java/com/example/app');p=base/'SequenceSettings.kt';s=p.read_text(encoding='utf-8').replace('val gripperDps: Int = 60,','val gripperDps: Int = 60,\n    val theta: Int = 0, val openAngle: Int = 60, val closeAngle: Int = 0,\n    val baseHome: Int = 248, val baseTilt: Int = 140,').replace('gripperDps in 6..300','gripperDps in 6..300 && theta in 0..270 && openAngle in 0..270 && closeAngle in 0..270 && baseHome in 0..360 && baseTilt in 0..360').replace('$downRpm,$gripperDps\\n','$downRpm,$gripperDps,$theta,$openAngle,$closeAngle,$baseHome,$baseTilt\\n').replace('gripper: String = "60"):','gripper: String = "60", theta: String = "0", open: String = "60", close: String = "0",\n                  baseHome: String = "248", baseTilt: String = "140"):').replace('down.trim().toInt(), gripper.trim().toInt())','down.trim().toInt(), gripper.trim().toInt(), theta.trim().toInt(), open.trim().toInt(),\n                close.trim().toInt(), baseHome.trim().toInt(), baseTilt.trim().toInt())');p.write_text(s,encoding='utf-8')
p=base/'MainActivity.kt';s=p.read_text(encoding='utf-8');a='    var gripper by rememberSaveable { mutableStateOf("60") }';s=s.replace(a,a+'''
    var theta by rememberSaveable { mutableStateOf("0") }
    var openAngle by rememberSaveable { mutableStateOf("60") }
    var closeAngle by rememberSaveable { mutableStateOf("0") }
    var baseHome by rememberSaveable { mutableStateOf("248") }
    var baseTilt by rememberSaveable { mutableStateOf("140") }''');a='    DistanceInput("夹子张开速度（度/秒，6–300）", gripper, true) { gripper = it }';s=s.replace(a,a+'''
    DistanceInput("转盘角度（0–270 度）", theta, true) { theta = it }
    DistanceInput("夹子张开角度（0–270 度）", openAngle, true) { openAngle = it }
    DistanceInput("夹子夹紧角度（0–270 度）", closeAngle, true) { closeAngle = it }
    DistanceInput("基座初始位（0–360 度）", baseHome, true) { baseHome = it }
    DistanceInput("基座翻转位（0–360 度）", baseTilt, true) { baseTilt = it }''').replace('parse(r1, z1, r2, z2, radial, up, down, gripper)','parse(r1, z1, r2, z2, radial, up, down, gripper, theta, openAngle, closeAngle, baseHome, baseTilt)');p.write_text(s,encoding='utf-8')
