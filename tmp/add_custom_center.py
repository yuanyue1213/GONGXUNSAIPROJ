from pathlib import Path

root = Path('yundong_part')
def edit(path, old, new):
    p = root / path
    s = p.read_text(encoding='utf-8')
    assert old in s, (path, old)
    p.write_text(s.replace(old, new), encoding='utf-8')

app = 'application/app/src/main/java/com/example/app/'
edit(app+'AlignmentSettings.kt', 'val fineRpm: Int = 10, val coarseRpm: Int = 20, val offsetMm: Int = 10,',
     'val fineRpm: Int = 10, val coarseRpm: Int = 20, val offsetMm: Int = 10,\n    val centerX: Double = 256.0, val centerY: Double = 160.0,')
edit(app+'AlignmentSettings.kt', 'offsetMm in -100..100', 'offsetMm in -100..100 &&\n        validPixel(centerX, 512.0) && validPixel(centerY, 320.0)')
edit(app+'AlignmentSettings.kt', '$offsetMm\\n"', '$offsetMm${centerSuffix()}\\n"\n    internal fun centerSuffix() = if (centerX == 256.0 && centerY == 160.0) "" else\n        ",${pixel(centerX)},${pixel(centerY)}"\n    private fun pixel(value: Double) = java.math.BigDecimal.valueOf(value).stripTrailingZeros().toPlainString()')
edit(app+'AlignmentSettings.kt', 'fun parse(fineRpm: String, coarseRpm: String, offsetMm: String): AlignmentSettings? = try {',
     'private fun validPixel(value: Double, limit: Double) = value.isFinite() && value >= 0 && value < limit &&\n            java.math.BigDecimal.valueOf(value).stripTrailingZeros().scale() <= 1\n        fun parse(fineRpm: String, coarseRpm: String, offsetMm: String, centerX: String = "256", centerY: String = "160"): AlignmentSettings? = try {')
edit(app+'AlignmentSettings.kt', 'offsetMm.trim().toInt())', 'offsetMm.trim().toInt(),\n                centerX.trim().toDouble(), centerY.trim().toDouble())')
edit(app+'JointAlignmentSettings.kt', '${if (reverse) 1 else 0}\\n', '${if (reverse) 1 else 0}${settings.centerSuffix()}\\n')
edit(app+'MainActivity.kt', 'fun alignmentSettings() = AlignmentSettings.parse(fineRpm, coarseRpm, xOffset)',
     'var centerX by rememberSaveable { mutableStateOf("256") }\n    var centerY by rememberSaveable { mutableStateOf("160") }\n    fun alignmentSettings() = AlignmentSettings.parse(fineRpm, coarseRpm, xOffset, centerX, centerY)')
edit(app+'MainActivity.kt', 'Text("位置修正参数", style = MaterialTheme.typography.titleMedium)',
     'Text("位置修正参数", style = MaterialTheme.typography.titleMedium)\n        Text("画面分辨率 512×320，左上角为 (0,0)，支持一位小数。")\n        DistanceInput("目标中心 X（px，0–511.9）", centerX, !running, true) { centerX = it }\n        DistanceInput("目标中心 Y（px，0–319.9）", centerY, !running, true) { centerY = it }\n        Button(onClick = { centerX = "256"; centerY = "160" }, enabled = !running) { Text("恢复画面中心 (256,160)") }')
edit(app+'MainActivity.kt', '请检查位置修正参数及底盘标定', '请检查目标中心范围及精度、位置修正参数和底盘标定')
edit(app+'MainActivity.kt', '请填写实际轮距、摄像头偏移及底盘标定参数', '请检查目标中心范围及精度、实际轮距、摄像头偏移和底盘标定参数')

h = 'shared/camera_position_protocol.h'
edit(h, 'int32_t offset_mm; } CameraAlignSettings;', 'int32_t offset_mm; float center_x, center_y; bool custom_center; } CameraAlignSettings;')
edit(h, '(CameraAlignSettings){10U,20U,10}', '(CameraAlignSettings){10U,20U,10,256.0f,160.0f,false}')
edit(h, 'static inline bool CameraProtocol_ParseAlign(', '''/* Optional target coordinates preserve legacy frames and settings initializers. */
static inline float CameraProtocol_TargetX(const CameraAlignSettings *s) { return s->custom_center ? s->center_x : 256.0f; }
static inline float CameraProtocol_TargetY(const CameraAlignSettings *s) { return s->custom_center ? s->center_y : 160.0f; }
static inline bool CameraProtocol_TargetSuffix(const char **cursor, CameraAlignSettings *s)
{
    const char *p = *cursor;
    if (*p == '\\0') return true;
    if (*p++ != ',' || !CameraProtocol_Pixel(&p, &s->center_x, ',', 512U) ||
        !CameraProtocol_Pixel(&p, &s->center_y, '\\0', 320U)) return false;
    s->custom_center = true;
    *cursor = p;
    return true;
}
static inline bool CameraProtocol_ParseAlign(''')
edit(h, "RobotProtocol_U32(&p, &offset, '\\0')", "RobotProtocol_U32(&p, &offset, strchr(p, ',') ? ',' : '\\0')")
# U32 consumes the comma; suffix helper expects it.
edit(h, 'v->offset_mm = negative ?', "if (*p != '\\0') { --p; if (!CameraProtocol_TargetSuffix(&p, v)) return false; }\n        v->offset_mm = negative ?")
# Empty suffix after comma must be rejected as well; choose delimiter before parsing.
edit(h, 'uint32_t offset;\n        if', "uint32_t offset;\n        bool has_target = strchr(p, ',') != NULL;\n        if")
edit(h, "if (*p != '\\0') { --p;", 'if (has_target) { --p;')
edit(h, 'center->x - 256.0f', 'center->x - CameraProtocol_TargetX(settings)')
edit(h, 'center->y - 160.0f', 'center->y - CameraProtocol_TargetY(settings)')
edit(h, 'sum_x / count - 256.0, sum_y / count - 160.0', 'sum_x / count - CameraProtocol_TargetX(settings), sum_y / count - CameraProtocol_TargetY(settings)')
edit(h, '(double)center->x - 256.0, dy = (double)center->y - 160.0', '(double)center->x - CameraProtocol_TargetX(settings), dy = (double)center->y - CameraProtocol_TargetY(settings)')

h = 'shared/camera_pose_math.h'
edit(h, 'const char *p = frame + 11U;', 'const char *p = frame + 11U;\n    c->settings = CameraProtocol_DefaultSettings();')
edit(h, "RobotProtocol_U32(&p, &c->reverse, '\\0') &&", "RobotProtocol_U32(&p, &c->reverse, strchr(p, ',') ? ',' : '\\0') &&\n        CameraProtocol_TargetSuffix(*p == '\\0' && p[-1] != ',' ? &p : (p--, &p), &c->settings) &&")
edit(h, '(center.x - 256.0f)', '(center.x - CameraProtocol_TargetX(&c->settings))')
edit(h, '(center.y - 160.0f)', '(center.y - CameraProtocol_TargetY(&c->settings))')
edit(h, 'center.x == 256U && center.y == 160U', 'center.x == CameraProtocol_TargetX(&c->settings) && center.y == CameraProtocol_TargetY(&c->settings)')
edit(h, 'float desired_a = a;', 'float desired_a = a - (CameraProtocol_TargetX(&c->settings)-256.0f)*scale;\n    float desired_b = b + (CameraProtocol_TargetY(&c->settings)-160.0f)*scale;')
edit(h, 'cs*desired_a - sn*b', 'cs*desired_a - sn*desired_b')
edit(h, 'float ty = b + ey - (sn*desired_a + cs*b);', 'float ty = b + (center.y-160.0f)*scale - (sn*desired_a + cs*desired_b);')
