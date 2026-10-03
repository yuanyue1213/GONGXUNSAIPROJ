from pathlib import Path
p=Path('yundong_part/shared/camera_position_protocol.h');s=p.read_text(encoding='utf-8')
a=s.index('typedef struct {\n    uint32_t x, y, scale');b=s.index('typedef struct {\n    uint32_t sequence',a)
s=s[:a]+'''typedef struct { uint32_t fine_rpm, coarse_rpm; int32_t offset_mm; } CameraAlignSettings;
static inline CameraAlignSettings CameraProtocol_DefaultSettings(void)
{
    return (CameraAlignSettings){10U,20U,10};
}
'''+s[b:]
a=s.index('            !RobotProtocol_U32(&p, &v->x');b=s.index('        bool negative',a)
s=s[:a]+'''            !RobotProtocol_U32(&p, &v->fine_rpm, ',') || !RobotProtocol_U32(&p, &v->coarse_rpm, ',')) return false;
'''+s[b:]
a=s.index('            v->x < 512U');b=s.index('\n    }',a)
s=s[:a]+'''            v->fine_rpm >= 5U && v->fine_rpm <= 60U && v->coarse_rpm >= 5U && v->coarse_rpm <= 60U;'''+s[b:]
s=s.replace('center->x - (int)settings->x, dy = (int)center->y - (int)settings->y','center->x - 256, dy = (int)center->y - 160')
s=s.replace('static inline bool CameraProtocol_ConfiguredCorrection(const CameraCenter *center, const CameraAlignSettings *settings, char *direction, uint32_t *mm)','static inline bool CameraProtocol_Correction(const CameraCenter *center, char *direction, uint32_t *mm)').replace('settings->coarse_gain : settings->fine_gain','CAMERA_CORRECTION_COARSE_GAIN : CAMERA_CORRECTION_FINE_GAIN').replace('pixels * settings->scale * gain','pixels * 1090819U * gain').replace('if (*mm > settings->max_mm) *mm = settings->max_mm;','if (*mm > CAMERA_CORRECTION_MAX_MM) *mm = CAMERA_CORRECTION_MAX_MM;')
a=s.rfind('static inline bool CameraProtocol_Correction(');b=s.index('#endif',a);s=s[:a]+s[b:];p.write_text(s,encoding='utf-8')
p=Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c');s=p.read_text(encoding='utf-8').replace('CameraProtocol_ConfiguredCorrection(&center, &s_align_settings, &direction, &mm)','CameraProtocol_Correction(&center, &direction, &mm)');p.write_text(s,encoding='utf-8')
base=Path('yundong_part/application/app/src/main/java/com/example/app')
(base/'AlignmentSettings.kt').write_text('''package com.example.app

internal data class AlignmentSettings(
    val fineRpm: Int = 10, val coarseRpm: Int = 20, val offsetMm: Int = 10,
) {
    fun valid() = fineRpm in 5..60 && coarseRpm in 5..60 && offsetMm in -100..100
    fun frame(sequence: Long, ring: Int, forward: Int, lateral: Int) =
        "ALIGN_CFG,$sequence,$ring,$forward,$lateral,$fineRpm,$coarseRpm,$offsetMm\\n"
    companion object {
        fun parse(fineRpm: String, coarseRpm: String, offsetMm: String): AlignmentSettings? = try {
            AlignmentSettings(fineRpm.trim().toInt(), coarseRpm.trim().toInt(), offsetMm.trim().toInt())
                .takeIf { it.valid() }
        } catch (_: IllegalArgumentException) { null }
    }
}
''',encoding='utf-8')
p=base/'MainActivity.kt';s=p.read_text(encoding='utf-8');lines=s.splitlines();removeVars=['targetX','targetY','pixelScale','fineGain','coarseGain','maxStep'];lines=[line for line in lines if not any('var '+name+' by rememberSaveable' in line for name in removeVars) and not any('DistanceInput("'+label in line for label in ['目标中心','比例（mm/px','精调比例','粗调比例','单步上限'])];s='\n'.join(lines)+'\n';s=s.replace('AlignmentSettings.parse(targetX, targetY, pixelScale, fineGain,\n        coarseGain, fineRpm, coarseRpm, maxStep, xOffset)','AlignmentSettings.parse(fineRpm, coarseRpm, xOffset)');p.write_text(s,encoding='utf-8')
