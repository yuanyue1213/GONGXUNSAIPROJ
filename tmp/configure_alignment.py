from pathlib import Path
p=Path('yundong_part/shared/camera_position_protocol.h');s=p.read_text(encoding='utf-8');s=s.replace('typedef struct { uint32_t sequence, forward_ppm, lateral_ppm, ring_index; } CameraAlignCommand;', '''typedef struct {
    uint32_t x, y, scale, fine_gain, coarse_gain, fine_rpm, coarse_rpm, max_mm;
    int32_t offset_mm;
} CameraAlignSettings;
static inline CameraAlignSettings CameraProtocol_DefaultSettings(void)
{
    return (CameraAlignSettings){256U,160U,1090819U,30U,50U,10U,20U,15U,10};
}
typedef struct {
    uint32_t sequence, forward_ppm, lateral_ppm, ring_index;
    CameraAlignSettings settings;
} CameraAlignCommand;''')
s=s.replace('    cmd->ring_index = 0U;','''    cmd->ring_index = 0U;
    cmd->settings = CameraProtocol_DefaultSettings();
    if (strncmp(frame, "ALIGN_CFG,", 10U) == 0) {
        p = frame + 10U;
        CameraAlignSettings *v = &cmd->settings;
        if (!RobotProtocol_U32(&p, &cmd->sequence, ',') ||
            !RobotProtocol_U32(&p, &cmd->ring_index, ',') ||
            !RobotProtocol_U32(&p, &cmd->forward_ppm, ',') ||
            !RobotProtocol_U32(&p, &cmd->lateral_ppm, ',') ||
            !RobotProtocol_U32(&p, &v->x, ',') || !RobotProtocol_U32(&p, &v->y, ',') ||
            !RobotProtocol_U32(&p, &v->scale, ',') ||
            !RobotProtocol_U32(&p, &v->fine_gain, ',') || !RobotProtocol_U32(&p, &v->coarse_gain, ',') ||
            !RobotProtocol_U32(&p, &v->fine_rpm, ',') || !RobotProtocol_U32(&p, &v->coarse_rpm, ',') ||
            !RobotProtocol_U32(&p, &v->max_mm, ',')) return false;
        bool negative = *p == '-'; if (negative) ++p;
        uint32_t offset;
        if (!RobotProtocol_U32(&p, &offset, '\\0') || offset > 100U) return false;
        v->offset_mm = negative ? -(int32_t)offset : (int32_t)offset;
        return cmd->sequence != 0U && cmd->ring_index <= 3U &&
            cmd->forward_ppm >= 1U && cmd->forward_ppm <= 1000000U &&
            cmd->lateral_ppm >= 1U && cmd->lateral_ppm <= 1000000U &&
            v->x < 512U && v->y < 320U && v->scale >= 10000U && v->scale <= 10000000U &&
            v->fine_gain >= 1U && v->fine_gain <= 100U && v->coarse_gain >= 1U && v->coarse_gain <= 100U &&
            v->fine_rpm >= 5U && v->fine_rpm <= 60U && v->coarse_rpm >= 5U && v->coarse_rpm <= 60U &&
            v->max_mm >= 1U && v->max_mm <= 100U;
    }''')
s=s.replace('static inline uint16_t CameraProtocol_CorrectionRpm(const CameraCenter *center)','static inline uint16_t CameraProtocol_ConfiguredRpm(const CameraCenter *center, const CameraAlignSettings *settings)').replace('static inline bool CameraProtocol_Correction(const CameraCenter *center, char *direction, uint32_t *mm)','static inline bool CameraProtocol_ConfiguredCorrection(const CameraCenter *center, const CameraAlignSettings *settings, char *direction, uint32_t *mm)').replace('center->x - 256, dy = (int)center->y - 160','center->x - (int)settings->x, dy = (int)center->y - (int)settings->y').replace('CAMERA_CORRECTION_COARSE_RPM : CAMERA_CORRECTION_FINE_RPM;','settings->coarse_rpm : settings->fine_rpm;').replace('CAMERA_CORRECTION_COARSE_GAIN : CAMERA_CORRECTION_FINE_GAIN;','settings->coarse_gain : settings->fine_gain;').replace('pixels * 1090819U * gain','pixels * settings->scale * gain').replace('if (*mm > CAMERA_CORRECTION_MAX_MM) *mm = CAMERA_CORRECTION_MAX_MM;','if (*mm > settings->max_mm) *mm = settings->max_mm;')
s=s.replace('#endif\n', '''static inline uint16_t CameraProtocol_CorrectionRpm(const CameraCenter *center)
{
    CameraAlignSettings settings = CameraProtocol_DefaultSettings();
    return CameraProtocol_ConfiguredRpm(center, &settings);
}
static inline bool CameraProtocol_Correction(const CameraCenter *center, char *direction, uint32_t *mm)
{
    CameraAlignSettings settings = CameraProtocol_DefaultSettings();
    return CameraProtocol_ConfiguredCorrection(center, &settings, direction, mm);
}
#endif
''') if False else s
# Insert wrappers only at final include guard, not direction macro guards.
pos=s.rfind('#endif');s=s[:pos]+'''static inline uint16_t CameraProtocol_CorrectionRpm(const CameraCenter *center)
{
    CameraAlignSettings settings = CameraProtocol_DefaultSettings();
    return CameraProtocol_ConfiguredRpm(center, &settings);
}
static inline bool CameraProtocol_Correction(const CameraCenter *center, char *direction, uint32_t *mm)
{
    CameraAlignSettings settings = CameraProtocol_DefaultSettings();
    return CameraProtocol_ConfiguredCorrection(center, &settings, direction, mm);
}
'''+s[pos:];p.write_text(s,encoding='utf-8')
p=Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c');s=p.read_text(encoding='utf-8').replace('#define ROBOT_COMMAND_MAX_LENGTH       63U','#define ROBOT_COMMAND_MAX_LENGTH       127U').replace('static bool s_align_active;','static bool s_align_active;\nstatic CameraAlignSettings s_align_settings;').replace('CameraProtocol_Correction(&center, &direction, &mm)','CameraProtocol_ConfiguredCorrection(&center, &s_align_settings, &direction, &mm)').replace('        direction = CAMERA_IMAGE_RIGHT_DIRECTION; mm = 10U; compensating = true;','''        if (s_align_settings.offset_mm == 0) { RobotControl_CancelAlignment(); return; }
        direction = s_align_settings.offset_mm > 0 ? CAMERA_IMAGE_RIGHT_DIRECTION :
            CameraProtocol_Opposite(CAMERA_IMAGE_RIGHT_DIRECTION);
        mm = (uint32_t)(s_align_settings.offset_mm > 0 ? s_align_settings.offset_mm : -s_align_settings.offset_mm);
        compensating = true;''').replace('compensating ? CAMERA_CORRECTION_FINE_RPM : CameraProtocol_CorrectionRpm(&center)','compensating ? s_align_settings.fine_rpm : CameraProtocol_ConfiguredRpm(&center, &s_align_settings)').replace('strncmp(frame, "ALIGN_RING,", 11U) == 0)', 'strncmp(frame, "ALIGN_RING,", 11U) == 0 || strncmp(frame, "ALIGN_CFG,", 10U) == 0)').replace('s_align_forward_ppm = cmd.forward_ppm; s_align_lateral_ppm = cmd.lateral_ppm;','s_align_forward_ppm = cmd.forward_ppm; s_align_lateral_ppm = cmd.lateral_ppm;\n        s_align_settings = cmd.settings;');s=s.replace('Both camera modes finish with one 10 mm move along image +X.','Both camera modes apply the configured final X offset once.');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/esp32s3/hello_world/main/robot_remote_main.c');s=p.read_text(encoding='utf-8').replace('#define MAX_FRAME_LENGTH        63','#define MAX_FRAME_LENGTH        127').replace('strncmp(frame, "ALIGN_RING,", 11U) == 0)','strncmp(frame, "ALIGN_RING,", 11U) == 0 || strncmp(frame, "ALIGN_CFG,", 10U) == 0)');p.write_text(s,encoding='utf-8')
