#ifndef CAMERA_POSITION_PROTOCOL_H
#define CAMERA_POSITION_PROTOCOL_H
#include "robot_distance_protocol.h"
#include <math.h>

typedef struct { float x, y; } CameraCenter;
/* ring_index: 0 = green block, 1/2/3 = rings sorted from left to right. */
typedef struct { uint32_t fine_rpm, coarse_rpm; int32_t offset_mm; float center_x, center_y; bool custom_center; } CameraAlignSettings;
static inline CameraAlignSettings CameraProtocol_DefaultSettings(void)
{
    return (CameraAlignSettings){10U,20U,10,256.0f,160.0f,false};
}
typedef struct {
    uint32_t sequence, forward_ppm, lateral_ppm, ring_index;
    CameraAlignSettings settings;
} CameraAlignCommand;
/* Camera coordinate: unsigned integer or exactly one fractional digit.
 * Keep strict syntax (no sign, exponent, NaN, whitespace or extra precision). */
static inline bool CameraProtocol_Pixel(const char **cursor, float *value, char delimiter, uint32_t limit)
{
    const char *p = *cursor;
    if (*p < '0' || *p > '9') return false;
    uint32_t whole = 0U, tenth = 0U;
    while (*p >= '0' && *p <= '9') {
        whole = whole * 10U + (uint32_t)(*p++ - '0');
        if (whole >= limit) return false;
    }
    if (*p == '.') {
        ++p;
        if (*p < '0' || *p > '9') return false;
        tenth = (uint32_t)(*p++ - '0');
    }
    if (*p != delimiter) return false;
    *value = (float)whole + (float)tenth * 0.1f;
    *cursor = p + (delimiter == '\0' ? 0 : 1);
    return true;
}
static inline bool CameraProtocol_ParseRings(const char *frame, CameraCenter centers[3])
{
    if (strncmp(frame, "RINGS,", 6U) != 0) return false;
    const char *p = frame + 6U;
    for (unsigned i = 0; i < 3U; ++i) {
        float x, y;
        if (!CameraProtocol_Pixel(&p, &x, ',', 512U) ||
            !CameraProtocol_Pixel(&p, &y, i == 2U ? '\0' : ',', 320U)) return false;
        centers[i].x = x; centers[i].y = y;
        if (i > 0U && centers[i].x < centers[i - 1U].x) return false;
    }
    return true;
}
/* camera/color_v4.py: x=xxx,y=yyy, with LF/CRLF removed by the receiver. */
static inline bool CameraProtocol_ParseCenter(const char *frame, CameraCenter *center)
{
    if (strlen(frame) != 11U || frame[0] != 'x' || frame[1] != '=' ||
        frame[5] != ',' || frame[6] != 'y' || frame[7] != '=') return false;
    unsigned x = 0U, y = 0U;
    for (unsigned i = 0; i < 3U; ++i) {
        if (frame[2+i] < '0' || frame[2+i] > '9' || frame[8+i] < '0' || frame[8+i] > '9') return false;
        x = x * 10U + (unsigned)(frame[2+i] - '0');
        y = y * 10U + (unsigned)(frame[8+i] - '0');
    }
    if (x >= 512U || y >= 320U) return false;
    center->x = (uint16_t)x; center->y = (uint16_t)y;
    return true;
}
/* Optional target coordinates preserve legacy frames and settings initializers. */
static inline float CameraProtocol_TargetX(const CameraAlignSettings *s) { return s->custom_center ? s->center_x : 256.0f; }
static inline float CameraProtocol_TargetY(const CameraAlignSettings *s) { return s->custom_center ? s->center_y : 160.0f; }
static inline bool CameraProtocol_TargetSuffix(const char **cursor, CameraAlignSettings *s)
{
    const char *p = *cursor;
    if (*p == '\0') return true;
    if (*p++ != ',' || !CameraProtocol_Pixel(&p, &s->center_x, ',', 512U) ||
        !CameraProtocol_Pixel(&p, &s->center_y, '\0', 320U)) return false;
    s->custom_center = true;
    *cursor = p;
    return true;
}
static inline bool CameraProtocol_FinalU32Target(const char **cursor, uint32_t *value, CameraAlignSettings *s)
{
    bool has_target = strchr(*cursor, ',') != NULL;
    if (!RobotProtocol_U32(cursor, value, has_target ? ',' : '\0')) return false;
    if (has_target) --*cursor;
    return CameraProtocol_TargetSuffix(cursor, s);
}
static inline bool CameraProtocol_ParseAlign(const char *frame, CameraAlignCommand *cmd)
{
    const char *p;
    cmd->ring_index = 0U;
    cmd->settings = CameraProtocol_DefaultSettings();
    if (strncmp(frame, "ALIGN_CFG,", 10U) == 0) {
        p = frame + 10U;
        CameraAlignSettings *v = &cmd->settings;
        if (!RobotProtocol_U32(&p, &cmd->sequence, ',') ||
            !RobotProtocol_U32(&p, &cmd->ring_index, ',') ||
            !RobotProtocol_U32(&p, &cmd->forward_ppm, ',') ||
            !RobotProtocol_U32(&p, &cmd->lateral_ppm, ',') ||
            !RobotProtocol_U32(&p, &v->fine_rpm, ',') || !RobotProtocol_U32(&p, &v->coarse_rpm, ',')) return false;
        bool negative = *p == '-'; if (negative) ++p;
        uint32_t offset;
        if (!CameraProtocol_FinalU32Target(&p, &offset, v) || offset > 100U) return false;
        v->offset_mm = negative ? -(int32_t)offset : (int32_t)offset;
        return cmd->sequence != 0U && cmd->ring_index <= 3U &&
            cmd->forward_ppm >= 1U && cmd->forward_ppm <= 1000000U &&
            cmd->lateral_ppm >= 1U && cmd->lateral_ppm <= 1000000U &&
            v->fine_rpm >= 5U && v->fine_rpm <= 60U && v->coarse_rpm >= 5U && v->coarse_rpm <= 60U;
    }
    if (strncmp(frame, "ALIGN_RING,", 11U) == 0) {
        p = frame + 11U;
        if (!RobotProtocol_U32(&p, &cmd->sequence, ',') ||
            !RobotProtocol_U32(&p, &cmd->ring_index, ',') ||
            cmd->ring_index < 1U || cmd->ring_index > 3U) return false;
    } else if (strncmp(frame, "ALIGN,", 6U) == 0) {
        p = frame + 6U;
        if (!RobotProtocol_U32(&p, &cmd->sequence, ',')) return false;
    } else return false;
    if (!RobotProtocol_U32(&p, &cmd->forward_ppm, ',') ||
        !RobotProtocol_U32(&p, &cmd->lateral_ppm, '\0')) return false;
    return cmd->sequence != 0U && cmd->forward_ppm >= 1U && cmd->forward_ppm <= 1000000U &&
           cmd->lateral_ppm >= 1U && cmd->lateral_ppm <= 1000000U;
}
/* 0 px tolerance on each center axis; calculate full X/Y errors. Exact scale 1.090819 mm/px.
 * Motor command mapping after on-car feedback: image right -> B; image down -> L.
 * Command letters describe the existing driver mapping, not a verified physical heading. */
#define CAMERA_CORRECTION_COARSE_PX    20U
#define CAMERA_CORRECTION_FINE_RPM     10U
#define CAMERA_CORRECTION_COARSE_RPM   20U
#ifndef CAMERA_IMAGE_RIGHT_DIRECTION
#define CAMERA_IMAGE_RIGHT_DIRECTION 'B'
#endif
#ifndef CAMERA_IMAGE_DOWN_DIRECTION
#define CAMERA_IMAGE_DOWN_DIRECTION 'L'
#endif
static inline char CameraProtocol_Opposite(char dir)
{
    return dir == 'F' ? 'B' : dir == 'B' ? 'F' : dir == 'L' ? 'R' : 'L';
}
/* Select maximum wheel speed from the larger image error. */
static inline uint16_t CameraProtocol_ConfiguredRpm(const CameraCenter *center, const CameraAlignSettings *settings)
{
    float ax = fabsf(center->x - CameraProtocol_TargetX(settings)), ay = fabsf(center->y - CameraProtocol_TargetY(settings));
    return (ax > CAMERA_CORRECTION_COARSE_PX || ay > CAMERA_CORRECTION_COARSE_PX) ?
        settings->coarse_rpm : settings->fine_rpm;
}
typedef struct { uint32_t sequence, ppm, rpm, step_mm, reverse; } CameraParallelCommand;
/* Decimal samples remain fractional through averaging and distance conversion.
 * Convert to integral pulses only at the final output. */
static inline bool CameraProtocol_CenterMeanAxes(double sum_x, double sum_y, uint32_t count,
    uint32_t forward_ppm, uint32_t lateral_ppm, const CameraAlignSettings *settings,
    int32_t pulses[2], uint16_t speeds[2], bool *aligned)
{
    if (count == 0U || count > 50U || !isfinite(sum_x) || !isfinite(sum_y) ||
        sum_x < 0.0 || sum_y < 0.0 || sum_x >= 512.0 * count || sum_y >= 320.0 * count ||
        forward_ppm == 0U || forward_ppm > 1000000U || lateral_ppm == 0U || lateral_ppm > 1000000U) return false;
    double errors[2] = {sum_x / count - CameraProtocol_TargetX(settings), sum_y / count - CameraProtocol_TargetY(settings)};
    const uint32_t ppm[2] = {forward_ppm, lateral_ppm};
    *aligned = true;
    for (unsigned i = 0; i < 2U; ++i) {
        double error = errors[i];
        speeds[i] = fabs(error) > CAMERA_CORRECTION_COARSE_PX ? settings->coarse_rpm : settings->fine_rpm;
        pulses[i] = 0;
        if (error == 0.0) continue;
        *aligned = false;
        pulses[i] = (int32_t)lround(error * 1.090819 * ppm[i] / 1000.0);
        if (pulses[i] == 0) pulses[i] = error > 0 ? 1 : -1;
        if ((i == 0U && CAMERA_IMAGE_RIGHT_DIRECTION == 'F') ||
            (i == 1U && CAMERA_IMAGE_DOWN_DIRECTION == 'R')) pulses[i] = -pulses[i];
    }
    return true;
}
/* Mix unrounded X/Y travel first; round only each final wheel pulse target. */
static inline bool CameraProtocol_CenterProfile(const CameraCenter *center, uint32_t forward_ppm,
    uint32_t lateral_ppm, const CameraAlignSettings *settings, int32_t pulses[4], uint16_t speeds[4], bool *aligned)
{
    if (!isfinite(center->x) || !isfinite(center->y) || center->x < 0 || center->y < 0 ||
        center->x >= 512U || center->y >= 320U || forward_ppm == 0U || forward_ppm > 1000000U ||
        lateral_ppm == 0U || lateral_ppm > 1000000U) return false;
    double dx = (double)center->x - CameraProtocol_TargetX(settings), dy = (double)center->y - CameraProtocol_TargetY(settings);
    *aligned = dx == 0.0 && dy == 0.0;
    double x = dx * 1.090819 * forward_ppm / 1000.0;
    double y = dy * 1.090819 * lateral_ppm / 1000.0;
    if (CAMERA_IMAGE_RIGHT_DIRECTION == 'F') x = -x;
    if (CAMERA_IMAGE_DOWN_DIRECTION == 'R') y = -y;
    const double targets[4] = {x+y, x-y, x+y, x-y};
    uint32_t largest = 0U;
    for (unsigned i = 0; i < 4U; ++i) {
        pulses[i] = (int32_t)lround(targets[i]);
        if (pulses[i] == 0 && targets[i] != 0.0) pulses[i] = targets[i] > 0 ? 1 : -1;
        uint32_t magnitude = (uint32_t)(pulses[i] < 0 ? -pulses[i] : pulses[i]);
        if (magnitude > largest) largest = magnitude;
    }
    uint32_t rpm = CameraProtocol_ConfiguredRpm(center, settings);
    for (unsigned i = 0; i < 4U; ++i) {
        uint32_t magnitude = (uint32_t)(pulses[i] < 0 ? -pulses[i] : pulses[i]);
        speeds[i] = largest == 0U ? (uint16_t)rpm : (uint16_t)(((uint64_t)rpm*magnitude + largest/2U)/largest);
        if (speeds[i] == 0U) speeds[i] = 1U;
    }
    return true;
}
static inline bool CameraProtocol_ParseParallel(const char *frame, CameraParallelCommand *cmd)
{
    if (strncmp(frame, "PARALLEL,", 9U) != 0) return false;
    const char *p = frame + 9U;
    return RobotProtocol_U32(&p, &cmd->sequence, ',') && RobotProtocol_U32(&p, &cmd->ppm, ',') &&
        RobotProtocol_U32(&p, &cmd->rpm, ',') && RobotProtocol_U32(&p, &cmd->step_mm, ',') &&
        RobotProtocol_U32(&p, &cmd->reverse, '\0') && cmd->sequence != 0U && cmd->ppm >= 1U &&
        cmd->ppm <= 1000000U && cmd->rpm >= 5U && cmd->rpm <= 60U && cmd->step_mm >= 1U &&
        cmd->step_mm <= 10U && cmd->reverse <= 1U;
}
/* Least-squares slope using all three centers, in thousandths (image Y points down).
 * Reject narrow/degenerate baselines and non-collinear detections (>5 px residual). */
static inline bool CameraProtocol_RingSlope(const CameraCenter c[3], float *slope)
{
    if (c[2].x < c[0].x + 80.0f || c[1].x <= c[0].x || c[2].x <= c[1].x) return false;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (unsigned i = 0; i < 3U; ++i) {
        if (!isfinite(c[i].x) || !isfinite(c[i].y) || c[i].x < 0 || c[i].x >= 512 || c[i].y < 0 || c[i].y >= 320) return false;
        sx += c[i].x; sy += c[i].y; sxx += (double)c[i].x*c[i].x; sxy += (double)c[i].x*c[i].y;
    }
    double den = 3*sxx - sx*sx, num = 3*sxy - sx*sy;
    if (den <= 0) return false;
    for (unsigned i = 0; i < 3U; ++i) {
        double residual = (3*(double)c[i].y - sy)*den - num*(3*(double)c[i].x - sx);
        if (fabs(residual) > 15*den) return false;
    }
    *slope = (float)(num*1000.0/den); return true;
}
/* Pixel tolerance is independent of ring spacing: a wide baseline must not
 * make a visible height difference pass the horizontal completion check. */
static inline float CameraProtocol_RingYSpread(const CameraCenter c[3])
{
    float lo = c[0].y, hi = c[0].y;
    for (unsigned i = 1; i < 3U; ++i) {
        if (c[i].y < lo) lo = c[i].y;
        if (c[i].y > hi) hi = c[i].y;
    }
    return hi - lo;
}
#endif
