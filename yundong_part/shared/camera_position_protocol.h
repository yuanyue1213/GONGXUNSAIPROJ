#ifndef CAMERA_POSITION_PROTOCOL_H
#define CAMERA_POSITION_PROTOCOL_H
#include "robot_distance_protocol.h"

typedef struct { uint16_t x, y; } CameraCenter;
/* ring_index: 0 = green block, 1/2/3 = rings sorted from left to right. */
typedef struct { uint32_t fine_rpm, coarse_rpm; int32_t offset_mm; } CameraAlignSettings;
static inline CameraAlignSettings CameraProtocol_DefaultSettings(void)
{
    return (CameraAlignSettings){10U,20U,10};
}
typedef struct {
    uint32_t sequence, forward_ppm, lateral_ppm, ring_index;
    CameraAlignSettings settings;
} CameraAlignCommand;
static inline bool CameraProtocol_ParseRings(const char *frame, CameraCenter centers[3])
{
    if (strncmp(frame, "RINGS,", 6U) != 0) return false;
    const char *p = frame + 6U;
    for (unsigned i = 0; i < 3U; ++i) {
        uint32_t x, y;
        if (!RobotProtocol_U32(&p, &x, ',') ||
            !RobotProtocol_U32(&p, &y, i == 2U ? '\0' : ',') ||
            x >= 512U || y >= 320U) return false;
        centers[i].x = (uint16_t)x; centers[i].y = (uint16_t)y;
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
        if (!RobotProtocol_U32(&p, &offset, '\0') || offset > 100U) return false;
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
    int dx = (int)center->x - 256, dy = (int)center->y - 160;
    unsigned ax = (unsigned)(dx < 0 ? -dx : dx), ay = (unsigned)(dy < 0 ? -dy : dy);
    return (ax > CAMERA_CORRECTION_COARSE_PX || ay > CAMERA_CORRECTION_COARSE_PX) ?
        settings->coarse_rpm : settings->fine_rpm;
}
typedef struct { uint32_t sequence, ppm, rpm, step_mm, reverse; } CameraParallelCommand;
/* Preserve sum/count exactly: 50 integer samples retain 0.02 px resolution.
 * Compare pixel tolerances before dividing; round only the final motor pulses. */
static inline bool CameraProtocol_CenterMeanAxes(uint32_t sum_x, uint32_t sum_y, uint32_t count,
    uint32_t forward_ppm, uint32_t lateral_ppm, const CameraAlignSettings *settings,
    int32_t pulses[2], uint16_t speeds[2], bool *aligned)
{
    if (count == 0U || count > 50U || sum_x > 511U * count || sum_y > 319U * count ||
        forward_ppm == 0U || forward_ppm > 1000000U || lateral_ppm == 0U || lateral_ppm > 1000000U) return false;
    int32_t errors[2] = {(int32_t)sum_x - (int32_t)(256U * count),
                         (int32_t)sum_y - (int32_t)(160U * count)};
    const uint32_t ppm[2] = {forward_ppm, lateral_ppm};
    int64_t denominator = (int64_t)1000000000 * count;
    *aligned = true;
    for (unsigned i = 0; i < 2U; ++i) {
        int32_t error = errors[i];
        uint32_t magnitude = (uint32_t)(error < 0 ? -error : error);
        speeds[i] = magnitude > CAMERA_CORRECTION_COARSE_PX * count ? settings->coarse_rpm : settings->fine_rpm;
        pulses[i] = 0;
        if (magnitude == 0U) continue; /* Skip only an exactly centered axis. */
        *aligned = false;
        int64_t raw = (int64_t)error * 1090819 * ppm[i];
        pulses[i] = (int32_t)((raw + (raw < 0 ? -denominator/2 : denominator/2)) / denominator);
        if (pulses[i] == 0) pulses[i] = error > 0 ? 1 : -1;
        if ((i == 0U && CAMERA_IMAGE_RIGHT_DIRECTION == 'F') ||
            (i == 1U && CAMERA_IMAGE_DOWN_DIRECTION == 'R')) pulses[i] = -pulses[i];
    }
    return true;
}
/* One camera sample -> full X/Y translation, with no gain or step-distance cap.
 * Wheel travel remains signed pulses; per-wheel RPM follows travel magnitude. */
static inline bool CameraProtocol_CenterProfile(const CameraCenter *center, uint32_t forward_ppm,
    uint32_t lateral_ppm, const CameraAlignSettings *settings, int32_t pulses[4], uint16_t speeds[4], bool *aligned)
{
    if (center->x >= 512U || center->y >= 320U || forward_ppm == 0U || forward_ppm > 1000000U ||
        lateral_ppm == 0U || lateral_ppm > 1000000U) return false;
    int32_t dx = (int32_t)center->x - 256, dy = (int32_t)center->y - 160;
    *aligned = dx == 0 && dy == 0;
    int64_t x_raw = (int64_t)dx * 1090819 * forward_ppm;
    int64_t y_raw = (int64_t)dy * 1090819 * lateral_ppm;
    int32_t x = (int32_t)((x_raw + (x_raw < 0 ? -500000000 : 500000000)) / 1000000000);
    int32_t y = (int32_t)((y_raw + (y_raw < 0 ? -500000000 : 500000000)) / 1000000000);
    if (x == 0 && dx != 0) x = dx > 0 ? 1 : -1;
    if (y == 0 && dy != 0) y = dy > 0 ? 1 : -1;
    if (CAMERA_IMAGE_RIGHT_DIRECTION == 'F') x = -x;
    if (CAMERA_IMAGE_DOWN_DIRECTION == 'R') y = -y;
    pulses[0] = pulses[2] = x + y;
    pulses[1] = pulses[3] = x - y;
    uint32_t largest = 0U;
    for (unsigned i = 0; i < 4U; ++i) {
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
static inline bool CameraProtocol_RingSlope(const CameraCenter c[3], int32_t *slope)
{
    if (c[2].x < c[0].x + 80U || c[1].x <= c[0].x || c[2].x <= c[1].x) return false;
    int64_t sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (unsigned i = 0; i < 3U; ++i) {
        sx += c[i].x; sy += c[i].y; sxx += (int64_t)c[i].x*c[i].x; sxy += (int64_t)c[i].x*c[i].y;
    }
    int64_t den = 3*sxx - sx*sx, num = 3*sxy - sx*sy;
    if (den <= 0) return false;
    for (unsigned i = 0; i < 3U; ++i) {
        int64_t residual = (3*(int64_t)c[i].y - sy)*den - num*(3*(int64_t)c[i].x - sx);
        if (residual < 0) residual = -residual;
        if (residual > 15*den) return false;
    }
    *slope = (int32_t)(num*1000/den); return true;
}
/* Pixel tolerance is independent of ring spacing: a wide baseline must not
 * make a visible height difference pass the horizontal completion check. */
static inline uint16_t CameraProtocol_RingYSpread(const CameraCenter c[3])
{
    uint16_t lo = c[0].y, hi = c[0].y;
    for (unsigned i = 1; i < 3U; ++i) {
        if (c[i].y < lo) lo = c[i].y;
        if (c[i].y > hi) hi = c[i].y;
    }
    return hi - lo;
}
#endif
