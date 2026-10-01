#ifndef CAMERA_POSITION_PROTOCOL_H
#define CAMERA_POSITION_PROTOCOL_H
#include "robot_distance_protocol.h"

typedef struct { uint16_t x, y; } CameraCenter;
typedef struct { uint32_t sequence, forward_ppm, lateral_ppm; } CameraAlignCommand;
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
    if (strncmp(frame, "ALIGN,", 6U) != 0) return false;
    const char *p = frame + 6U;
    if (!RobotProtocol_U32(&p, &cmd->sequence, ',') ||
        !RobotProtocol_U32(&p, &cmd->forward_ppm, ',') ||
        !RobotProtocol_U32(&p, &cmd->lateral_ppm, '\0')) return false;
    return cmd->sequence != 0U && cmd->forward_ppm >= 1U && cmd->forward_ppm <= 1000000U &&
           cmd->lateral_ppm >= 1U && cmd->lateral_ppm <= 1000000U;
}
/* 2 px tolerance, choose the larger error axis first. Exact scale 1.090819 mm/px.
 * Motor command mapping after on-car feedback: image right -> B; image down -> L.
 * Command letters describe the existing driver mapping, not a verified physical heading. */
#define CAMERA_CORRECTION_COARSE_PX    20U
#define CAMERA_CORRECTION_FINE_GAIN    30U
#define CAMERA_CORRECTION_COARSE_GAIN  50U
#define CAMERA_CORRECTION_FINE_RPM     10U
#define CAMERA_CORRECTION_COARSE_RPM   20U
#define CAMERA_CORRECTION_MAX_MM       15U
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
/* Match speed to the same larger-axis error used for the distance calculation. */
static inline uint16_t CameraProtocol_CorrectionRpm(const CameraCenter *center)
{
    int dx = (int)center->x - 256, dy = (int)center->y - 160;
    unsigned ax = (unsigned)(dx < 0 ? -dx : dx), ay = (unsigned)(dy < 0 ? -dy : dy);
    return (ax > CAMERA_CORRECTION_COARSE_PX || ay > CAMERA_CORRECTION_COARSE_PX) ?
        CAMERA_CORRECTION_COARSE_RPM : CAMERA_CORRECTION_FINE_RPM;
}
static inline bool CameraProtocol_Correction(const CameraCenter *center, char *direction, uint32_t *mm)
{
    int dx = (int)center->x - 256, dy = (int)center->y - 160;
    unsigned ax = (unsigned)(dx < 0 ? -dx : dx), ay = (unsigned)(dy < 0 ? -dy : dy);
    if (ax <= 2U && ay <= 2U) { *direction = 'S'; *mm = 0U; return false; }
    unsigned pixels;
    if (ax >= ay) {
        pixels = ax;
        *direction = dx > 0 ? CAMERA_IMAGE_RIGHT_DIRECTION :
            CameraProtocol_Opposite(CAMERA_IMAGE_RIGHT_DIRECTION);
    } else {
        pixels = ay;
        *direction = dy > 0 ? CAMERA_IMAGE_DOWN_DIRECTION :
            CameraProtocol_Opposite(CAMERA_IMAGE_DOWN_DIRECTION);
    }
    /* Coarse correction beyond 20 px, fine correction near the center.
     * Keep both stages capped; 64-bit arithmetic prevents scale * pixels * gain overflow. */
    unsigned gain = pixels > CAMERA_CORRECTION_COARSE_PX ?
        CAMERA_CORRECTION_COARSE_GAIN : CAMERA_CORRECTION_FINE_GAIN;
    *mm = (uint32_t)(((uint64_t)pixels * 1090819U * gain +
                     50000000U) / 100000000U);
    if (*mm == 0U) *mm = 1U;
    if (*mm > CAMERA_CORRECTION_MAX_MM) *mm = CAMERA_CORRECTION_MAX_MM;
    return true;
}
#endif
