#ifndef CAMERA_POSE_MATH_H
#define CAMERA_POSE_MATH_H
#include "camera_position_protocol.h"
#include <math.h>

typedef struct {
    uint32_t sequence, ring_index, forward_ppm, lateral_ppm, wheelbase_mm, track_mm, reverse;
    int32_t camera_forward_mm, camera_left_mm;
    CameraAlignSettings settings;
} CameraPoseAlignCommand;
static inline bool CameraProtocol_ParsePoseAlign(const char *frame, CameraPoseAlignCommand *c)
{
    if (strncmp(frame, "ALIGN_POSE,", 11U) != 0) return false;
    const char *p = frame + 11U;
    return RobotProtocol_U32(&p, &c->sequence, ',') && RobotProtocol_U32(&p, &c->ring_index, ',') &&
        RobotProtocol_U32(&p, &c->forward_ppm, ',') && RobotProtocol_U32(&p, &c->lateral_ppm, ',') &&
        RobotProtocol_U32(&p, &c->settings.fine_rpm, ',') && RobotProtocol_U32(&p, &c->settings.coarse_rpm, ',') &&
        RobotProtocol_SequenceCoordinate(&p, &c->settings.offset_mm, 100U) &&
        RobotProtocol_U32(&p, &c->wheelbase_mm, ',') && RobotProtocol_U32(&p, &c->track_mm, ',') &&
        RobotProtocol_SequenceCoordinate(&p, &c->camera_forward_mm, 2000U) &&
        RobotProtocol_SequenceCoordinate(&p, &c->camera_left_mm, 2000U) && RobotProtocol_U32(&p, &c->reverse, '\0') &&
        c->sequence != 0U && c->ring_index >= 1U && c->ring_index <= 3U &&
        c->forward_ppm >= 1U && c->forward_ppm <= 1000000U && c->lateral_ppm >= 1U && c->lateral_ppm <= 1000000U &&
        c->settings.fine_rpm >= 5U && c->settings.fine_rpm <= 60U && c->settings.coarse_rpm >= 5U && c->settings.coarse_rpm <= 60U &&
        c->wheelbase_mm >= 50U && c->wheelbase_mm <= 2000U && c->track_mm >= 50U && c->track_mm <= 2000U && c->reverse <= 1U;
}
/* Camera +X maps to backward chassis travel; +Y to leftward travel.
 * Camera offsets locate the view-center ground point relative to chassis center.
 * Return one constant body-twist wheel profile for the complete measured pose
 * error. The SE(2) inverse accounts for changing heading during translation. */
static inline bool CameraProtocol_PoseProfile(const CameraPoseAlignCommand *c, const CameraCenter rings[3],
    int32_t pulses[4], uint16_t speeds[4], bool *aligned)
{
    if (c->ring_index < 1U || c->ring_index > 3U) return false;
    int32_t slope;
    if (!CameraProtocol_RingSlope(rings, &slope)) return false;
    CameraCenter center = rings[c->ring_index - 1U];
    const float scale = 1.090819f;
    float ex = ((int)center.x - 256) * scale + c->settings.offset_mm;
    float ey = ((int)center.y - 160) * scale;
    *aligned = fabsf(ex) <= 2.0f*scale && fabsf(ey) <= 2.0f*scale && CameraProtocol_RingYSpread(rings) <= 1U;
    if (*aligned) return true;
    if (slope == 0 && CameraProtocol_RingYSpread(rings) > 1U) return false;
    float theta = -atanf(slope / 1000.0f);
    if (c->reverse != 0U) theta = -theta;
    float cs = cosf(theta), sn = sinf(theta);
    float a = (float)c->camera_forward_mm, b = (float)c->camera_left_mm;
    float desired_a = a + c->settings.offset_mm;
    float tx = a - ((int)center.x-256)*scale - (cs*desired_a - sn*b);
    float ty = b + ey - (sn*desired_a + cs*b);
    float u = tx, v = ty;
    if (fabsf(theta) > 0.00001f) {
        float j = sn/theta, k = (1.0f-cs)/theta;
        float den = j*j + k*k;
        u = (j*tx + k*ty)/den;
        v = (-k*tx + j*ty)/den;
    }
    float f = u*c->forward_ppm/1000.0f, l = v*c->lateral_ppm/1000.0f;
    float r = (c->wheelbase_mm+c->track_mm)*0.5f*theta*c->forward_ppm/1000.0f;
    const float targets[4] = {-f+l+r, -f-l-r, -f+l-r, -f-l+r};
    uint32_t largest = 0U;
    for (unsigned i = 0; i < 4U; ++i) {
        pulses[i] = (int32_t)lroundf(targets[i]);
        if (pulses[i] == 0 && fabsf(targets[i]) > 0.0001f) pulses[i] = targets[i] > 0 ? 1 : -1;
        uint32_t distance = (uint32_t)(pulses[i] < 0 ? -pulses[i] : pulses[i]);
        if (distance > largest) largest = distance;
    }
    if (largest == 0U) return false;
    uint32_t rpm = fabsf(ex) <= 20.0f*scale && fabsf(ey) <= 20.0f*scale && slope >= -50 && slope <= 50 ?
        c->settings.fine_rpm : c->settings.coarse_rpm;
    for (unsigned i = 0; i < 4U; ++i) {
        uint32_t distance = (uint32_t)(pulses[i] < 0 ? -pulses[i] : pulses[i]);
        speeds[i] = (uint16_t)(((uint64_t)rpm*distance + largest/2U)/largest);
        if (speeds[i] == 0U) speeds[i] = 1U;
    }
    return true;
}
#endif
