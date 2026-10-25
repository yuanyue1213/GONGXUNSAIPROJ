#ifndef SERVO_MOTION_H
#define SERVO_MOTION_H
#include <stdint.h>
#include <math.h>
/* Same 6t^5-15t^4+10t^3 easing used by the reference PWM.c. */
static inline uint16_t ServoMotion_Angle(int32_t start, int32_t target, uint32_t elapsed, uint32_t duration)
{
    if(duration==0U||elapsed>=duration) return (uint16_t)target;
    float t=(float)elapsed/(float)duration;
    float s=t*t*t*(10.0f+t*(-15.0f+6.0f*t));
    return (uint16_t)lroundf(start+(target-start)*s);
}
#endif
