/* 舵机五次缓动数学函数。速度参数按峰值角速度解释，输出时间单位为毫秒。 */
#ifndef SERVO_MOTION_H
#define SERVO_MOTION_H
#include <stdint.h>
#include <math.h>
/* Quintic easing has peak derivative 1.875. Duration therefore respects peak
 * degrees/s instead of silently treating that parameter as average speed. */
static inline uint32_t ServoMotion_Duration(uint32_t distance,uint32_t peak_dps)
{
    if(!distance||!peak_dps) return 0U;
    return (uint32_t)(((uint64_t)distance*1875U+peak_dps-1U)/peak_dps);
}
/* Same 6t^5-15t^4+10t^3 easing used by the reference PWM.c. */
static inline uint16_t ServoMotion_Angle(int32_t start, int32_t target, uint32_t elapsed, uint32_t duration)
{
    if(duration==0U||elapsed>=duration) return (uint16_t)target;
    float t=(float)elapsed/(float)duration;
    float s=t*t*t*(10.0f+t*(-15.0f+6.0f*t));
    return (uint16_t)lroundf(start+(target-start)*s);
}
#endif
