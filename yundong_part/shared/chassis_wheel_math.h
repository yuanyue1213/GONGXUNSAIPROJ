#ifndef CHASSIS_WHEEL_MATH_H
#define CHASSIS_WHEEL_MATH_H

/* Original wheel mixer: slots are left-front, right-front, right-rear,
 * left-rear. The temporary strafe/yaw exchange was reverted after the user
 * identified reversed wheel installation. Motor IDs/polarities are unchanged. */
static inline void RobotChassis_WheelTargets(double forward, double left,
                                             double left_yaw, double out[4])
{
    out[0] = -forward + left + left_yaw;
    out[1] = -forward - left - left_yaw;
    out[2] = -forward + left - left_yaw;
    out[3] = -forward - left + left_yaw;
}
#endif
