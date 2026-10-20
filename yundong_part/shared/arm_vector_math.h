#ifndef ARM_VECTOR_MATH_H
#define ARM_VECTOR_MATH_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* RPM is a vector in motor-revolution space, so axes share nominal travel time
 * despite their different mm/rev. Zero-distance axes receive no motor command. */
static inline bool ArmVector_Speeds(int64_t delta_r, int64_t delta_z, uint32_t combined,
                                    uint16_t *r_rpm, uint16_t *z_rpm)
{
    if (combined < 5U || combined > 600U) return false;
    double r = fabs((double)delta_r), z = fabs((double)delta_z), length = hypot(r, z);
    *r_rpm = *z_rpm = 0U;
    if (length == 0.0) return true;
    if (r != 0.0) { *r_rpm = (uint16_t)lround(combined * r / length); if (*r_rpm == 0U) *r_rpm = 1U; }
    if (z != 0.0) { *z_rpm = (uint16_t)lround(combined * z / length); if (*z_rpm == 0U) *z_rpm = 1U; }
    return true;
}
/* Positive lift motor displacement means downward chassis Z travel.
 * Reserve at most 900 tenths of a motor degree = 10 mm at 40 mm/revolution. */
static inline int64_t ArmVector_DescentStage(int64_t current_z_motor, int64_t final_z_motor)
{
    if (final_z_motor <= current_z_motor) return final_z_motor;
    return final_z_motor - current_z_motor <= 900 ? current_z_motor : final_z_motor - 900;
}
#endif
