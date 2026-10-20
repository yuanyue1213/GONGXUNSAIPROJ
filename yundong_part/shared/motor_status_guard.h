#ifndef MOTOR_STATUS_GUARD_H
#define MOTOR_STATUS_GUARD_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { bool pending; uint32_t since; } MotorStatusGuard;
/* X42S manual 5.5.15: Cgi (bit2) has no persistence test, whereas Cgp
 * (bit3) is confirmed driver protection. Keep hardware protection immediate;
 * confirm Cgi for 400 ms in software rather than stopping on a startup sample. */
static inline bool MotorStatusGuard_Fault(MotorStatusGuard *guard, uint8_t flags, uint32_t now)
{
    if ((flags & 0x01U) == 0U || (flags & 0x08U) != 0U) return true;
    if ((flags & 0x04U) == 0U || (flags & 0x02U) != 0U) {
        guard->pending = false;
        return false;
    }
    if (!guard->pending) { guard->pending = true; guard->since = now; }
    return (uint32_t)(now - guard->since) >= 400U;
}
#endif
