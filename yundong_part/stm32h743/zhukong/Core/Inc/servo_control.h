#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* G: PC8 clamp, T: PA8 turntable, B: PC6 base. */
void ServoControl_Init(void);
bool ServoControl_SetAngle(char channel, uint16_t angle);

#endif
