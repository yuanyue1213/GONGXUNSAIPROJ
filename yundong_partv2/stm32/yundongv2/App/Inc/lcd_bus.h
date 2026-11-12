#ifndef LCD_BUS_H
#define LCD_BUS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void LcdBus_Init(void);
void LcdBus_Write(bool data,const uint8_t *bytes,size_t count);
void LcdBus_Delay(uint32_t ms);
void LcdBus_Backlight(void);
#endif
