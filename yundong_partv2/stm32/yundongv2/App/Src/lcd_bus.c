/* ILI9341软件SPI，Mode 0、MSB先发，只在任务码/进度变化时刷屏。
 * 第8脚主板标LCD_RST，实物连接BLK，高电平点亮；SDO不用。 */
#include "lcd_bus.h"
#include "robot_config.h"
#include "stm32h7xx_hal.h"
static void pause_clock(void)
{
    for(volatile unsigned i=0;i<LCD_SPI_DELAY_LOOPS;++i) __NOP();
}
void LcdBus_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef gpio={0};
    gpio.Mode=GPIO_MODE_OUTPUT_PP; gpio.Pull=GPIO_NOPULL; gpio.Speed=GPIO_SPEED_FREQ_HIGH;
    LCD_CS_PORT->BSRR=LCD_CS_PIN;
    LCD_BLK_PORT->BSRR=(uint32_t)LCD_BLK_PIN<<16;
    LCD_SCK_PORT->BSRR=(uint32_t)LCD_SCK_PIN<<16;
    LCD_MOSI_PORT->BSRR=(uint32_t)LCD_MOSI_PIN<<16;
    gpio.Pin=LCD_MOSI_PIN; HAL_GPIO_Init(LCD_MOSI_PORT,&gpio);
    gpio.Pin=LCD_SCK_PIN; HAL_GPIO_Init(LCD_SCK_PORT,&gpio);
    gpio.Pin=LCD_DC_PIN; HAL_GPIO_Init(LCD_DC_PORT,&gpio);
    gpio.Pin=LCD_CS_PIN|LCD_BLK_PIN; HAL_GPIO_Init(LCD_CS_PORT,&gpio);
}
void LcdBus_Write(bool data,const uint8_t *bytes,size_t count)
{
    LCD_DC_PORT->BSRR=data?LCD_DC_PIN:(uint32_t)LCD_DC_PIN<<16;
    LCD_CS_PORT->BSRR=(uint32_t)LCD_CS_PIN<<16;
    for(size_t i=0;i<count;++i) {
        uint8_t value=bytes[i];
        for(unsigned bit=0;bit<8;++bit) {
            LCD_MOSI_PORT->BSRR=(value&0x80U)?LCD_MOSI_PIN:(uint32_t)LCD_MOSI_PIN<<16;
            pause_clock(); LCD_SCK_PORT->BSRR=LCD_SCK_PIN;
            pause_clock(); LCD_SCK_PORT->BSRR=(uint32_t)LCD_SCK_PIN<<16;
            value<<=1;
        }
    }
    LCD_CS_PORT->BSRR=LCD_CS_PIN;
}
void LcdBus_Delay(uint32_t ms) { HAL_Delay(ms); }
void LcdBus_Backlight(void) { LCD_BLK_PORT->BSRR=LCD_BLK_PIN; }
