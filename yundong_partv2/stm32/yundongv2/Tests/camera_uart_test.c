/* 上电直接测试 STM32 PA0 TX -> 相机 B1 RX，115200 8N1。
 * 开机请求扫码，收到结果后确认并等待，不等待 App、IMU、机械臂或电机。
 * 只在 CameraUartTest 预设中编译；普通 Debug/Release 不启用。 */
#include "stm32h7xx_hal.h"
#include "camera_link.h"

/* count为已收到的扫码结果数（0或1），status为初始化/请求结果。
 * 复用比赛链路的编号、重发和ACK逻辑，测试完成后不自动开始下一次扫码。 */
volatile uint32_t camera_uart_test_count;
volatile HAL_StatusTypeDef camera_uart_test_status;
volatile uint32_t camera_uart_test_clock_hz;

void CameraUartTest_Run(UART_HandleTypeDef *uart)
{
    /* UART测试使用内部HSI，排除板载晶振与HSE_VALUE不一致导致的乱码。
     * USART2/3/UART4/5共用此时钟选择，本测试不使用其他串口。
     * 普通比赛固件不会执行这里，不改变其时钟配置。 */
    RCC_OscInitTypeDef osc={0};
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSI;
    osc.HSIState=RCC_HSI_DIV1;
    osc.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
    camera_uart_test_status=HAL_RCC_OscConfig(&osc);
    if(camera_uart_test_status!=HAL_OK) for(;;) {}
    RCC_PeriphCLKInitTypeDef clocks={0};
    clocks.PeriphClockSelection=RCC_PERIPHCLK_UART4;
    clocks.Usart234578ClockSelection=RCC_USART234578CLKSOURCE_HSI;
    camera_uart_test_status=HAL_RCCEx_PeriphCLKConfig(&clocks);
    if(camera_uart_test_status!=HAL_OK) for(;;) {}
    /* 句柄已初始化，直接重新计算BRR；不DeInit，避免MSP恢复旧时钟源。 */
    uart->Init.BaudRate=115200U;
    camera_uart_test_status=HAL_UART_Init(uart);
    if(camera_uart_test_status!=HAL_OK) for(;;) {}
    camera_uart_test_clock_hz=HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_UART4);
    CameraLink_Init(uart,CameraLink_Process);
    TaskVision result;
    if(CameraLink_Wait(TASK_QR,0,&result)) ++camera_uart_test_count;
    else camera_uart_test_status=HAL_ERROR;
    for(;;) {
        CameraLink_Process(); /* ACK丢失时继续确认重复结果。 */
        HAL_Delay(1U);
    }
}
