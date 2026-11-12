/* 独立转盘测试：上电输出0度并保持，其他舵机通道不启动。 */
#include "main.h"
#include "servo_control.h"
#include "robot_config.h"

_Noreturn void TurntableTest_Run(TIM_HandleTypeDef *tim1,TIM_HandleTypeDef *tim3)
{
    if(!ServoControl_Init(tim1,tim3)||!ServoControl_SetAngle('T',TURNTABLE_TEST_ANGLE))
        Error_Handler();
    /* PA8/TIM1_CH1持续输出PWM，无需App或任何串口应答。 */
    for(;;) HAL_Delay(1000U);
}
