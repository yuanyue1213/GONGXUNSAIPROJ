/* 独立底座测试：上电输出0度并保持，夹爪及转盘PWM不启动。 */
#include "main.h"
#include "servo_control.h"
#include "robot_config.h"

_Noreturn void BaseTest_Run(TIM_HandleTypeDef *tim1,TIM_HandleTypeDef *tim3)
{
    if(!ServoControl_Init(tim1,tim3)||!ServoControl_SetAngle('B',BASE_TEST_ANGLE))
        Error_Handler();
    /* PC6/TIM3_CH1持续输出PWM，无需App或串口应答。 */
    for(;;) HAL_Delay(1000U);
}
