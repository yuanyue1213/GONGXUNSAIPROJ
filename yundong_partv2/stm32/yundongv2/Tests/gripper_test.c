/* 独立夹爪测试固件：上电将G通道设为15度，然后保持PWM。
 * 接线：PC8/TIM3_CH3为夹爪信号。无需App、相机、IMU或电机应答。
 * 仅由GripperTest预设编译，普通Debug/Release不启用。 */
#include "main.h"
#include "servo_control.h"

_Noreturn void GripperTest_Run(TIM_HandleTypeDef *tim1,TIM_HandleTypeDef *tim3)
{
    if(!ServoControl_Init(tim1,tim3)||!ServoControl_SetAngle('G',15U))
        Error_Handler();
    /* PWM由定时器持续输出，底座和转盘通道不启动。 */
    for(;;) HAL_Delay(1000U);
}
