/* 上电只显示固定任务码，进度均未完成；不启动App、视觉、雷达或电机。 */
#include "task_display.h"
#include "robot_config.h"
#include "stm32h7xx_hal.h"
_Noreturn void ScreenTest_Run(void)
{
    TaskCode code;
    TaskDisplay_Init();
    if(TaskProtocol_Code(MISSION_FIXED_QR_TEXT,&code)) TaskDisplay_Task(&code);
    else TaskDisplay_State(DISPLAY_ERROR);
    for(;;) HAL_Delay(100);
}
