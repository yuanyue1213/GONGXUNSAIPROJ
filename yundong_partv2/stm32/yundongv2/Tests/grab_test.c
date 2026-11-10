/* 独立抓取固件：上电准备、收臂，然后循环从原料中间位抓一件放入0度槽位。
 * 复用arm_flow.h动作，不等待App、相机、雷达或IMU；每轮结束后等待再继续。
 * 启动前伸缩和升降在约定零位；每轮需取走槽位物料并补充抓取位置物料。 */
#include "main.h"
#include "arm_flow.h"
#include "lift_motor.h"
#include "motion_control.h"
#include "servo_control.h"

_Noreturn void GrabTest_Run(UART_HandleTypeDef *uart,TIM_HandleTypeDef *tim1,TIM_HandleTypeDef *tim3)
{
    LiftMotor_Init(uart);
    Motion_Init(NULL);
    if(!ServoControl_Init(tim1,tim3)) Error_Handler();
    const ArmStep prepare[]={ARM_PREPARE_STEPS,ARM_WAIT(ARM_GRAB_TEST_STARTUP_MS)};
    const ArmStep grab[]={
        ARM_PICK_APPROACH(ARM_AREA_RAW,ARM_CENTER),
        ARM_PICK_FINISH(ARM_AREA_RAW,ARM_CENTER),
    };
    if(ArmFlow_Run(prepare,ARM_COUNT(prepare))&&ArmFlow_Stow()) {
        for(;;) {
            if(!ArmFlow_Run(grab,ARM_COUNT(grab))) break;
            Motion_Wait(ARM_GRAB_TEST_INTERVAL_MS);
        }
    }
    (void)LiftMotor_Stop();
    (void)ArmMotor_StopForeAft();
    ServoControl_Cancel('G');
    ServoControl_Cancel('T');
    ServoControl_Cancel('B');
    Error_Handler();
    for(;;) HAL_Delay(1000U);
}
