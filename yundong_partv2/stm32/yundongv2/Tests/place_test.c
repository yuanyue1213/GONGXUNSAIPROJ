/* 独立放置固件：上电准备、收臂，循环从0度车载槽位取料，放到粗加工中间位。
 * 复用arm_flow.h的ARM_PUT_STEPS，不等待App、相机、雷达或IMU。
 * 启动前伸缩和升降在约定零位；每轮需向槽位补料，并清空目标放置位置。 */
#include "main.h"
#include "arm_flow.h"
#include "lift_motor.h"
#include "motion_control.h"
#include "servo_control.h"

_Noreturn void PlaceTest_Run(UART_HandleTypeDef *uart,TIM_HandleTypeDef *tim1,TIM_HandleTypeDef *tim3)
{
    LiftMotor_Init(uart);
    Motion_Init(NULL);
    if(!ServoControl_Init(tim1,tim3)) Error_Handler();
    const ArmStep prepare[]={ARM_PREPARE_STEPS,ARM_WAIT(ARM_PLACE_TEST_STARTUP_MS)};
    const ArmStep place[]={ARM_PUT_STEPS(ARM_AREA_PROCESS,ARM_CENTER)};
    if(ArmFlow_Run(prepare,ARM_COUNT(prepare))&&ArmFlow_Stow()) {
        for(;;) {
            if(!ArmFlow_Run(place,ARM_COUNT(place))) break;
            Motion_Wait(ARM_PLACE_TEST_INTERVAL_MS);
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
