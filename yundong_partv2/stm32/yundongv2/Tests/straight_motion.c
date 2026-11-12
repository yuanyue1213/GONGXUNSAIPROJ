/* 仅ForwardTest：四轮位置模式直行，驱动器自行加减速，预计时间后停车。 */
#include "motion_control.h"
#include "robot_config.h"
#include "zdt_motor.h"
#include <math.h>

bool Motion_MoveStraight(char direction,float mm,uint16_t rpm)
{
    if((direction!='F'&&direction!='B')||!isfinite(mm)||mm<=0||!rpm||rpm>STEPPER_MAX_SPEED_RPM) return false;
    double pulses=round((double)mm*FORWARD_TEST_PPM/1000.0);
    if(pulses<1||pulses>INT32_MAX) return false;
    double revolutions=pulses/MISSION_WHEEL_PPR;
    double acceleration=20000.0/(256.0-FORWARD_TEST_ACCEL);
    double peak=FORWARD_TEST_ACCEL?fmin(rpm,sqrt(60.0*revolutions*acceleration)):rpm;
    double seconds=revolutions*60.0/peak+(FORWARD_TEST_ACCEL?peak/acceleration:0);
    double estimate=ceil(seconds*1000.0)+MISSION_MOTION_SETTLE_MS;
    if(estimate>UINT32_MAX) return false;
    if(ZDT_Motor_StopWheelsImmediate()!=HAL_OK) goto failed;
    for(uint8_t id=1;id<=4;++id)
        if(ZDT_Motor_Enable(id)!=HAL_OK) goto failed;
    int32_t sign=direction=='F'?1:-1,wheel[4];
    for(unsigned i=0;i<4;++i) wheel[i]=sign*(int32_t)pulses;
    if(ZDT_Motor_MoveWheelSynchronous(wheel,rpm,FORWARD_TEST_ACCEL)!=HAL_OK) goto failed;
    Motion_Wait((uint32_t)estimate);
    if(ZDT_Motor_StopWheelsImmediate()==HAL_OK) return true;
failed:
    Motion_Stop();
    return false;
}
