#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motion_control.h"
#include "mission_config.h"
#include "servo_control.h"
#include "imu_link.h"
#include "zdt_motor.h"
#include "lift_motor.h"

static uint32_t tick,services,arm_at,lift_at,base_at;
static unsigned arm_calls,lift_calls,wheel_calls,stop_calls;
static uint16_t angles[3]={15,0,248};
static float yaw;
static float turn_yaws[5],turn_target;
static bool turn_feedback,angle_window_feedback;
static unsigned turn_sample,positive_speeds,negative_speeds;
static unsigned high_speeds,low_speeds;
static int16_t yaw_speed;
static bool fail_arm;
static char firmware_codes[2]={'X','X'};
static bool inject_gap,recovered,fail_stop;
static unsigned lost_reads;
static unsigned index_of(char c) { return c=='G'?0:c=='T'?1:2; }
uint32_t HAL_GetTick(void) { return tick++; }
static void service(void) { ++services; tick+=10; }
/* 脚本化IMU反馈验证指令，不用理想运动模型假定实机能达到0.01度。 */
static void set_turn_feedback(float target)
{
    turn_target=target;
    float sign=target>yaw?1.0f:-1.0f;
    turn_yaws[0]=(yaw+target)*.5f;
    turn_yaws[1]=target+sign*.02f;
    turn_yaws[2]=target-sign*.02f;
    turn_yaws[3]=turn_yaws[4]=target+sign*.005f;
    turn_sample=0; turn_feedback=true; angle_window_feedback=false;
}
bool ImuLink_Read(ImuSample *s)
{
    if(inject_gap&&yaw_speed) { inject_gap=false; lost_reads=3; return false; }
    if(lost_reads) {
        assert(yaw_speed==0); /* 等待恢复期间四轮必须已经收到停车指令。 */
        if(!--lost_reads) recovered=true;
        return false;
    }
    float rate=0;
    if(turn_feedback) {
        angle_window_feedback=turn_sample>=3U;
        yaw=turn_yaws[turn_sample];
        if(turn_sample==3U) rate=10; /* 已进角度窗口但还在转：必须先停车。 */
        if(turn_sample<4U) ++turn_sample;
    }
    *s=(ImuSample){.yaw=yaw,.rate=rate}; return true;
}
HAL_StatusTypeDef ZDT_Motor_Enable(uint8_t id) { assert(id>=1&&id<=4); return HAL_OK; }
HAL_StatusTypeDef ZDT_Motor_MoveWheelSynchronous(const int32_t p[4],uint16_t rpm,uint8_t a)
{
    assert(rpm<=1500&&a==MISSION_WHEEL_ACCEL);
    if(!wheel_calls) for(unsigned i=0;i<4;++i) assert(p[i]==12585);
    ++wheel_calls; return HAL_OK;
}
HAL_StatusTypeDef ZDT_Motor_SetWheelSpeedsSynchronous(const int16_t rpm[4],uint8_t acceleration)
{
    assert(acceleration==240);
    assert(!angle_window_feedback); /* 误差达标但角速度未稳时也不得继续驱动。 */
    unsigned expected=fabsf(turn_target-yaw)<=TURN_SLOW_ANGLE?TURN_SLOW_RPM:MISSION_TURN_RPM;
    for(unsigned i=0;i<4;++i) assert(abs(rpm[i])==(int)expected);
    if(expected==TURN_SLOW_RPM) ++low_speeds; else ++high_speeds;
    if(rpm[0]>0) ++positive_speeds; else ++negative_speeds;
    yaw_speed=rpm[0]; return HAL_OK;
}
HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id)
{ yaw_speed=0; ++stop_calls; return fail_stop&&id==1?HAL_ERROR:HAL_OK; }
HAL_StatusTypeDef ArmMotor_MoveAngle(char d,uint32_t angle,uint16_t rpm,uint32_t ppr)
{
    assert(d=='E'||d=='C'); assert(angle&&rpm<=1500&&ppr==3200);
    arm_at=tick; ++arm_calls; return fail_arm?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef LiftMotor_MoveAngle(char d,uint32_t angle,uint16_t rpm,uint32_t ppr)
{
    assert(d=='U'||d=='D'); assert(angle&&rpm<=1500&&ppr==3200);
    lift_at=tick; ++lift_calls; return HAL_OK;
}
char LiftMotor_FirmwareCode(uint8_t id) { assert(id==1||id==2); return firmware_codes[id-1U]; }
HAL_StatusTypeDef LiftMotor_Stop(void) { return HAL_OK; }
HAL_StatusTypeDef ArmMotor_StopForeAft(void) { return HAL_OK; }
bool ServoControl_GetAngle(char c,uint16_t *v) { *v=angles[index_of(c)]; return true; }
bool ServoControl_MoveTo(char c,uint16_t v,uint32_t ms)
{ (void)ms; if(c=='B') base_at=tick; angles[index_of(c)]=v; return true; }
void ServoControl_Cancel(char c) { (void)c; }
int main(void)
{
    Motion_Init(service);
    assert(Motion_EstimateMs(0,100,200)==0);
    assert(Motion_EstimateMs(1,60,60)==2000+MISSION_MOTION_SETTLE_MS); /* 1 s acceleration + 1 s deceleration */
    assert(Motion_EstimateMs(2,60,60)==3000+MISSION_MOTION_SETTLE_MS);
    uint32_t wheel_start=tick;
    assert(Motion_Move('F',1000,MISSION_ROUTE_RPM)&&wheel_calls==1&&services>0);
    uint32_t wheel_ms=tick-wheel_start;
    /* 12585/3200圈，档位10=81.3RPM/s，耗时约3.41秒+余量。
     * 旧代码误用10RPM/s，会等约9.72秒。 */
    uint32_t wheel_expected=(uint32_t)ceil(2.0*sqrt((12585.0/3200.0)*60.0*246.0/20000.0)*1000.0)+MISSION_MOTION_SETTLE_MS;
    assert(wheel_ms>=wheel_expected&&wheel_ms<wheel_expected+30U);
    /* 短距离同样使用正确加速度，不能套用错误的旧等待时间。 */
    wheel_start=tick;
    assert(Motion_Move('L',100,MISSION_FINE_RPM));
    wheel_ms=tick-wheel_start;
    wheel_expected=(uint32_t)ceil(2.0*sqrt((1323.0/3200.0)*60.0*246.0/20000.0)*1000.0)+MISSION_MOTION_SETTLE_MS;
    assert(wheel_ms>=wheel_expected&&wheel_ms<wheel_expected+30U);
    uint32_t before=tick;
    assert(Motion_MoveAll(1270,40,200,300)); /* both axes = one motor revolution */
    uint32_t duration=tick-before;
    assert(arm_calls==1&&lift_calls==1&&lift_at-arm_at<5&&base_at-arm_at<5);
    uint32_t estimate=Motion_EstimateMs(1,300,200);
    assert(duration>=estimate&&duration<estimate+30); /* max, not sum */
    unsigned old_arm=arm_calls,old_lift=lift_calls;
    assert(Motion_MoveAll(1270,40,200,300)&&arm_calls==old_arm&&lift_calls==old_lift);
    /* Emm两轴均一圈：档位10应等约1.77秒，而非误用10RPM/s的4.95秒。 */
    firmware_codes[0]=firmware_codes[1]='E';
    before=tick;
    assert(Motion_MoveAll(2540,80,200,1500));
    duration=tick-before;
    estimate=(uint32_t)ceil(2.0*sqrt(60.0*246.0/20000.0)*1000.0)+MISSION_MOTION_SETTLE_MS;
    assert(duration>=estimate&&duration<estimate+30U);
    /* 混合固件取两轴最大时间；当前Emm档位10比X的200RPM/s更慢。 */
    firmware_codes[0]='X';
    before=tick;
    assert(Motion_MoveAll(1270,40,200,1500));
    duration=tick-before;
    uint32_t x_estimate=Motion_EstimateMs(1,1500,ARM_X_ACCEL_RPM_S);
    if(x_estimate>estimate) estimate=x_estimate;
    assert(duration>=estimate&&duration<estimate+30U);
    assert(Motion_Reference());
    set_turn_feedback(90);
    assert(Motion_Turn('R',90,MISSION_TURN_RPM)&&fabsf(yaw-90)<=TURN_ANGLE_TOLERANCE);
    assert(positive_speeds&&negative_speeds&&high_speeds&&low_speeds);
    set_turn_feedback(0);
    assert(Motion_Turn('L',90,MISSION_TURN_RPM)&&fabsf(yaw)<=TURN_ANGLE_TOLERANCE);
    unsigned high_before=high_speeds,low_before=low_speeds;
    set_turn_feedback(5);
    assert(Motion_Turn('R',5,MISSION_TURN_RPM)&&fabsf(yaw-5)<=TURN_ANGLE_TOLERANCE);
    set_turn_feedback(0);
    assert(Motion_Turn('L',5,MISSION_TURN_RPM)&&fabsf(yaw)<=TURN_ANGLE_TOLERANCE);
    assert(high_speeds==high_before&&low_speeds>low_before); /* 小角度全程低速。 */
    unsigned stopped=stop_calls;
    inject_gap=true;
    set_turn_feedback(90);
    assert(Motion_Turn('R',90,MISSION_TURN_RPM)&&recovered&&fabsf(yaw-90)<=TURN_ANGLE_TOLERANCE);
    assert(stop_calls>=stopped+8); /* 失联时停一次，到位后再停一次。 */
    stopped=stop_calls;
    inject_gap=true; fail_stop=true;
    set_turn_feedback(180);
    assert(!Motion_Turn('R',90,MISSION_TURN_RPM)&&stop_calls==stopped+4); /* 一轮失败仍停其余轮。 */
    lost_reads=0; fail_stop=false;
    assert(!Motion_Move('F',100,1501)&&!Motion_MoveAll(0,0,0,1501));
    fail_arm=true; assert(!Motion_MoveAll(0,0,248,300));
    Motion_Stop(); assert(stop_calls>=4);
    tick=UINT32_MAX-10; Motion_Wait(30); assert(tick<100);
    puts("PASS: sequential motion, parallel max-duration wait, logical axis positions, directions, IMU turns, speed bounds and timer wrap");
}
