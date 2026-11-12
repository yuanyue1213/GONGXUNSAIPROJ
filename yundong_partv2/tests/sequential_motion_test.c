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
static float turn_yaws[5],turn_target,turn_sign,feedback_rate;
static bool turn_feedback,settled_feedback;
static bool fine_feedback;
static unsigned turn_sample,positive_speeds,negative_speeds;
static unsigned high_speeds,low_speeds,braking_speeds,limited_speeds,speed_calls;
static uint16_t turn_limit=MISSION_TURN_RPM;
static int16_t yaw_speed;
static int32_t last_wheel[4];
static uint8_t expected_accel=MISSION_WHEEL_ACCEL;
static bool fail_arm;
static char firmware_codes[2]={'X','X'};
static bool inject_gap,recovered,fail_stop;
static unsigned lost_reads;
static unsigned index_of(char c) { return c=='G'?0:c=='T'?1:2; }
uint32_t HAL_GetTick(void) { return tick++; }
static void service(void) { ++services; tick+=10; }
/* 脚本化IMU反馈：覆盖过冲、角速度制动、稳定窗口和失联停车。 */
static void set_turn_feedback(float target)
{
    turn_target=target;
    float sign=target>yaw?1.0f:-1.0f;
    turn_sign=sign;
    turn_yaws[0]=(yaw+target)*.5f;
    turn_yaws[1]=target+sign*2.0f;
    turn_yaws[2]=turn_yaws[3]=target-sign*.5f;
    turn_yaws[4]=target+sign*.5f;
    turn_sample=0; turn_feedback=true; settled_feedback=false;
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
    if(fine_feedback) {
        yaw-=yaw_speed*.02f; /* 小角度反馈随实际非零转速推进，零指令不能到位。 */
    } else if(turn_feedback) {
        yaw=turn_yaws[turn_sample];
        if(turn_sample==3U) rate=turn_sign*10; /* 尚未到目标但角速度偏大，PD应反向制动。 */
        if(turn_sample<4U) ++turn_sample;
    }
    feedback_rate=rate;
    settled_feedback=fabsf(turn_target-yaw)<=(fine_feedback?PARALLEL_TURN_TOLERANCE_DEG:TURN_ANGLE_TOLERANCE)&&fabsf(rate)<=TURN_RATE_TOLERANCE;
    *s=(ImuSample){.yaw=yaw,.rate=rate}; return true;
}
HAL_StatusTypeDef ZDT_Motor_Enable(uint8_t id) { assert(id>=1&&id<=4); return HAL_OK; }
HAL_StatusTypeDef ZDT_Motor_MoveWheelSynchronous(const int32_t p[4],uint16_t rpm,uint8_t a)
{
    assert(rpm<=1500&&a==expected_accel);
    if(!wheel_calls) for(unsigned i=0;i<4;++i) assert(p[i]==13064);
    memcpy(last_wheel,p,sizeof(last_wheel));
    ++wheel_calls; return HAL_OK;
}
HAL_StatusTypeDef ZDT_Motor_SetWheelSpeedsSynchronous(const int16_t rpm[4],uint8_t acceleration)
{
    assert(acceleration==240);
    assert(!settled_feedback);
    for(unsigned i=0;i<4;++i) assert(abs(rpm[i])<=turn_limit&&abs(rpm[i])==abs(rpm[0]));
    assert(rpm[0]==-rpm[1]&&rpm[0]==-rpm[2]&&rpm[0]==rpm[3]);
    if(speed_calls++==0U) assert(rpm[0]==-36); /* 初次右转：误差45°、角速度0 -> 36 RPM。 */
    if(abs(rpm[0])>10) ++high_speeds; else ++low_speeds;
    if(abs(rpm[0])==turn_limit) ++limited_speeds;
    if(feedback_rate!=0) {
        assert(rpm[0]*feedback_rate>0&&rpm[0]*(turn_target-yaw)>0);
        ++braking_speeds;
    }
    if(rpm[0]>0) ++positive_speeds; else ++negative_speeds;
    yaw_speed=rpm[0]; return HAL_OK;
}
HAL_StatusTypeDef ZDT_Motor_Stop(uint8_t id)
{ yaw_speed=0; ++stop_calls; return fail_stop&&id==1?HAL_ERROR:HAL_OK; }
HAL_StatusTypeDef ArmMotor_MoveAngle(char d,uint32_t angle,uint16_t rpm,uint32_t ppr)
{
    assert(d=='E'||d=='C'); assert(angle&&rpm<=STEPPER_MAX_SPEED_RPM&&ppr==3200);
    arm_at=tick; ++arm_calls; return fail_arm?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef LiftMotor_MoveAngle(char d,uint32_t angle,uint16_t rpm,uint32_t ppr)
{
    assert(d=='U'||d=='D'); assert(angle&&rpm<=STEPPER_MAX_SPEED_RPM&&ppr==3200);
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
    /* 路线300RPM、20档，1米仍是三角形曲线。 */
    uint32_t wheel_expected=(uint32_t)ceil(2.0*sqrt((13064.0/3200.0)*60.0*236.0/20000.0)*1000.0)+MISSION_MOTION_SETTLE_MS;
    assert(wheel_ms>=wheel_expected&&wheel_ms<wheel_expected+30U);
    /* 校准保留200RPM、30档，等待时间必须与本次下发档位一致。 */
    wheel_start=tick;
    expected_accel=MISSION_ALIGN_ACCEL;
    assert(Motion_MoveAt('L',100,MISSION_FINE_RPM,MISSION_ALIGN_ACCEL));
    wheel_ms=tick-wheel_start;
    wheel_expected=(uint32_t)ceil(2.0*sqrt((1323.0/3200.0)*60.0*226.0/20000.0)*1000.0)+MISSION_MOTION_SETTLE_MS;
    assert(wheel_ms>=wheel_expected&&wheel_ms<wheel_expected+30U);
    expected_accel=MISSION_WHEEL_ACCEL;
    uint32_t before=tick;
    assert(Motion_TurnPulses(-3,5));
    assert(last_wheel[0]==-3&&last_wheel[1]==3&&last_wheel[2]==3&&last_wheel[3]==-3);
    assert(Motion_TurnPulses(3,5));
    assert(last_wheel[0]==3&&last_wheel[1]==-3&&last_wheel[2]==-3&&last_wheel[3]==3);
    assert(!Motion_TurnPulses(0,5)&&!Motion_TurnPulses(INT32_MIN,5)&&!Motion_TurnPulses(3,STEPPER_MAX_SPEED_RPM+1U));
    before=tick;
    assert(Motion_MoveAll(1270,40,200,300)); /* both axes = one motor revolution */
    uint32_t duration=tick-before;
    assert(arm_calls==1&&lift_calls==1&&lift_at-arm_at<5&&base_at-arm_at<5);
    uint32_t estimate=Motion_EstimateMs(1,300,400);
    assert(duration>=estimate&&duration<estimate+30); /* max, not sum */
    unsigned old_arm=arm_calls,old_lift=lift_calls;
    assert(Motion_MoveAll(1270,40,200,300)&&arm_calls==old_arm&&lift_calls==old_lift);
    /* Emm两轴均一圈：档位150约188.68 RPM/s，估时同步使用下发加速度。 */
    firmware_codes[0]=firmware_codes[1]='E';
    before=tick;
    assert(Motion_MoveAll(2540,80,200,MOTION_ARM_RPM));
    duration=tick-before;
    estimate=(uint32_t)ceil(2.0*sqrt(60.0*106.0/20000.0)*1000.0)+MISSION_MOTION_SETTLE_MS;
    assert(duration>=estimate&&duration<estimate+30U);
    /* 混合固件取两轴最大时间；Emm约188.68RPM/s比X的400RPM/s更慢。 */
    firmware_codes[0]='X';
    before=tick;
    assert(Motion_MoveAll(1270,40,200,MOTION_ARM_RPM));
    duration=tick-before;
    uint32_t x_estimate=Motion_EstimateMs(1,MOTION_ARM_RPM,ARM_X_ACCEL_RPM_S);
    if(x_estimate>estimate) estimate=x_estimate;
    assert(duration>=estimate&&duration<estimate+30U);
    assert(Motion_Reference());
    set_turn_feedback(90);
    assert(Motion_Turn('R',90,MISSION_TURN_RPM)&&fabsf(yaw-90)<=TURN_ANGLE_TOLERANCE);
    assert(positive_speeds&&negative_speeds&&high_speeds&&low_speeds&&braking_speeds);
    set_turn_feedback(0);
    assert(Motion_Turn('L',90,MISSION_TURN_RPM)&&fabsf(yaw)<=TURN_ANGLE_TOLERANCE);
    unsigned high_before=high_speeds,low_before=low_speeds;
    set_turn_feedback(5);
    assert(Motion_Turn('R',5,MISSION_TURN_RPM)&&fabsf(yaw-5)<=TURN_ANGLE_TOLERANCE);
    set_turn_feedback(0);
    assert(Motion_Turn('L',5,MISSION_TURN_RPM)&&fabsf(yaw)<=TURN_ANGLE_TOLERANCE);
    assert(high_speeds==high_before&&low_speeds>low_before); /* 小角度全程低速。 */
    turn_limit=10;
    set_turn_feedback(90);
    assert(Motion_Turn('R',90,turn_limit)&&limited_speeds);
    turn_limit=MISSION_TURN_RPM;
    set_turn_feedback(0);
    assert(Motion_Turn('L',90,turn_limit));
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
    fine_feedback=true; turn_feedback=false; yaw_speed=0; turn_limit=PARALLEL_FINE_RPM;
    before=tick; unsigned low_start=low_speeds;
    turn_target=yaw+.2f;
    assert(Motion_TurnFine(.2f,turn_limit)&&fabsf(yaw-turn_target)<=.05f&&low_speeds>low_start);
    assert(tick-before>=TURN_STABLE_MS);
    turn_target=yaw-.3f;
    assert(Motion_TurnFine(-.3f,turn_limit)&&fabsf(yaw-turn_target)<=.05f);
    assert(!Motion_TurnFine(0,5)&&!Motion_TurnFine(NAN,5)&&!Motion_TurnFine(PARALLEL_MAX_ANGLE_DEG+1,5));
    assert(!Motion_TurnFine(1,0)&&!Motion_TurnFine(1,STEPPER_MAX_SPEED_RPM+1U));
    /* 初始30°，第一次停在120.5°，第二次目标仍是210°，不是210.5°。 */
    fine_feedback=turn_feedback=false; yaw_speed=0; yaw=30; turn_limit=MISSION_TURN_RPM;
    assert(Motion_Reference());
    set_turn_feedback(120);
    assert(Motion_Turn('R',90,turn_limit)&&fabsf(yaw-120.5f)<.001f);
    set_turn_feedback(210);
    assert(Motion_Turn('R',90,turn_limit)&&fabsf(yaw-210.5f)<.001f);
    /* 工位视觉微调独立执行，不累加到后续路线转角。 */
    fine_feedback=true; turn_feedback=false; turn_limit=PARALLEL_FINE_RPM;
    turn_target=yaw+.2f;
    assert(Motion_TurnFine(.2f,turn_limit)&&fabsf(yaw-turn_target)<=.05f);
    fine_feedback=false; turn_limit=MISSION_TURN_RPM;
    /* 恢复正负累计角度，可超过单次视觉修正15度，且不改路线目标。 */
    fine_feedback=true; turn_limit=PARALLEL_RPM;
    turn_target=yaw+40;
    assert(Motion_RestoreYaw(turn_target,turn_limit)&&fabsf(yaw-turn_target)<=.05f);
    turn_target=yaw-40;
    assert(Motion_RestoreYaw(turn_target,turn_limit)&&fabsf(yaw-turn_target)<=.05f);
    assert(!Motion_RestoreYaw(NAN,10)&&!Motion_RestoreYaw(0,0)&&!Motion_RestoreYaw(0,STEPPER_MAX_SPEED_RPM+1U));
    fine_feedback=false; turn_limit=MISSION_TURN_RPM;
    set_turn_feedback(120);
    assert(Motion_Turn('L',90,turn_limit)&&fabsf(yaw-119.5f)<.001f);
    assert(!Motion_Move('F',100,STEPPER_MAX_SPEED_RPM+1U)&&!Motion_MoveAll(0,0,0,STEPPER_MAX_SPEED_RPM+1U));
    fail_arm=true; assert(!Motion_MoveAll(0,0,248,300));
    Motion_Stop(); assert(stop_calls>=4);
    tick=UINT32_MAX-10; Motion_Wait(30); assert(tick<100);
    puts("PASS: sequential motion, parallel waits, PD turns, cumulative route targets without stop-error or visual-offset accumulation, decreasing speed, rate braking, RPM cap, IMU recovery and timer wrap");
}
