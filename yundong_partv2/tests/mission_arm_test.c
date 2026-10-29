#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_flow.h"

/* 使用同一个真实执行器，换成包含自定义动作组的小路线。 */
static FlowStep test_flow[]={
    FLOW_PREPARE(), FLOW_ARM(arm_ready),
    FLOW_MOVE(FLOW_FORWARD,100), FLOW_ARM(arm_ready),
    FLOW_MOVE(FLOW_LEFT,200), FLOW_ARM(arm_ready),
    FLOW_MOVE(FLOW_BACKWARD,300), FLOW_END(),
};
#define mission_flow test_flow
#include "../stm32/yundongv2/App/Src/mission_runner.c"
#undef mission_flow

static char trace[32],last_status[128];
static unsigned used,waits,stops;
static bool fail_servo,allow_qr;
static void record(char c) { trace[used++]=c; trace[used]=0; }
void StartLink_Publish(const char *s) { snprintf(last_status,sizeof(last_status),"%s",s); }
bool ServoControl_SetAngle(char c,uint16_t a) { (void)c; (void)a; return true; }
bool Motion_Reference(void) { return true; }
bool Motion_Move(char d,float mm,uint16_t rpm)
{
    (void)d; (void)mm; (void)rpm; record('C'); return true;
}
bool Motion_MoveAll(int32_t r,int32_t z,uint16_t b,uint16_t rpm)
{
    assert(r==110&&z==0&&b==180&&rpm==MOTION_ARM_RPM);
    record('A'); return true;
}
bool Motion_Servo(char c,uint16_t a,uint16_t dps)
{
    assert(c=='G'&&a==30&&dps==MOTION_GRIPPER_DPS);
    record('G'); return !fail_servo;
}
void Motion_Wait(uint32_t ms) { if(ms==200) ++waits; else assert(ms==100); }
void Motion_Stop(void) { ++stops; }
int32_t Motion_Radial(void) { assert(false); return 0; }
bool ServoControl_GetAngle(char c,uint16_t *v) { (void)c; (void)v; assert(false); return false; }
bool Motion_Turn(char d,float deg,uint16_t rpm) { (void)d; (void)deg; (void)rpm; assert(false); return false; }
void CameraLink_Idle(void) {}
bool CameraLink_Wait(TaskVisionKind k,unsigned t,TaskVision *v)
{
    assert(allow_qr&&k==TASK_QR&&t==0);
    return TaskProtocol_Vision("VISION,1,QR,156+123+516+231",v);
}
bool CameraLink_Observe(TaskVisionKind k,unsigned t,TaskVision *v)
{ (void)k; (void)t; (void)v; assert(false); return false; }
int main(void)
{
    MissionRunner_Run(1);
    assert(!strcmp(trace,"AGCAGCAGC")&&waits==3&&stops==0);
    assert(strstr(last_status,",DONE,")&&!MissionRunner_Running());
    used=0; trace[0]=0; waits=0; fail_servo=true;
    MissionRunner_Run(2);
    assert(!strcmp(trace,"AG")&&waits==0&&stops==1);
    assert(strstr(last_status,",STOPPED,")&&!MissionRunner_Running());
    /* 流程允许任意批次字段，但站点不能因此越界访问任务码。 */
    test_flow[2]=(FlowStep)FLOW_SCAN_TASK();
    test_flow[3]=(FlowStep)FLOW_STORE((FlowBatch)3);
    test_flow[1]=(FlowStep)FLOW_IMU_ZERO();
    used=0; trace[0]=0; allow_qr=true;
    assert(!Flow_Check(test_flow,FLOW_COUNT(test_flow)));
    MissionRunner_Run(3);
    assert(used==0&&stops==2&&strstr(last_status,",STOPPED,"));
    puts("PASS: custom arm groups without IMU, reused between chassis steps, waits and failure prevents later mission steps");
}
