#include "gyro_turn.h"
#include "imu_link.h"
#include "zdt_motor.h"
#include "../../../../shared/chassis_wheel_math.h"
#include "../../../../shared/motor_status_guard.h"
#include <math.h>
#include <string.h>
/* RPM/degree and RPM/(degree/second); I=0 for the initial PD controller.
 * These conservative starting gains require tuning on the actual chassis. */
#define GYRO_KP 0.8f
#define GYRO_KD 0.18f
#define GYRO_SLEW_RPM_S 60.0f
static struct {
    GyroTurnState state;
    const char *reason;
    float target,reference,output,initial_error,progress_yaw;
    uint32_t started,updated,status_tick,stable_tick,sample_sequence,progress_tick;
    uint32_t sample_epoch,reference_epoch;
    uint16_t rpm;
    unsigned wheel;
    uint8_t stop_mask;
    bool reference_valid,braking,stable;
    bool driving,travel_done;
    int direction;
    int64_t origin[4];
    double distance_counts,progress_counts;
    float forward_output;
    MotorStatusGuard guards[4];
} g;
static bool stop_all(void)
{
    for(unsigned i=0;i<4U;++i) if((g.stop_mask&(1U<<i))==0U) {
        bool stopped=ZDT_Motor_Stop((uint8_t)(i+1U),false)==HAL_OK;
        if(!stopped) {
            /* Some firmware rejects STOP while disabled. A fresh successful
             * status read proving the driver disabled also releases ownership. */
            uint8_t flags=0;
            stopped=ZDT_Motor_ReadStatus((uint8_t)(i+1U),&flags)==HAL_OK&&(flags&1U)==0U;
        }
        if(stopped) g.stop_mask|=(uint8_t)(1U<<i);
    }
    return g.stop_mask==15U;
}
static void fail(const char *reason)
{
    g.reason=reason; g.state=GYRO_ERROR; g.stop_mask=0; (void)stop_all();
}
void GyroTurn_Init(void) { memset(&g,0,sizeof(g)); g.reason="0"; }
void GyroTurn_SafeBoot(void)
{
    fail("STARTUP_STOP");
    if(g.stop_mask==15U) { g.state=GYRO_IDLE; g.reason="0"; }
}
void GyroTurn_Stop(void)
{
    if(g.state==GYRO_RUNNING) fail("CANCELLED");
}
GyroTurnState GyroTurn_State(void) { return g.state; }
bool GyroTurn_Busy(void) { return g.state==GYRO_RUNNING||(g.state==GYRO_ERROR&&g.stop_mask!=15U); }
const char *GyroTurn_Reason(void) { return g.reason; }
bool GyroTurn_Start(const GyroCommand *c)
{
    ImuSample s;
    if(g.state==GYRO_RUNNING||(g.state==GYRO_ERROR&&g.stop_mask!=15U)) return false;
    g.reason="0";
    if(!ImuLink_Read(&s)||fabsf(s.rate)>3.0f) { g.reason="IMU_NOT_READY"; return false; }
    if(c->kind==GYRO_REFERENCE) {
        g.reference=s.yaw; g.reference_epoch=s.epoch; g.reference_valid=true; g.state=GYRO_DONE; return true;
    }
    if(c->kind==GYRO_HEADING&&(!g.reference_valid||g.reference_epoch!=s.epoch)) { g.reason="IMU_NO_REFERENCE"; return false; }
    g.driving=c->kind==GYRO_DRIVE; g.travel_done=false; g.forward_output=0;
    if(g.driving) {
        g.target=s.yaw; g.direction=c->direction=='F'?1:-1;
        g.distance_counts=(double)c->pulses*65536.0/c->pulses_per_revolution;
        g.progress_counts=0;
        for(unsigned i=0;i<4U;++i)
            if(ZDT_Motor_ReadWheelPosition((uint8_t)(i+1U),&g.origin[i])!=HAL_OK) {
                fail("ODOM_READ"); return false;
            }
    }
    else if(c->kind==GYRO_RELATIVE) g.target=s.yaw+(c->direction=='W'?c->degrees:-c->degrees);
    else {
        float delta=fmodf(c->degrees-(s.yaw-g.reference),360.0f);
        if(delta>180) delta-=360;
        if(delta< -180) delta+=360;
        g.target=s.yaw+delta;
    }
    g.rpm=(uint16_t)c->rpm; g.output=0; g.stop_mask=0;
    g.braking=g.stable=false; g.wheel=0; memset(g.guards,0,sizeof(g.guards));
    g.started=g.updated=g.status_tick=HAL_GetTick(); g.sample_sequence=s.sequence;
    g.sample_epoch=s.epoch;
    g.initial_error=g.target-s.yaw;
    g.progress_yaw=s.yaw; g.progress_tick=g.started;
    g.state=GYRO_RUNNING;
    for(unsigned i=1;i<=4U;++i) if(ZDT_Motor_Enable((uint8_t)i,true)!=HAL_OK) { fail("MOTOR_ENABLE"); return false; }
    return true;
}
void GyroTurn_Tick(void)
{
    if(g.state==GYRO_ERROR) { if(g.stop_mask!=15U) (void)stop_all(); return; }
    if(g.state!=GYRO_RUNNING) return;
    uint32_t now=HAL_GetTick(); ImuSample s;
    if(!ImuLink_Read(&s)) { fail("IMU_TIMEOUT"); return; }
    if(s.epoch!=g.sample_epoch) { fail("IMU_RESET"); return; }
    if((uint32_t)(now-g.started)>(g.driving?1800000U:60000U)) { fail(g.driving?"DRIVE_TIMEOUT":"TURN_TIMEOUT"); return; }
    if((uint32_t)(now-g.status_tick)>=50U) {
        uint8_t flags=0; g.status_tick=now;
        HAL_StatusTypeDef status_result=ZDT_Motor_ReadStatus((uint8_t)(g.wheel+1U),&flags);
        if(status_result!=HAL_OK||
            MotorStatusGuard_Fault(&g.guards[g.wheel],flags,now)) {
            if(status_result==HAL_OK) ZDT_Motor_Fault((uint8_t)(g.wheel+1U),"DRIVER_FAULT");
            fail("MOTOR_FAULT"); return;
        }
        g.wheel=(g.wheel+1U)%4U;
    }
    if(s.sequence==g.sample_sequence) return;
    uint32_t elapsed=(uint32_t)(now-g.updated); if(elapsed<20U) return;
    g.updated=now; g.sample_sequence=s.sequence;
    float error=g.target-s.yaw;
    float forward=0;
    if(g.driving&&!g.travel_done) {
        double distance=0;
        for(unsigned i=0;i<4U;++i) {
            int64_t position;
            if(ZDT_Motor_ReadWheelPosition((uint8_t)(i+1U),&position)!=HAL_OK) { fail("ODOM_READ"); return; }
            /* Forward is negative in the shared wheel convention. Yaw terms
             * cancel in the four-wheel mean. Distance comes from encoders. */
            distance-=(double)(position-g.origin[i])*g.direction/4.0;
        }
        if(distance < -655.36) { fail("ODOM_DIRECTION"); return; }
        if(distance-g.progress_counts>=65.536) { g.progress_counts=distance; g.progress_tick=now; }
        if((uint32_t)(now-g.progress_tick)>5000U) { fail("DRIVE_NO_PROGRESS"); return; }
        double remaining=g.distance_counts-distance;
        if(remaining<=1.0) {
            g.travel_done=true; g.forward_output=0; g.progress_tick=now;
            g.progress_yaw=s.yaw;
            g.stop_mask=0;
            if(!stop_all()) { fail("MOTOR_STOP"); return; }
        } else {
            /* 60 RPM/s acceleration, begin braking using measured remaining
             * revolutions, plus a 50 ms control/bus delay allowance. */
            double safe=fmax(0.0,remaining/65536.0-fabs(g.forward_output)*0.05/60.0);
            float desired=fminf((float)g.rpm,(float)(sqrt(2.0*safe)*60.0));
            desired=fmaxf(1.0f,desired);
            float ramp=GYRO_SLEW_RPM_S*fminf(elapsed*0.001f,0.05f);
            g.forward_output=fminf(desired,g.forward_output+ramp);
            forward=g.direction*g.forward_output;
        }
    }
    if((!g.driving||g.travel_done)&&fabsf(s.yaw-g.progress_yaw)>=0.2f) { g.progress_yaw=s.yaw; g.progress_tick=now; }
    /* A wrong yaw sign or unintended rotation must not continue indefinitely. */
    if(fabsf(error)>fabsf(g.initial_error)+20.0f) { fail("IMU_DIRECTION"); return; }
    if((!g.driving||g.travel_done)&&fabsf(error)<=1.0f) {
        if(!g.braking) { g.stop_mask=0; g.braking=true; g.output=0; }
        if(!stop_all()) { fail("MOTOR_STOP"); return; }
        if(fabsf(s.rate)<=3.0f) {
            if(!g.stable) { g.stable=true; g.stable_tick=now; }
            if((uint32_t)(now-g.stable_tick)>=200U) g.state=GYRO_DONE;
        } else g.stable=false;
        return;
    }
    g.stable=false; g.braking=false; g.stop_mask=0;
    if((!g.driving||g.travel_done)&&(uint32_t)(now-g.progress_tick)>5000U) { fail("TURN_NO_PROGRESS"); return; }
    float output=GYRO_KP*error-GYRO_KD*s.rate;
    float limit=g.driving&&!g.travel_done?fminf(20.0f,fabsf(forward)*0.5f):fminf(60.0f,(float)g.rpm);
    output=fmaxf(-limit,fminf(limit,output));
    float slew=GYRO_SLEW_RPM_S*fminf(elapsed*0.001f,0.05f);
    output=fmaxf(g.output-slew,fminf(g.output+slew,output));
    output=fmaxf(-limit,fminf(limit,output)); g.output=output;
    double targets[4]; int16_t speeds[4];
    /* Correction sign is independent of forward/reverse direction. Leave
     * headroom so no wheel exceeds the requested maximum RPM. */
    forward=copysignf(fminf(fabsf(forward),fmaxf(0,(float)g.rpm-fabsf(output))),forward);
    RobotChassis_WheelTargets(forward,0,-output,targets);
    for(unsigned i=0;i<4U;++i) speeds[i]=(int16_t)lround(targets[i]);
    if(ZDT_Motor_SetWheelSpeeds(speeds,240U)!=HAL_OK) fail("MOTOR_COMMAND");
}
