/* 参考项目的动作组织方式：直接调用、预计时间等待、多轴同时发出后取最大耗时。
 * 保留本车 UART 电机协议、方向、丝杆尺寸及 TIM6 舵机轨迹，不使用 ACTION 状态机。 */
#include "motion_control.h"
#include "mission_config.h"
#include "zdt_motor.h"
#include "lift_motor.h"
#include "servo_control.h"
#include "servo_motion.h"
#include "imu_link.h"
#include "chassis_wheel_math.h"
#include <math.h>
#include <stdlib.h>

static MotionService service;
static int32_t radial, height;
static float heading;

void Motion_Init(MotionService callback)
{
    service=callback;
    radial=height=0;
    heading=0;
}
void Motion_Wait(uint32_t ms)
{
    uint32_t start=HAL_GetTick();
    while((uint32_t)(HAL_GetTick()-start)<ms) {
        if(service) service();
    }
}
/* 与参考项目相同的三角形/梯形速度时间估计，统一使用转数、RPM、RPM/s。
 * 这里只估计动作时间；没有逐轴到位查询或运行超时停车。 */
uint32_t Motion_EstimateMs(double revolutions,uint16_t rpm,double acceleration)
{
    if(revolutions<=0.0||!rpm||acceleration<=0.0) return 0;
    double ramp_revolutions=(double)rpm*rpm/(60.0*acceleration);
    double seconds=revolutions<=ramp_revolutions?
        2.0*sqrt(revolutions*60.0/acceleration):
        2.0*rpm/acceleration+(revolutions-ramp_revolutions)*60.0/rpm;
    return (uint32_t)ceil(seconds*1000.0)+MISSION_MOTION_SETTLE_MS;
}
static ImuSample wait_imu(void)
{
    ImuSample sample;
    while(!ImuLink_Read(&sample)) {
        if(service) service();
    }
    return sample;
}
bool Motion_Reference(void)
{
    heading=wait_imu().yaw;
    return true;
}
static bool wheels_enable(void)
{
    for(uint8_t id=1;id<=4;++id) {
        if(ZDT_Motor_Enable(id)!=HAL_OK) return false;
    }
    return true;
}
static bool wheels_stop(void)
{
    bool ok=true;
    for(uint8_t id=1;id<=4;++id)
        if(ZDT_Motor_Stop(id)!=HAL_OK) ok=false;
    return ok;
}
bool Motion_Move(char direction,float mm,uint16_t rpm)
{
    if(mm<0||!rpm||rpm>STEPPER_MAX_SPEED_RPM) return false;
    bool lateral=direction=='L'||direction=='R';
    int32_t pulses=(int32_t)lroundf(mm*(lateral?MISSION_LATERAL_PPM:MISSION_FORWARD_PPM)/1000.0f);
    if(!pulses) return true;
    double target[4]; int32_t wheel[4];
    switch(direction) {
    case 'F': RobotChassis_WheelTargets(-pulses,0,0,target); break;
    case 'B': RobotChassis_WheelTargets(pulses,0,0,target); break;
    case 'L': RobotChassis_WheelTargets(0,-pulses,0,target); break;
    case 'R': RobotChassis_WheelTargets(0,pulses,0,target); break;
    default: return false;
    }
    for(unsigned i=0;i<4;++i) wheel[i]=(int32_t)target[i];
    if(!wheels_enable()) return false;
    if(ZDT_Motor_MoveWheelSynchronous(wheel,rpm,MISSION_WHEEL_ACCEL)!=HAL_OK) return false;
    double revolutions=(double)pulses/MISSION_WHEEL_PPR;
    /* Emm档位不是RPM/s：每(256-acc)*50us改变1RPM，acc=0为无加减速。
     * 见X42S手册Emm相对位置控制章节；不能把档位10当成10RPM/s。 */
    uint32_t duration=MISSION_WHEEL_ACCEL==0U?
        (uint32_t)ceil(revolutions*60000.0/rpm)+MISSION_MOTION_SETTLE_MS:
        Motion_EstimateMs(revolutions,rpm,20000.0/(256.0-MISSION_WHEEL_ACCEL));
    Motion_Wait(duration);
    return true;
}
bool Motion_Turn(char direction,float degrees,uint16_t rpm)
{
    if((direction!='L'&&direction!='R')||degrees<=0||degrees>360||!rpm||rpm>STEPPER_MAX_SPEED_RPM) return false;
    heading+=direction=='L'?-degrees:degrees;
    if(!wheels_enable()) return false;
    /* 保留参考项目的 IMU 闭环转向思想，但串口命令放在前台，避免在中断内等待 UART。
     * 等待达到目标，不设总超时，也不轮询电机状态；IMU 失联时先停轮再等待恢复。 */
    uint32_t stable_since=0;
    bool stable=false;
    for(;;) {
        ImuSample s;
        if(!ImuLink_Read(&s)) {
            if(!wheels_stop()) return false;
            stable=false;
            s=wait_imu();
        }
        float error=heading-s.yaw;
        if(fabsf(error)<=TURN_ANGLE_TOLERANCE&&fabsf(s.rate)<=TURN_RATE_TOLERANCE) {
            if(!stable) {
                if(!wheels_stop()) return false;
                stable_since=HAL_GetTick();
                stable=true;
            }
            if((uint32_t)(HAL_GetTick()-stable_since)>=TURN_STABLE_MS) return true;
        } else {
            stable=false;
            float output=fmaxf(-(float)rpm,fminf(rpm,TURN_KP*error-TURN_KD*s.rate));
            double target[4]; int16_t speed[4];
            RobotChassis_WheelTargets(0,0,-output,target);
            for(unsigned i=0;i<4;++i) speed[i]=(int16_t)lround(target[i]);
            if(ZDT_Motor_SetWheelSpeedsSynchronous(speed,TURN_WHEEL_ACCEL)!=HAL_OK) return false;
        }
        Motion_Wait(TURN_LOOP_MS);
    }
}
static bool servo_start(char channel,uint16_t target,uint16_t dps,uint32_t *duration)
{
    uint16_t current;
    if(!dps||!ServoControl_GetAngle(channel,&current)) return false;
    *duration=ServoMotion_Duration((uint32_t)abs((int)target-current),dps);
    return ServoControl_MoveTo(channel,target,*duration);
}
bool Motion_Servo(char channel,uint16_t target,uint16_t dps)
{
    uint32_t duration;
    if(!servo_start(channel,target,dps,&duration)) return false;
    Motion_Wait(duration+SERVO_UPDATE_MS);
    return true;
}
bool Motion_MoveAll(int32_t r,int32_t z,uint16_t base,uint16_t rpm)
{
    if(!rpm||rpm>STEPPER_MAX_SPEED_RPM||base>360U) return false;
    int32_t dr=r-radial,dz=z-height;
    uint32_t r_angle=(uint32_t)llround(fabs((double)dr)*360.0/ARM_RADIAL_MM_PER_REV);
    uint32_t z_angle=(uint32_t)llround(fabs((double)dz)*3600.0/ARM_LIFT_MM_PER_REV);
    uint32_t base_ms;
    /* 两个步进位置指令依次发送，电机独立执行；舵机由 TIM6 在后台推进。 */
    if(r_angle&&ArmMotor_MoveAngle(dr>0?'E':'C',r_angle,rpm,ARM_PULSES_PER_REV)!=HAL_OK) return false;
    if(z_angle&&LiftMotor_MoveAngle(dz>0?'U':'D',z_angle,rpm,ARM_PULSES_PER_REV)!=HAL_OK) return false;
    if(!servo_start('B',base,MOTION_BASE_TRANSFER_DPS,&base_ms)) return false;
    double ra=LiftMotor_FirmwareCode(2U)=='X'?ARM_X_ACCEL_RPM_S:MISSION_ARM_EMM_ACCEL;
    double za=LiftMotor_FirmwareCode(1U)=='X'?ARM_X_ACCEL_RPM_S:MISSION_ARM_EMM_ACCEL;
    uint32_t r_ms=Motion_EstimateMs(r_angle/3600.0,rpm,ra);
    uint32_t z_ms=Motion_EstimateMs(z_angle/3600.0,rpm,za);
    uint32_t duration=r_ms>z_ms?r_ms:z_ms;
    if(base_ms+SERVO_UPDATE_MS>duration) duration=base_ms+SERVO_UPDATE_MS;
    Motion_Wait(duration);
    /* 与参考项目一样更新逻辑位置；不读取电机实际位置作完成判定。 */
    radial=r;
    height=z;
    return true;
}
int32_t Motion_Radial(void) { return radial; }
void Motion_Stop(void)
{
    (void)wheels_stop();
    (void)LiftMotor_Stop();
    (void)ArmMotor_StopForeAft();
    ServoControl_Cancel('G');
    ServoControl_Cancel('T');
    ServoControl_Cancel('B');
}
