/* 小动作执行循环；动作表和硬件驱动分别维护。 */
#include "arm_flow.h"
#include "motion_control.h"
#include "servo_control.h"

static const StationPose area_poses[][3]={
    ARM_RAW_POSES,
    ARM_PROCESS_POSES,
    ARM_STORE_POSES,
    ARM_STACK_POSES,
};
static const StationPose *pose(ArmArea area,unsigned position)
{
    if(area<ARM_AREA_RAW||area>ARM_AREA_STACK||position>ARM_RIGHT) return NULL;
    return &area_poses[(unsigned)area-1U][position];
}

bool ArmFlow_Run(const ArmStep *steps,size_t count)
{
    ArmArea area=ARM_AREA_NONE; /* 每次执行独立，不沿用上一组的区域。 */
    for(size_t i=0;i<count;++i) {
        const ArmStep *s=&steps[i];
        switch(s->op) {
        case ARM_OP_AREA:
            if(s->value<ARM_AREA_RAW||s->value>ARM_AREA_STACK) return false;
            area=(ArmArea)s->value;
            break;
        case ARM_OP_PICK_POSITION:
        case ARM_OP_PUT_POSITION:
        case ARM_OP_READY_POSITION:
        case ARM_OP_PICK_ABOVE:
        case ARM_OP_PUT_ABOVE: {
            const StationPose *p=pose(area,s->value);
            if(!p) return false;
            bool put=s->op==ARM_OP_PUT_POSITION||s->op==ARM_OP_PUT_ABOVE;
            bool down=s->op==ARM_OP_PICK_POSITION||s->op==ARM_OP_PUT_POSITION;
            int32_t r=put?p->put_r:p->pick_r;
            if(s->op==ARM_OP_READY_POSITION) r=ARM_READY_R;
            int32_t z=down?p->z:0;
            if(s->op==ARM_OP_PUT_POSITION&&area==ARM_AREA_STACK) z+=ARM_STACK_HEIGHT;
            if(!Motion_MoveAll(r,z,p->base,s->speed)) return false;
            break;
        }
        case ARM_OP_MOVE:
            if(!Motion_MoveAll(s->r,s->z,s->base,s->speed)) return false;
            break;
        case ARM_OP_SERVO:
            if(s->value>UINT16_MAX) return false;
            if(!Motion_Servo(s->channel,(uint16_t)s->value,s->speed)) return false;
            break;
        case ARM_OP_WAIT:
            Motion_Wait(s->value);
            break;
        case ARM_OP_SET:
            if(s->value>UINT16_MAX) return false;
            if(!ServoControl_SetAngle(s->channel,(uint16_t)s->value)) return false;
            break;
        default:
            return false;
        }
    }
    return true;
}

bool ArmFlow_Stow(void)
{
    uint16_t base;
    if(!ServoControl_GetAngle('B',&base)) return false;
    const ArmStep steps[]={ARM_STOW_STEPS(Motion_Radial(),base)};
    return ArmFlow_Run(steps,ARM_COUNT(steps));
}
