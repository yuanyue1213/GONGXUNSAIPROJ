#include <assert.h>
#include <stdio.h>
#include "arm_flow.h"

static unsigned calls,fail_at,waits;
static int32_t radial,height;
static uint16_t base,grip,turntable;
static uint32_t waited;
static int32_t last_r[2],last_z[2],observed_z,observe_r;
static uint16_t observe_base;
static unsigned observed;
static bool next(void) { return ++calls!=fail_at; }
bool Motion_MoveAll(int32_t r,int32_t z,uint16_t b,uint16_t rpm)
{
    assert(rpm==MOTION_ARM_RPM||rpm==MOTION_STOW_RPM);
    if(!next()) return false;
    last_r[0]=last_r[1]; last_z[0]=last_z[1];
    last_r[1]=r; last_z[1]=z;
    if(r==observe_r&&b==observe_base&&z!=0) { observed_z=z; ++observed; }
    radial=r; height=z; base=b; return true;
}
bool Motion_Servo(char c,uint16_t a,uint16_t dps)
{
    assert(dps==(c=='G'?MOTION_GRIPPER_DPS:c=='T'?MOTION_TURNTABLE_DPS:MOTION_BASE_TRANSFER_DPS));
    if(!next()) return false;
    if(c=='G') grip=a;
    else if(c=='T') turntable=a;
    else base=a;
    return true;
}
bool ServoControl_SetAngle(char c,uint16_t a) { return Motion_Servo(c,a,c=='G'?MOTION_GRIPPER_DPS:c=='T'?MOTION_TURNTABLE_DPS:MOTION_BASE_TRANSFER_DPS); }
void Motion_Wait(uint32_t ms) { ++waits; waited+=ms; }
int32_t Motion_Radial(void) { return radial; }
bool ServoControl_GetAngle(char c,uint16_t *angle) { assert(c=='B'); *angle=base; return true; }
int main(void)
{
    const StationPose poses[]=ARM_STATION_POSES;
    const StationPose raw_poses[]=ARM_RAW_POSES;
    const ArmStep prepare[]={ARM_PREPARE_STEPS};
    assert(ArmFlow_Run(prepare,ARM_COUNT(prepare))&&grip==15&&base==90&&turntable==0);
    for(unsigned i=0;i<3;++i) {
        const StationPose *p=&poses[i];
        const ArmStep approach[]={ARM_PICK_APPROACH(ARM_AREA_RAW,i)};
        const ArmStep finish[]={ARM_PICK_FINISH(ARM_AREA_RAW,i)};
        assert(ArmFlow_Run(approach,ARM_COUNT(approach)));
        assert(radial==ARM_READY_R&&height==0&&base==raw_poses[i].base&&grip==30);
        unsigned before=waits; uint32_t before_ms=waited;
        assert(ArmFlow_Run(finish,ARM_COUNT(finish)));
        assert(radial==110&&height==0&&base==180&&grip==30);
        assert(waits==before+2&&waited==before_ms+ARM_GRIP_SETTLE_MS+ARM_RELEASE_SETTLE_MS);
        for(unsigned stack=0;stack<2;++stack) {
            ArmArea area=stack?ARM_AREA_STACK:ARM_AREA_PROCESS;
            const ArmStep put[]={ARM_PUT_STEPS(area,i)};
            observe_r=p->put_r; observe_base=p->base; observed=0;
            /* 放置后抬升收回；中途失败不得继续执行释放及后续动作。 */
            assert(ArmFlow_Run(put,ARM_COUNT(put)));
            assert(radial==110&&height==0&&base==p->base&&grip==30);
            assert(observed==1&&observed_z==p->z+(stack?60:0));
        }
        calls=0; fail_at=2; before=waits;
        assert(!ArmFlow_Run(finish,ARM_COUNT(finish))&&calls==2&&waits==before);
        fail_at=0;
    }
    for(unsigned area=ARM_AREA_RAW;area<=ARM_AREA_STACK;++area) {
        for(unsigned position=ARM_LEFT;position<=ARM_RIGHT;++position) {
            const StationPose *p=area==ARM_AREA_RAW?&raw_poses[position]:&poses[position];
            const ArmStep pick[]={ARM_AREA(area),ARM_TO_PICK(position)};
            assert(ArmFlow_Run(pick,ARM_COUNT(pick)));
            assert(radial==p->pick_r&&height==p->z&&base==p->base);
            const ArmStep put[]={ARM_AREA(area),ARM_TO_PUT(position)};
            assert(ArmFlow_Run(put,ARM_COUNT(put)));
            assert(radial==p->put_r&&base==p->base);
            assert(height==p->z+(area==ARM_AREA_STACK?60:0));
            const ArmStep above[]={ARM_AREA(area),ARM_ABOVE_PUT(position)};
            assert(ArmFlow_Run(above,ARM_COUNT(above))&&height==0&&radial==p->put_r);
            const ArmStep ready[]={ARM_AREA(area),ARM_TO_READY(position)};
            assert(ArmFlow_Run(ready,ARM_COUNT(ready))&&height==0&&radial==ARM_READY_R&&base==p->base);
        }
    }
    unsigned before_calls=calls;
    const ArmStep no_area[]={ARM_TO_PICK(ARM_CENTER)};
    assert(!ArmFlow_Run(no_area,ARM_COUNT(no_area))&&calls==before_calls);
    const ArmStep bad_area[]={ARM_AREA(99),ARM_MOVE(110,0,170)};
    assert(!ArmFlow_Run(bad_area,ARM_COUNT(bad_area))&&calls==before_calls);
    const ArmStep bad_position[]={ARM_AREA(ARM_AREA_RAW),ARM_TO_PUT(3)};
    assert(!ArmFlow_Run(bad_position,ARM_COUNT(bad_position))&&calls==before_calls);
    const ArmStep switch_area[]={ARM_AREA(ARM_AREA_PROCESS),ARM_TO_PUT(ARM_CENTER),
        ARM_AREA(ARM_AREA_STACK),ARM_TO_PUT(ARM_CENTER)};
    assert(ArmFlow_Run(switch_area,ARM_COUNT(switch_area))&&height== -66);
    radial=1230; height=-100; base=209;
    assert(ArmFlow_Stow()&&radial==0&&height==0&&base==180&&turntable==0);
    assert(last_r[0]==1230&&last_z[0]==0&&last_r[1]==0&&last_z[1]==0);
    puts("PASS: arm sequences, four work areas, left/center/right poses, stack height, independent group scope, invalid areas/positions, waits and failure exit");
}
