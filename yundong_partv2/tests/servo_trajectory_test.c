#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "servo_trajectory.h"
#include "servo_motion.h"
#include "motion_speed.h"
int main(void)
{
    assert(MOTION_GRIPPER_DPS==300);
    assert(MOTION_BASE_ALIGN_DPS==360&&MOTION_BASE_TRANSFER_DPS==360&&MOTION_TURNTABLE_DPS==360);
    assert(ServoMotion_Duration(100,300)==625);
    assert(ServoMotion_Duration(180,360)==938);
    assert(ServoMotion_Duration(0,360)==0);
    /* The peak derivative is bounded, including integer-duration rounding. */
    assert(1875.0*180/ServoMotion_Duration(180,360)<=360.0);
    ServoTrajectory jobs[3]={0};
    for(unsigned i=0;i<3U;++i) ServoTrajectory_Set(&jobs[i],0);
    ServoTrajectory_Start(&jobs[0],180,1000,0);
    ServoTrajectory_Start(&jobs[1],120,2000,0);
    ServoTrajectory_Start(&jobs[2],270,3000,0);
    unsigned previous[3]={0};
    for(unsigned now=0;now<=3000;now+=20) {
        for(unsigned i=0;i<3;++i) {
            unsigned angle=ServoTrajectory_Update(&jobs[i],now);
            assert(angle>=previous[i]); previous[i]=angle;
        }
        if(now==1000) assert(!jobs[0].active&&jobs[1].active&&jobs[2].active);
        if(now==2000) assert(!jobs[1].active&&jobs[2].active);
    }
    assert(jobs[0].angle==180&&jobs[1].angle==120&&jobs[2].angle==270&&!jobs[2].active);
    ServoTrajectory_Start(&jobs[0],0,1000,3000);
    assert(ServoTrajectory_Update(&jobs[0],3500)==90);
    ServoTrajectory_Start(&jobs[0],180,1000,3500);
    assert(jobs[0].start==90&&jobs[0].angle==90);
    ServoTrajectory_Update(&jobs[0],4000); assert(jobs[0].angle==135);
    jobs[0].active=false; ServoTrajectory_Update(&jobs[0],6000); assert(jobs[0].angle==135);
    ServoTrajectory_Start(&jobs[0],0,1000,UINT32_MAX-499U);
    assert(ServoTrajectory_Update(&jobs[0],0)==68);
    assert(ServoTrajectory_Update(&jobs[0],500)==0&&!jobs[0].active);
    puts("PASS: independent 20 ms servo trajectories, three-axis barrier, replacement, hold and timer wrap");
}
