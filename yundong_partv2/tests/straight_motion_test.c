#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_control.h"
#include "robot_config.h"
#include "zdt_motor.h"

static unsigned moves,enabled,stops,emergency,fail_enable,fail_stop;
static uint32_t waited;
static int32_t last[4];
static bool fail_move;
HAL_StatusTypeDef ZDT_Motor_Enable(uint8_t id)
{
    assert(id>=1&&id<=4); ++enabled; return id==fail_enable?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef ZDT_Motor_StopWheelsImmediate(void)
{
    ++stops; return stops==fail_stop?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef ZDT_Motor_MoveWheelSynchronous(const int32_t pulses[4],uint16_t rpm,uint8_t acceleration)
{
    assert(enabled==4&&stops==1&&rpm==200&&acceleration==20);
    for(unsigned i=0;i<4;++i) assert(pulses[i]==pulses[0]);
    memcpy(last,pulses,sizeof(last)); ++moves; return fail_move?HAL_ERROR:HAL_OK;
}
void Motion_Wait(uint32_t ms) { assert(moves==1&&stops==1); waited=ms; }
void Motion_Stop(void) { ++emergency; }
static void reset(void)
{
    moves=enabled=stops=emergency=fail_enable=fail_stop=waited=0;
    fail_move=false; memset(last,0,sizeof(last));
}
int main(void)
{
    for(unsigned backwards=0;backwards<2;++backwards) {
        reset();
        assert(Motion_MoveStraight(backwards?'B':'F',3000,200));
        assert(last[0]==(backwards?-39192:39192)&&waited==6085);
        assert(stops==2&&moves==1&&!emergency);
    }
    reset(); assert(Motion_MoveStraight('F',100,200));
    assert(last[0]==1306&&waited>1000&&waited<1200&&stops==2);
    reset(); fail_enable=2;
    assert(!Motion_MoveStraight('F',3000,200)&&!moves&&!waited&&emergency==1);
    reset(); fail_move=true;
    assert(!Motion_MoveStraight('F',3000,200)&&moves==1&&!waited&&emergency==1);
    for(unsigned failure=1;failure<=2;++failure) {
        reset(); fail_stop=failure;
        assert(!Motion_MoveStraight('F',3000,200)&&emergency==1);
    }
    reset();
    assert(!Motion_MoveStraight('L',3000,200)&&!Motion_MoveStraight('F',NAN,200));
    assert(!Motion_MoveStraight('F',0,200)&&!Motion_MoveStraight('F',3000,0));
    assert(!Motion_MoveStraight('F',3000,STEPPER_MAX_SPEED_RPM+1));
    assert(!Motion_MoveStraight('F',1e10f,200)&&!moves&&!stops);
    puts("PASS: pure FD position F/B 3000mm at 200RPM/20, trapezoid/triangle wait, no IMU/compensation dependency, failure stop");
}
