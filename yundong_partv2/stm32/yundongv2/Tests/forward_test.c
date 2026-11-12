/* 独立直行固件：App启动一次3000mm，纯位置模式直行并停车，不使用IMU；无需PC/雷达。 */
#include "mission_runner.h"
#include "motion_control.h"
#include "robot_config.h"
#include "start_link.h"
#include <stdio.h>

static bool running;
bool MissionRunner_Running(void) { return running; }
static void report(uint32_t id,const char *state,const char *reason)
{
    char line[96];
    snprintf(line,sizeof(line),"AUTO,%lu,%s,1,0,0,0,%s\n",(unsigned long)id,state,reason);
    StartLink_Publish(line);
}
void MissionRunner_Run(uint32_t id)
{
    if(running) return;
    running=true;
    report(id,"RUNNING","FORWARD_TEST");
    bool ok=Motion_MoveStraight('F',FORWARD_TEST_DISTANCE_MM,FORWARD_TEST_RPM);
    report(id,ok?"DONE":"STOPPED",ok?"FORWARD_TEST":"COMMAND_FAILED");
    running=false;
}
