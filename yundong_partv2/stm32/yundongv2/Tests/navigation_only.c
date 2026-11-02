/* NavigationOnly独立固件：只跑起步、两次雷达扫描和PC下载的八段导航。
 * 仍由App START触发；到各站直接继续，不扫码、不视觉校准、不取放或收臂。 */
#include "mission_runner.h"
#include "route_plan.h"
#include "motion_control.h"
#include "start_link.h"
#include <stdio.h>

static bool running;
bool MissionRunner_Running(void) { return running; }
static void report(uint32_t id,const char *state,unsigned leg,const char *reason)
{
    char line[96];
    snprintf(line,sizeof(line),"AUTO,%lu,%s,%u,0,0,0,%s\n",(unsigned long)id,state,leg,reason);
    StartLink_Publish(line);
}
void MissionRunner_Run(uint32_t id)
{
    if(running) return;
    running=true;
    unsigned leg=0;
    report(id,"STARTING",leg,"NAV_ONLY");
    if(!Motion_Reference()||!RoutePlan_Prepare(id)) goto failed;
    for(leg=NAV_QR;leg<=NAV_RETURN;++leg) {
        report(id,"RUNNING",leg,"NAV_ONLY");
        if(!RoutePlan_Run(leg)) goto failed;
    }
    report(id,"DONE",NAV_RETURN,"NAV_ONLY");
    running=false;
    return;
failed:
    Motion_Stop();
    report(id,"STOPPED",leg,"COMMAND_FAILED");
    running=false;
}
