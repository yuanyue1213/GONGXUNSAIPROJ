/* 起步、两次完整扫描握手，以及PC路线存储。仅在前台处理，不重入任务执行器。 */
#include "route_plan.h"
#include "robot_config.h"
#include "start_link.h"
#include "mission_runner.h"
#include "motion_control.h"
#include "task_protocol.h"
#include "nav_protocol.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef struct { uint16_t amount; uint8_t leg,turn,direction; } NavStep;
static NavStep steps[NAV_MAX_STEPS];
static unsigned zone,run_zone,scan_wait,count,received,next_leg;
static uint32_t run_id,pc_tick,expected_crc,crc;
static bool pc_known,waiting_path,receiving,ready;

static void reply(const char *kind,const char *reason)
{
    char line[96];
    snprintf(line,sizeof(line),"%s,%lu,%s\n",kind,(unsigned long)run_id,reason);
    StartLink_Publish(line);
}
void RoutePlan_Init(void)
{
    zone=run_zone=scan_wait=count=received=next_leg=0;
    run_id=pc_tick=expected_crc=crc=0;
    pc_known=waiting_path=receiving=ready=false;
}
bool RoutePlan_CanStart(void)
{
    return pc_known&&zone>=1U&&zone<=2U&&(uint32_t)(HAL_GetTick()-pc_tick)<NAV_PC_FRESH_MS;
}
/* 所有数字字段均拒绝负数、溢出、尾随字符；路线不能携带机械臂或任意执行指针。 */
bool RoutePlan_Command(const char *frame)
{
    const char *p=frame; uint32_t id=0,a,b,c;
    if(strncmp(p,"PC_HELLO,",9U)==0) {
        p+=9;
        if(!TaskProtocol_U32(&p,&a,0)||a<1U||a>2U) return true;
        if(MissionRunner_Running()&&a!=run_zone) { reply("NAV_ERROR","BUSY"); return true; }
        zone=a; if(!MissionRunner_Running()) run_zone=a;
        pc_known=true; pc_tick=HAL_GetTick();
        char line[32]; snprintf(line,sizeof(line),"PC_READY,%u\n",zone);
        StartLink_Publish(line);
        return true;
    }
    if(strncmp(p,"SCAN_OK,",8U)==0) {
        p+=8;
        if(TaskProtocol_U32(&p,&id,',')&&TaskProtocol_U32(&p,&a,0)&&id==run_id&&a==scan_wait&&scan_wait) {
            if(scan_wait==2U) waiting_path=true;
            scan_wait=0;
        }
        return true;
    }
    if(strncmp(p,"NAV_",4U)!=0) return false;
    if(strncmp(p,"NAV_BEGIN,",10U)==0) {
        p+=10;
        if(!TaskProtocol_U32(&p,&id,',')||!TaskProtocol_U32(&p,&a,',')||
           !TaskProtocol_U32(&p,&b,',')||strlen(p)!=8U||strspn(p,"0123456789abcdefABCDEF")!=8U) {
            reply("NAV_ERROR","FORMAT"); return true;
        }
        if(id!=run_id||!waiting_path||a!=run_zone||!b||b>NAV_MAX_STEPS) { reply("NAV_ERROR","STATE"); return true; }
        c=(uint32_t)strtoul(p,NULL,16);
        if(!receiving||count!=b||expected_crc!=c) {
            count=b; received=0; expected_crc=c; ready=false; receiving=true;
            char header[64];
            int n=snprintf(header,sizeof(header),"%lu,%lu,%lu\n",(unsigned long)id,(unsigned long)a,(unsigned long)b);
            crc=Nav_Crc32(UINT32_MAX,header,(size_t)n);
        }
        reply("NAV_ACK","BEGIN"); return true;
    }
    if(strncmp(p,"NAV_STEP,",9U)==0) {
        p+=9; NavStep step={0};
        if(!TaskProtocol_U32(&p,&id,',')||!TaskProtocol_U32(&p,&a,',')||!TaskProtocol_U32(&p,&b,',')) {
            reply("NAV_ERROR","FORMAT"); return true;
        }
        if(strncmp(p,"MOVE,",5U)==0) { p+=5; step.turn=0; }
        else if(strncmp(p,"TURN,",5U)==0) { p+=5; step.turn=1; }
        else { reply("NAV_ERROR","OP"); return true; }
        char direction=*p;
        const char *value=direction?p+2:p;
        if(!direction||p[1]!=','||!TaskProtocol_U32(&value,&c,0)||
           (step.turn?(direction!='L'&&direction!='R')||c!=90U:
            (direction!='F'&&direction!='B'&&direction!='L'&&direction!='R')||!c||c>10000U)) {
            reply("NAV_ERROR","VALUE"); return true;
        }
        if(id!=run_id||!waiting_path||!receiving||a>=count||a>received||b<1U||b>8U) {
            reply("NAV_ERROR","SEQUENCE"); return true;
        }
        step.leg=(uint8_t)b; step.direction=(uint8_t)direction; step.amount=(uint16_t)c;
        if(a<received) {
            NavStep old=steps[a];
            if(old.leg!=step.leg||old.turn!=step.turn||old.direction!=step.direction||old.amount!=step.amount) {
                reply("NAV_ERROR","DUPLICATE"); return true;
            }
        } else {
            if(received&&step.leg<steps[received-1U].leg) { reply("NAV_ERROR","LEG"); return true; }
            steps[received++]=step;
            crc=Nav_Crc32(crc,frame,strlen(frame)); crc=Nav_Crc32(crc,"\n",1);
        }
        char seq[16]; snprintf(seq,sizeof(seq),"%lu",(unsigned long)a); reply("NAV_ACK",seq); return true;
    }
    if(strncmp(p,"NAV_END,",8U)==0) {
        p+=8;
        if(!TaskProtocol_U32(&p,&id,0)||id!=run_id) { reply("NAV_ERROR","ID"); return true; }
        if(ready) { reply("NAV_READY","OK"); return true; } /* END确认丢失时只重发确认。 */
        if(!waiting_path||!receiving||received!=count||!count) { reply("NAV_ERROR","COUNT"); return true; }
        unsigned legs=0;
        for(unsigned i=0;i<count;++i) legs|=1U<<(steps[i].leg-1U);
        if((crc^UINT32_MAX)!=expected_crc||legs!=255U) { receiving=false; reply("NAV_ERROR","CRC_OR_LEGS"); return true; }
        ready=true; receiving=false; waiting_path=false; next_leg=1;
        reply("NAV_READY","OK"); return true;
    }
    reply("NAV_ERROR","FORMAT"); return true;
}
static void scan(unsigned number,int y,unsigned heading)
{
    scan_wait=number;
    uint32_t sent=HAL_GetTick()-1000U;
    while(scan_wait) {
        if((uint32_t)(HAL_GetTick()-sent)>=1000U) {
            char line[96];
            snprintf(line,sizeof(line),"SCAN_READY,%lu,%u,%u,%d,%u\n",(unsigned long)run_id,number,
                     NAV_START_X_MM-NAV_ENTRY_LEFT_MM,y,heading);
            StartLink_Publish(line); sent=HAL_GetTick();
        }
        StartLink_Service();
    }
}
bool RoutePlan_Prepare(uint32_t run)
{
    if(!RoutePlan_CanStart()) return false;
    run_id=run; run_zone=zone; ready=receiving=waiting_path=false; received=count=0;
    if(!Motion_Move(zone==2U?'R':'L',NAV_ENTRY_LEFT_MM,MISSION_ROUTE_RPM)||
       !Motion_Move('F',NAV_ENTRY_FORWARD_MM,MISSION_ROUTE_RPM)) return false;
    int y=zone==2U?NAV_TOP_Y_MM-NAV_ENTRY_FORWARD_MM:NAV_BOTTOM_Y_MM+NAV_ENTRY_FORWARD_MM;
    Motion_Wait(NAV_SCAN_SETTLE_MS); scan(1,y,zone==2U?180U:0U);
    if(!Motion_Turn(zone==2U?'R':'L',90,MISSION_TURN_RPM)) return false;
    Motion_Wait(NAV_SCAN_SETTLE_MS); scan(2,y,270);
    uint32_t sent=HAL_GetTick()-1000U;
    while(!ready) {
        if((uint32_t)(HAL_GetTick()-sent)>=1000U) { reply("NAV_WAIT","PLAN"); sent=HAL_GetTick(); }
        StartLink_Service();
    }
    return true;
}
bool RoutePlan_Run(unsigned leg)
{
    if(!ready||leg!=next_leg||leg>8U) return false;
    for(unsigned i=0;i<count;++i) if(steps[i].leg==leg) {
        NavStep s=steps[i]; char line[96];
        snprintf(line,sizeof(line),"NAV_STATE,%lu,%u,%u,RUNNING\n",(unsigned long)run_id,leg,i);
        StartLink_Publish(line);
        bool ok=s.turn?Motion_Turn((char)s.direction,s.amount,MISSION_TURN_RPM):
            Motion_Move((char)s.direction,s.amount,MISSION_ROUTE_RPM);
        if(!ok) return false;
        snprintf(line,sizeof(line),"NAV_STATE,%lu,%u,%u,DONE\n",(unsigned long)run_id,leg,i);
        StartLink_Publish(line);
    }
    ++next_leg; return true;
}
