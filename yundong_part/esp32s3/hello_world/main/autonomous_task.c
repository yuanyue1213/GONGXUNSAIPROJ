#include "autonomous_task.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* S1 starts facing -Y. Travel uses in-place turns followed only by forward motion. */
typedef struct { float x,y; } Point;
static const Point route[]={{2250,2200},{2200,2200},{2200,1200}, /* QR stop */
    {2200,2200},{1200,2200}, /* raw 1 */
    {1200,1200},{1200,300}, /* process 1 */
    {1200,1200},{300,1200}, /* storage 1 */
    {300,2200},{1200,2200}, /* raw 2 */
    {1200,1200},{1200,300}, /* process 2 */
    {1200,1200},{300,1200}, /* storage 2 */
    {1200,1200},{2200,1200},{2200,300},{2250,300},{2250,150}};
typedef enum { OFF, READY, REFERENCE, STOW, NAV, QR, PREPARE, SLOT, OBSERVE, CORRECT, RESTORE, CARD, HOLD } Phase;
static struct {
    AutoOutput stm,event;
    Phase phase,next;
    bool running,waiting,have_code,vision_enabled,arm_enabled,options_loaded,config_error,drive_loaded,turning;
    uint32_t run,id,pending,since,last_query,settle_until,config_id,last_run;
    uint32_t forward,lateral,rpm,fine,mask,corrections,turn_rpm,correction_pulses,wheel_ppr;
    float target_x,target_y,x,y,dx,dy;
    char map_p[4],map_t[4];
    uint16_t slots[3];
    uint16_t turn_angle;
    char camera_line[64];
    RobotSequenceCommand profiles[5][3]; /* R,G,P,T,K */
    Point route[20];
    uint32_t route_mask;
    char manual_code[16];
    TaskCode code;
    unsigned node,group,item;
    unsigned grabs,puts;
    unsigned heading,pending_heading,correction_heading;
    float correction_dx,correction_dy;
} a;
static int role_index(char c) { const char *p=strchr("RGPTK",c); return p&&c?p-"RGPTK":-1; }
/* Conservative 300 mm square footprint on the published gray-lane map.
 * Actual random obstacles require an additional sensed occupancy map. */
static bool clear_point(float x,float y)
{
    if(x<150||x>2250||y<150||y>2250) return false;
    bool yellow_x=(x>400&&x<1150)||(x>1250&&x<2000);
    bool yellow_y=(y>400&&y<1150)||(y>1250&&y<2000);
    return !(yellow_x&&yellow_y);
}
static bool clear_segment(float x,float y,float dx,float dy)
{
    unsigned steps=(unsigned)ceilf(fmaxf(fabsf(dx),fabsf(dy))/10.0f)+1U;
    for(unsigned i=0;i<=steps;++i)
        if(!clear_point(x+dx*i/steps,y+dy*i/steps)) return false;
    return true;
}
static void emit(const char *state,const char *reason)
{
    char line[128]; snprintf(line,sizeof(line),"AUTO,%lu,%s,%u,%u,%u,%u,%s\n",
        (unsigned long)a.run,state,a.node,a.item,a.grabs,a.puts,reason);
    if(a.event) a.event(line);
}
void AutoTask_Init(AutoOutput stm,AutoOutput event)
{
    memset(&a,0,sizeof(a)); a.stm=stm; a.event=event;
}
bool AutoTask_Running(void) { return a.running; }
static void camera_idle(void)
{
    if(a.stm&&a.id<UINT32_MAX-1U) {
        char line[48]; snprintf(line,sizeof(line),"CAM,%lu,IDLE\n",(unsigned long)++a.id); a.stm(line);
    }
}
void AutoTask_Stop(const char *reason)
{
    if(a.stm&&a.running) {
        if(a.arm_enabled) { a.stm("CMD,0,Z\n"); a.stm("CMD,0,H\n"); a.stm("CMD,0,Q\n"); }
        a.stm("CMD,0,S\n"); camera_idle();
    }
    a.running=a.waiting=false; a.phase=OFF; emit("STOPPED",reason);
}
static uint32_t next_id(void)
{
    if(a.id>=UINT32_MAX-1U) { AutoTask_Stop("SEQUENCE_EXHAUSTED"); return 0U; }
    return ++a.id;
}
static void action(const char *body,Phase next,uint32_t now)
{
    uint32_t id=next_id(); if(!id) return;
    char line[192]; int n=snprintf(line,sizeof(line),"EXEC,%lu,%s\n",(unsigned long)id,body);
    if(n<0||(size_t)n>=sizeof(line)) { AutoTask_Stop("FRAME_LENGTH"); return; }
    a.pending=id; a.waiting=true; a.since=a.last_query=now; a.next=next;
    a.stm(line);
}
/* Clockwise headings are relative to the stationary JY901S reference at S1. */
static bool turn_to(unsigned heading,Phase next,uint32_t now)
{
    unsigned delta=(heading+4U-a.heading)%4U;
    if(!delta) return false;
    char body[64];
    snprintf(body,sizeof(body),"HEADING,%u.0,%lu",heading*90U,(unsigned long)a.turn_rpm);
    a.dx=a.dy=0; a.turning=true; a.pending_heading=heading;
    emit("TURNING","0"); action(body,next,now); return true;
}
static bool work_node(void)
{
    return a.node==4U||a.node==6U||a.node==8U||a.node==10U||a.node==12U||a.node==14U;
}
static bool permutation(const char *s)
{
    return strlen(s)==3U&&s[0]>='1'&&s[0]<='3'&&s[1]>='1'&&s[1]<='3'&&s[2]>='1'&&s[2]<='3'&&
        s[0]!=s[1]&&s[0]!=s[2]&&s[1]!=s[2];
}
bool AutoTask_Command(const char *line,uint32_t now)
{
    if(strncmp(line,"AUTO_",5U)!=0) return false;
    const char *p; uint32_t id;
    if(strncmp(line,"AUTO_STOP,",10U)==0) {
        p=line+10U; if(RobotProtocol_U32(&p,&id,'\0')&&id) AutoTask_Stop("USER"); return true;
    }
    if(a.running) { emit("BUSY","RUNNING"); return true; }
    bool prior_error=a.config_error;
    if(strncmp(line,"AUTO_START,",11U)!=0) a.config_error=true;
    if(strncmp(line,"AUTO_CFG,",9U)==0) {
        p=line+9U; uint32_t forward,lateral,rpm,fine,slots[3]; float tx,ty;
        char mp[4],mt[4];
        if(!RobotProtocol_U32(&p,&id,',')||id==0U||
            !RobotProtocol_U32(&p,&forward,',')||!RobotProtocol_U32(&p,&lateral,',')||
            !RobotProtocol_U32(&p,&rpm,',')||!RobotProtocol_U32(&p,&fine,',')||
            !CameraProtocol_Pixel(&p,&tx,',',512U)||!CameraProtocol_Pixel(&p,&ty,',',320U)||
            strlen(p)<8U) { emit("ERROR","CONFIG"); return true; }
        memcpy(mp,p,3U); mp[3]=0; p+=3U;
        if(*p++!=',') { emit("ERROR","CONFIG"); return true; }
        memcpy(mt,p,3U); mt[3]=0; p+=3U;
        if(*p++!=','||!permutation(mp)||!permutation(mt)||
            !RobotProtocol_U32(&p,&slots[0],',')||!RobotProtocol_U32(&p,&slots[1],',')||
            !RobotProtocol_U32(&p,&slots[2],'\0')||forward<1U||forward>1000000U||
            lateral<1U||lateral>1000000U||rpm<5U||rpm>300U||fine<5U||fine>300U||
            slots[0]>270U||slots[1]>270U||slots[2]>270U||
            slots[0]==slots[1]||slots[0]==slots[2]||slots[1]==slots[2]) {
            emit("ERROR","CONFIG"); return true;
        }
        a.config_id=id; a.forward=forward; a.lateral=lateral; a.rpm=rpm; a.fine=fine;
        a.target_x=tx; a.target_y=ty; memcpy(a.map_p,mp,4U); memcpy(a.map_t,mt,4U);
        for(unsigned i=0;i<3U;++i) a.slots[i]=(uint16_t)slots[i];
        a.mask=a.route_mask=0U; a.run=id; a.vision_enabled=a.arm_enabled=true;
        a.options_loaded=a.config_error=a.drive_loaded=false; a.manual_code[0]=0;
        memcpy(a.route,route,sizeof(route));
        emit("CONFIGURED","0"); return true;
    }
    if(strncmp(line,"AUTO_DRIVE,",11U)==0) {
        p=line+11U; uint32_t config,pulses,rpm,ppr;
        if(!RobotProtocol_U32(&p,&config,',')||!config||config!=a.config_id||
            !RobotProtocol_U32(&p,&pulses,',')||pulses<1U||pulses>500000U||
            !RobotProtocol_U32(&p,&rpm,',')||rpm<5U||rpm>60U||
            !RobotProtocol_U32(&p,&ppr,0)||ppr<200U||ppr>51200U) {
            emit("ERROR","DRIVE"); return true;
        }
        a.turn_rpm=rpm; a.wheel_ppr=ppr; a.drive_loaded=true;
        a.config_error=prior_error; emit("DRIVE","0"); return true;
    }
    if(strncmp(line,"AUTO_OPTIONS,",13U)==0) {
        p=line+13U; uint32_t config,vision,arm;
        if(!RobotProtocol_U32(&p,&config,',')||config!=a.config_id||!config||
            !RobotProtocol_U32(&p,&vision,',')||vision>1U||
            !RobotProtocol_U32(&p,&arm,',')||arm>1U) { emit("ERROR","OPTIONS"); return true; }
        TaskCode code;
        if(strcmp(p,"-")!=0&&!TaskProtocol_Code(p,&code)) { emit("ERROR","TASK_CODE"); return true; }
        if(!vision&&arm&&strcmp(p,"-")==0) { emit("ERROR","TASK_CODE_REQUIRED"); return true; }
        if(strcmp(p,"-")!=0) { memcpy(a.manual_code,p,16U); a.code=code; }
        else a.manual_code[0]=0;
        a.vision_enabled=vision!=0U; a.arm_enabled=arm!=0U; a.options_loaded=true;
        a.config_error=prior_error; emit("OPTIONS","0"); return true;
    }
    if(strncmp(line,"AUTO_ROUTE,",11U)==0) {
        p=line+11U; uint32_t config,index,x,y;
        if(!RobotProtocol_U32(&p,&config,',')||config!=a.config_id||!config||
            !RobotProtocol_U32(&p,&index,',')||index>=20U||
            !RobotProtocol_U32(&p,&x,',')||x>2400U||!RobotProtocol_U32(&p,&y,'\0')||y>2400U||
            !clear_point((float)x,(float)y)||(index==19U&&(x!=2250U||y!=150U))) { emit("ERROR","ROUTE"); return true; }
        a.route[index]=(Point){(float)x,(float)y}; a.route_mask|=1UL<<index;
        a.config_error=prior_error; emit("ROUTE","0"); return true;
    }
    if(strncmp(line,"AUTO_PROFILE,",13U)==0||strncmp(line,"AUTO_ARM,",9U)==0) {
        p=line+(strncmp(line,"AUTO_ARM,",9U)==0?9U:13U); uint32_t config,slot;
        if(!RobotProtocol_U32(&p,&config,',')||config!=a.config_id||!config) { emit("ERROR","PROFILE_CONFIG"); return true; }
        int role=role_index(*p++);
        if(role<0||*p++!=','||!RobotProtocol_U32(&p,&slot,',')||slot<1U||slot>3U) { emit("ERROR","PROFILE"); return true; }
        RobotSequenceCommand settings;
        if(strncmp(p,"STATE_V2,",9U)!=0||!RobotProtocol_ParseSequence(p,&settings)||
            settings.mode!=(role<2?'A':'P')) { emit("ERROR","PROFILE_MODE"); return true; }
        a.profiles[role][slot-1U]=settings; a.mask|=1U<<(role*3U+slot-1U);
        a.config_error=prior_error; emit("PARAMETERS","0"); return true;
    }
    if(strncmp(line,"AUTO_START,",11U)==0) {
        p=line+11U;
        if(!RobotProtocol_U32(&p,&id,',')||!id||id==a.last_run||
            !RobotProtocol_U32(&p,&a.run,'\0')||a.run!=a.config_id||a.config_error||!a.drive_loaded||
            (a.arm_enabled&&a.mask!=0x7FFFU)||
            (a.options_loaded&&a.route_mask!=0xFFFFFU)) {
            emit("ERROR","NOT_CONFIGURED_OR_DUPLICATE"); return true;
        }
        a.run=id; a.last_run=id; if(id>a.id) a.id=id;
        camera_idle();
        a.pending=next_id(); if(!a.pending) return true;
        a.running=true; a.waiting=false; a.phase=READY;
        a.since=a.last_query=now; a.have_code=!a.vision_enabled&&a.manual_code[0]!=0;
        a.x=2250; a.y=2250; a.node=a.group=a.item=a.grabs=a.puts=0U;
        a.heading=0U; a.turning=false; a.dx=a.dy=0;
        char query[40]; snprintf(query,sizeof(query),"QUERY,%lu\n",(unsigned long)a.pending); a.stm(query);
        emit("STARTING","S1_HEAD_DOWN"); return true;
    }
    emit("ERROR","COMMAND"); return true;
}
static void request_camera(TaskVisionKind kind,unsigned target,uint32_t now)
{
    a.pending=next_id(); if(!a.pending) return;
    if(kind==TASK_QR) snprintf(a.camera_line,sizeof(a.camera_line),"CAM,%lu,QR\n",(unsigned long)a.pending);
    else if(kind==TASK_MAT) snprintf(a.camera_line,sizeof(a.camera_line),"CAM,%lu,MAT,%u,%.1f,%.1f\n",
        (unsigned long)a.pending,target,(double)a.target_x,(double)a.target_y);
    else snprintf(a.camera_line,sizeof(a.camera_line),"CAM,%lu,RING,%u\n",(unsigned long)a.pending,target);
    a.since=a.last_query=now; a.stm(a.camera_line);
}
static unsigned batch(void) { return a.group>=4U?1U:0U; }
static unsigned role(void)
{
    static const unsigned roles[]={0,2,1,3,0,2,1,4}; return roles[a.group];
}
static unsigned color(void) { return a.code.colors[batch()][a.item]; }
static unsigned ring_id(void)
{
    if(role()==4U) {
        for(unsigned i=0;i<3U;++i) if(a.code.colors[0][i]==color()) return a.code.places[0][i];
        return 0U;
    }
    return a.code.places[batch()][a.item];
}
static unsigned visual_target(TaskVisionKind *kind)
{
    if(role()==0U||(a.arm_enabled&&(role()==1U||role()==4U))) { *kind=TASK_MAT; return color(); }
    *kind=TASK_RING; unsigned id=ring_id(); const char *map=(role()==1U||role()==2U)?a.map_p:a.map_t;
    for(unsigned i=0;i<3U;++i) if(map[i]==(char)('0'+id)) return i+1U;
    return 0U;
}
static void begin_observation(uint32_t now)
{
    TaskVisionKind kind; unsigned target=visual_target(&kind);
    if(!target) { AutoTask_Stop("TARGET_MAPPING"); return; }
    a.phase=OBSERVE; request_camera(kind,target,now);
}
static void enter_node(uint32_t now)
{
    if(a.node==2U&&a.vision_enabled) { a.phase=QR; request_camera(TASK_QR,0U,now); emit("SCANNING","0"); return; }
    if(work_node()) {
        if(a.vision_enabled||a.arm_enabled) {
            a.phase=PREPARE; a.item=0U; a.corrections=0U; emit(a.arm_enabled?"WORK":"CALIBRATING","0"); return;
        }
    }
    if(++a.node>=sizeof(route)/sizeof(route[0])) {
        camera_idle();
        a.running=false; a.phase=OFF; emit("DONE","S2");
    }
}
void AutoTask_Input(const char *line,uint32_t now)
{
    if(!a.running) return;
    if(strncmp(line,"STATUS,",7U)==0) {
        const char *p=line+7U; uint32_t id;
        if(!RobotProtocol_U32(&p,&id,',')||id!=a.pending) return;
        if(a.phase==READY) {
            uint32_t turn;
            if(strncmp(p,"IDLE,1,",7U)==0||(!a.arm_enabled&&strncmp(p,"IDLE,0,",7U)==0)) {
                p+=7U;
                if(RobotProtocol_U32(&p,&turn,'\0')&&turn<=270U) {
                    a.turn_angle=(uint16_t)turn; a.phase=REFERENCE;
                    a.settle_until=now+500U; emit("RUNNING","0");
                    if(a.have_code) {
                        char event[64]; snprintf(event,sizeof(event),"TASK,%lu,%s\n",(unsigned long)a.run,a.manual_code); a.event(event);
                    }
                    return;
                }
            }
            AutoTask_Stop(a.arm_enabled?"STM_NOT_READY_INITIALIZE_ORIGIN":"STM_NOT_IDLE");
        }
        return;
    }
    if(strncmp(line,"ACTION,",7U)==0) {
        const char *p=line+7U; uint32_t id;
        if(!RobotProtocol_U32(&p,&id,',')||id!=a.pending||!a.waiting) return;
        if(strcmp(p,"DONE,0")==0) {
            a.waiting=false;
            if(a.turning) { a.heading=a.pending_heading; a.turning=false; }
            if(a.phase==NAV||a.phase==CORRECT) { a.x+=a.dx; a.y+=a.dy; a.dx=a.dy=0; }
            if(a.phase==CARD) {
                ++a.grabs; ++a.puts; emit("WORK","CARD_DONE");
            }
            a.phase=a.next; a.settle_until=now+500U;
            if(a.phase==OBSERVE) a.pending=0U;
        } else if(strncmp(p,"ERROR,",6U)==0||strncmp(p,"CANCELLED,",10U)==0||strncmp(p,"BUSY,",5U)==0) {
            AutoTask_Stop(strncmp(p,"ERROR,",6U)==0?p+6U:"STM_ACTION_FAILED");
        }
        return;
    }
    TaskVision v;
    if(!TaskProtocol_Vision(line,&v)||v.request!=a.pending||a.waiting) return;
    if(a.phase==QR&&v.kind==TASK_QR) {
        a.code=v.code; a.have_code=true;
        char event[64]; snprintf(event,sizeof(event),"TASK,%lu,%s\n",(unsigned long)a.run,v.text); a.event(event);
        ++a.node; a.phase=NAV; a.settle_until=now; return;
    }
    if(a.phase!=OBSERVE) return;
    TaskVisionKind kind; unsigned target=visual_target(&kind);
    if(v.kind!=kind||v.target!=target) return;
    float ex=v.center.x-a.target_x, ey=v.center.y-a.target_y;
    if(fabsf(ex)<=1.0f&&fabsf(ey)<=1.0f) { a.phase=a.arm_enabled?CARD:HOLD; a.settle_until=now; return; }
    if(fabsf(ex)*1.090819f>200.0f||fabsf(ey)*1.090819f>200.0f||++a.corrections>20U) {
        AutoTask_Stop("VISION_OUT_OF_APPROACH_RANGE"); return;
    }
    /* Same installed image-to-wheel mapping as the existing STM32 alignment.
     * Reacquire after each axis; never use stale Y after moving X. */
    bool x=fabsf(ex)>1.0f;
    float mm=(x?ex:ey)*1.090819f;
    mm=fmaxf(-25.0f,fminf(25.0f,mm));
    char positive=x?CAMERA_IMAGE_RIGHT_DIRECTION:CAMERA_IMAGE_DOWN_DIRECTION;
    char dir=mm>0?positive:CameraProtocol_Opposite(positive);
    uint32_t ppm=a.forward;
    uint32_t pulses=(uint32_t)lroundf(fabsf(mm)*ppm/1000.0f); if(!pulses) pulses=1U;
    float travel=pulses*1000.0f/ppm;
    /* Observations are taken only at heading 0, preserving camera/arm calibration.
     * Store a world-axis correction, then turn and advance instead of strafing. */
    a.correction_dx=(dir=='L'?travel:dir=='R'?-travel:0);
    a.correction_dy=(dir=='B'?travel:dir=='F'?-travel:0);
    if(!clear_segment(a.x,a.y,a.correction_dx,a.correction_dy)) { AutoTask_Stop("LANE_BOUNDARY"); return; }
    a.correction_heading=dir=='F'?0U:dir=='R'?1U:dir=='B'?2U:3U;
    a.correction_pulses=pulses; a.phase=CORRECT; a.settle_until=now;
}
static void advance_card(void)
{
    a.corrections=0U;
    if(++a.item<3U) { a.phase=PREPARE; return; }
    a.item=0U;
    ++a.group;
    /* Process placement -> retrieval at the same dock, all three first. */
    if(a.group==2U||a.group==6U) { a.phase=PREPARE; return; }
    ++a.node; a.phase=a.arm_enabled?STOW:NAV;
}
void AutoTask_Tick(uint32_t now)
{
    if(!a.running) return;
    if(a.waiting||a.phase==READY) {
        if((uint32_t)(now-a.since)>155000U) { AutoTask_Stop("ACTION_TIMEOUT"); return; }
        if((uint32_t)(now-a.last_query)>=1000U) {
            char q[40]; snprintf(q,sizeof(q),"QUERY,%lu\n",(unsigned long)a.pending); a.stm(q); a.last_query=now;
        }
        return;
    }
    if((int32_t)(now-a.settle_until)<0) return;
    char body[160];
    switch(a.phase) {
    case REFERENCE:
        action("IMU_ZERO",a.arm_enabled?STOW:NAV,now); break;
    case STOW:
        if(!a.arm_enabled) { a.phase=NAV; break; }
        snprintf(body,sizeof(body),"ARM_POSE,0,0,0,20,3200");
        action(body,NAV,now); break;
    case NAV: {
        Point target=a.route[a.node];
        float dx=target.x-a.x, dy=target.y-a.y;
        /* User-authorized departure exception; custom nodes use normal turns. */
        if(a.node==1U&&a.heading==0U&&fabsf(a.x-2250.0f)<0.5f&&fabsf(a.y-2200.0f)<0.5f&&
            target.x==2200.0f&&target.y==2200.0f) {
            if(!clear_segment(a.x,a.y,-50.0f,0)) { AutoTask_Stop("LANE_BOUNDARY"); break; }
            a.dx=-50.0f; a.dy=0;
            snprintf(body,sizeof(body),"MOVE,R,50,%lu,%lu",(unsigned long)a.rpm,(unsigned long)a.lateral);
            emit("RUNNING","0"); action(body,NAV,now); break;
        }
        if(fabsf(dx)<0.5f&&fabsf(dy)<0.5f) {
            bool dock=(a.node==2U&&a.vision_enabled)||((a.vision_enabled||a.arm_enabled)&&work_node());
            if(dock&&turn_to(0U,NAV,now)) break;
            enter_node(now); break;
        }
        /* Explicit axis segments; first point keeps S1 departure forward. */
        bool y=fabsf(dy)>=0.5f;
        if(y&&!clear_segment(a.x,a.y,0,dy)) y=false;
        if(!y&&(fabsf(dx)<0.5f||!clear_segment(a.x,a.y,dx,0))) {
            AutoTask_Stop("LANE_BOUNDARY"); break;
        }
        int mm=(int)lroundf(fabsf(y?dy:dx)); if(mm<1) mm=1;
        unsigned heading=y?(dy>0?2U:0U):(dx>0?3U:1U);
        if(turn_to(heading,NAV,now)) break;
        a.dx=y?0:(dx>0?mm:-mm); a.dy=y?(dy>0?mm:-mm):0;
        uint32_t pulses=(uint32_t)(((uint64_t)mm*a.forward+500U)/1000U);
        if(!pulses) { AutoTask_Stop("DRIVE_DISTANCE"); break; }
        snprintf(body,sizeof(body),"DRIVE,F,%lu,%lu,%lu",(unsigned long)pulses,(unsigned long)a.rpm,(unsigned long)a.wheel_ppr);
        emit("RUNNING","0");
        action(body,NAV,now); break;
    }
    case CORRECT:
        if(turn_to(a.correction_heading,CORRECT,now)) break;
        a.dx=a.correction_dx; a.dy=a.correction_dy;
        snprintf(body,sizeof(body),"DRIVE,F,%lu,%lu,%lu",(unsigned long)a.correction_pulses,(unsigned long)a.fine,(unsigned long)a.wheel_ppr);
        action(body,RESTORE,now); break;
    case RESTORE:
        if(turn_to(0U,RESTORE,now)) break;
        begin_observation(now); break;
    case QR: case OBSERVE:
        if(a.phase==OBSERVE&&a.pending==0U) { begin_observation(now); break; }
        if((uint32_t)(now-a.since)>60000U) { AutoTask_Stop("VISION_TIMEOUT"); break; }
        /* Camera keeps streaming the requested mode. A fresh request after a
         * correction invalidates the previous frame and restarts stability. */
        if((uint32_t)(now-a.last_query)>2000U) { a.stm(a.camera_line); a.last_query=now; }
        break;
    case PREPARE: {
        if(!a.arm_enabled) { begin_observation(now); break; }
        RobotSequenceCommand *profile=&a.profiles[role()][a.item];
        snprintf(body,sizeof(body),"BASE,%lu,60",(unsigned long)profile->base_home);
        action(body,SLOT,now); break;
    }
    case SLOT:
        snprintf(body,sizeof(body),"STATE,T,%u,%u,60",a.turn_angle,a.slots[a.item]);
        a.turn_angle=a.slots[a.item];
        action(body,a.vision_enabled?OBSERVE:CARD,now); break;
    case CARD: {
        RobotSequenceCommand *s=&a.profiles[role()][a.item];
        snprintf(body,sizeof(body),"STATE_V2,%c,%ld,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
            s->mode,(long)s->r1,(long)s->z1,(long)s->r2,(long)s->z2,(unsigned long)s->combined_rpm,
            (unsigned long)s->gripper_dps,(unsigned long)s->theta,(unsigned long)s->open_angle,
            (unsigned long)s->close_angle,(unsigned long)s->base_home,(unsigned long)s->base_tilt);
        action(body,HOLD,now); break;
    }
    case HOLD: advance_card(); break;
    default: break;
    }
}
