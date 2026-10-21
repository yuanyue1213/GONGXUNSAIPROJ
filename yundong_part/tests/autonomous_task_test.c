#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../esp32s3/hello_world/main/autonomous_task.h"
static char outgoing[256],events[160];
static unsigned moves,cards,qr,mat,ring,done,stops,heading,references,detects,route_rpm;
static bool vision_enabled,arm_enabled;
/* Independent trace of the user's body-coordinate instructions. */
static const struct { char dir; unsigned amount; } expected[]={
    {'L',100},{'B',1000},{'F',900},{'C',90},{'F',1000},
    {'C',90},{'F',1900},{'C',90},{'B',900},{'W',90},
    {'B',900},{'B',900},{'W',90},{'B',900},{'C',90},
    {'F',1900},{'C',90},{'B',900},{'W',90},{'B',900},
    {'C',90},{'F',1900},{'W',90},{'F',1000},{'L',100}};
static void send_line(const char *s)
{
    assert(!strstr(s,",HEADING,")&&!strstr(s,",DRIVE,")&&!strstr(s,",MOVE,"));
    snprintf(outgoing,sizeof(outgoing),"%s",s);
    if(strncmp(s,"CMD,",4U)==0) ++stops;
    if(strncmp(s,"CAM,",4U)==0&&!strstr(s,",IDLE")) ++detects;
}
static void event(const char *s) { snprintf(events,sizeof(events),"%s",s); if(strstr(s,",DONE,")) ++done; }
static void setup(bool vision,bool arm,unsigned rpm)
{
    vision_enabled=vision; arm_enabled=arm; route_rpm=rpm;
    moves=cards=qr=mat=ring=done=stops=heading=references=detects=0;
    outgoing[0]=events[0]=0; AutoTask_Init(send_line,event); char line[192];
    snprintf(line,sizeof(line),"AUTO_CFG,10,10000,20000,%u,10,256.0,160.0,321,321,0,90,180",rpm); AutoTask_Command(line,0);
    AutoTask_Command("AUTO_DRIVE,10,1000,40,3200",0); AutoTask_Command("AUTO_PATH,10,BODY_V1",0);
    snprintf(line,sizeof(line),"AUTO_OPTIONS,10,%u,%u,%s",vision,arm,!vision&&arm?"156+123+516+231":"-"); AutoTask_Command(line,0);
    if(arm) for(unsigned r=0;r<5U;++r) for(unsigned slot=1;slot<=3U;++slot) {
        snprintf(line,sizeof(line),"AUTO_ARM,10,%c,%u,STATE_V2,10,%c,1100,-50,200,-40,20,60,0,60,0,248,140",
            "RGPTK"[r],slot,r<2U?'A':'P'); AutoTask_Command(line,0);
    }
    assert(!strstr(events,",ERROR,")); AutoTask_Command("AUTO_START,20,10",0); assert(AutoTask_Running());
}
static void respond(uint32_t now)
{
    if(!outgoing[0]) return;
    char line[256],reply[160]; strcpy(line,outgoing); outgoing[0]=0; unsigned id,target;
    if(sscanf(line,"QUERY,%u",&id)==1) {
        snprintf(reply,sizeof(reply),"STATUS,%u,IDLE,%u,0",id,arm_enabled?1U:0U); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"EXEC,%u,",&id)==1) {
        const char *body=strchr(strchr(line,',')+1,',')+1; char dir; unsigned amount,rpm,ppr;
        if(strcmp(body,"IMU_ZERO\n")==0) { assert(!moves); ++references; }
        else if(sscanf(body,"TURN,%c,%u.0,%u",&dir,&amount,&rpm)==3) {
            assert(moves<25U&&expected[moves].dir==dir&&amount==90U&&rpm==40U);
            heading=(heading+(dir=='C'?1U:3U))%4U; ++moves;
        } else if(sscanf(body,"ROUTE_MOVE,%c,%u,%u,%u",&dir,&amount,&rpm,&ppr)==4) {
            assert(moves<25U&&expected[moves].dir==dir&&ppr==3200U&&rpm==route_rpm);
            assert(amount==expected[moves].amount*(dir=='L'||dir=='R'?20U:10U)); ++moves;
        } else if(strncmp(body,"STATE_V2,",9U)==0) {
            static const unsigned docks[]={5,8,8,11,14,17,17,20}; static const char modes[]="APAPAPAP";
            assert(arm_enabled&&cards<24U&&moves==docks[cards/3U]&&body[9]==modes[cards/3U]); ++cards;
        } else assert(arm_enabled);
        snprintf(reply,sizeof(reply),"ACTION,%u,DONE,0",id); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,QR",&id)==1&&strstr(line,",QR")) {
        assert(vision_enabled&&moves==2U&&heading==0U); ++qr;
        snprintf(reply,sizeof(reply),"VISION,%u,QR,156+123+516+231",id); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,MAT,%u",&id,&target)==2) {
        static const unsigned dock[]={5,8,14,17,20},angles[]={1,3,1,3,2};
        unsigned index=mat/3U; if(arm_enabled) assert(index<5U&&moves==dock[index]&&heading==angles[index]); ++mat;
        snprintf(reply,sizeof(reply),"VISION,%u,MAT,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,RING,%u",&id,&target)==2) {
        static const unsigned dock[]={8,11,17},angles[]={3,2,3}; unsigned index=ring/3U;
        if(arm_enabled) assert(index<3U&&moves==dock[index]&&heading==angles[index]); ++ring;
        snprintf(reply,sizeof(reply),"VISION,%u,RING,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
}
static void test_four_modes(void)
{
    for(unsigned vision=0;vision<2U;++vision) for(unsigned arm=0;arm<2U;++arm) {
        setup(vision!=0,arm!=0,vision&&arm?1200U:45U);
        for(uint32_t now=0;now<200000U&&AutoTask_Running();now+=200U) { respond(now); AutoTask_Tick(now); }
        assert(!AutoTask_Running()&&done==1U&&moves==25U&&heading==2U&&references==1U);
        assert(cards==(arm?24U:0U)&&qr==(vision?1U:0U)); assert(vision||detects==0U);
        if(vision&&arm) assert(mat==15U&&ring==9U);
    }
}
static void test_interval_and_stale_completion(void)
{
    setup(false,false,45); uint32_t now;
    for(now=0;now<10000U;now+=200U) { if(strstr(outgoing,",ROUTE_MOVE,L,")) break; respond(now); AutoTask_Tick(now); }
    assert(now<10000U); unsigned id; assert(sscanf(outgoing,"EXEC,%u,",&id)==1); outgoing[0]=0;
    char line[80]; snprintf(line,sizeof(line),"ACTION,%u,DONE,0",id-1U); AutoTask_Input(line,now);
    AutoTask_Tick(now+99U); assert(!outgoing[0]);
    snprintf(line,sizeof(line),"ACTION,%u,DONE,0",id); AutoTask_Input(line,now);
    AutoTask_Tick(now+99U); assert(!outgoing[0]); AutoTask_Tick(now+100U); AutoTask_Tick(now+101U);
    assert(strstr(outgoing,",ROUTE_MOVE,B,10000,45,3200"));
    unsigned before=stops; AutoTask_Command("AUTO_STOP,1",now+102U); assert(!AutoTask_Running()&&stops>before);
}
static void test_camera_corrections(void)
{
    static const float sample[][2]={{253.9f,160},{258.1f,160},{256,162.1f},{256,157.9f}};
    static const char expected_dir[]={'F','B','L','R'};
    for(unsigned c=0;c<4U;++c) {
        setup(true,true,45); uint32_t now; unsigned request,target;
        for(now=0;now<30000U;now+=200U) { if(sscanf(outgoing,"CAM,%u,MAT,%u",&request,&target)==2) break; respond(now); AutoTask_Tick(now); }
        assert(now<30000U&&moves==5U&&heading==1U); outgoing[0]=0;
        char reply[160]; snprintf(reply,sizeof(reply),"VISION,%u,MAT,%u,%.1f,%.1f",request,target,(double)sample[c][0],(double)sample[c][1]);
        AutoTask_Input(reply,now); AutoTask_Tick(now); unsigned id,pulses,rpm,ppr; char dir;
        assert(sscanf(outgoing,"EXEC,%u,ROUTE_MOVE,%c,%u,%u,%u",&id,&dir,&pulses,&rpm,&ppr)==5);
        assert(dir==expected_dir[c]&&rpm==10U&&ppr==3200U&&pulses==(c<2U?23U:46U)); outgoing[0]=0;
        snprintf(reply,sizeof(reply),"ACTION,%u,DONE,0",id); AutoTask_Input(reply,now); AutoTask_Tick(now+100U);
        assert(strstr(outgoing,",MAT,")&&heading==1U); AutoTask_Stop("TEST");
    }
}
int main(void) { test_four_modes(); test_interval_and_stale_completion(); test_camera_corrections();
    puts("PASS: exact 25 body steps, six docks, both process retrievals, four modes, 1200RPM, 100ms gap, stale IDs, body-axis vision"); }
