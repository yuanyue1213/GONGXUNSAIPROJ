#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../esp32s3/hello_world/main/autonomous_task.h"
static char outgoing[256],events[160];
static unsigned moves,cards,qr,mat,ring,tables,occupied,done,stops,heading,references,detects,route_rpm,raw_preps,raw_waits,actual_turn;
static bool vision_enabled,arm_enabled,preset_enabled,task_announced;
static const unsigned physical[8][3]={{2,2,2},{3,2,1},{2,3,1},{3,2,1},{2,2,2},{2,1,3},{3,2,1},{2,3,1}};
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
static void event(const char *s) { snprintf(events,sizeof(events),"%s",s); if(strstr(s,",DONE,")) ++done;
    if(strncmp(s,"TASK,",5U)==0) { assert(strstr(s,"156+123+516+231")); task_announced=true; } }
static void setup_code(bool vision,bool arm,unsigned rpm,bool preset)
{
    vision_enabled=vision; arm_enabled=arm; route_rpm=rpm; preset_enabled=preset; task_announced=false;
    moves=cards=qr=mat=ring=tables=occupied=done=stops=heading=references=detects=0;
    actual_turn=raw_preps=raw_waits=0;
    outgoing[0]=events[0]=0; AutoTask_Init(send_line,event); char line[192];
    snprintf(line,sizeof(line),"AUTO_CFG,10,10000,20000,%u,10,256.0,160.0,321,321,0,90,180",rpm); AutoTask_Command(line,0);
    AutoTask_Command("AUTO_DRIVE,10,1000,40,3200",0);
    AutoTask_Command(preset?"AUTO_PATH,10,BODY_NEAR_DEFAULT_V1":"AUTO_PATH,10,BODY_NEAR_V3",0);
    snprintf(line,sizeof(line),"AUTO_OPTIONS,10,%u,%u,%s",vision,arm,(preset||(!vision&&arm))?"156+123+516+231":"-"); AutoTask_Command(line,0);
    if(arm) for(unsigned r=0;r<5U;++r) for(unsigned slot=1;slot<=3U;++slot) {
        snprintf(line,sizeof(line),"AUTO_ARM,10,%c,%u,STATE_V2,10,%c,%u,-50,200,-40,20,60,0,60,0,%u,140",
            "RGPTK"[r],slot,r<2U?'A':'P',1100+slot*10,240+slot); AutoTask_Command(line,0);
    }
    assert(!strstr(events,",ERROR,")); AutoTask_Command("AUTO_START,20,10",0); assert(AutoTask_Running());
}
static void setup(bool vision,bool arm,unsigned rpm) { setup_code(vision,arm,rpm,false); }
static void respond(uint32_t now)
{
    if(!outgoing[0]) return;
    char line[256],reply[160]; strcpy(line,outgoing); outgoing[0]=0; unsigned id,target;
    if(sscanf(line,"QUERY,%u",&id)==1) {
        snprintf(reply,sizeof(reply),"STATUS,%u,IDLE,%u,0",id,arm_enabled?1U:0U); AutoTask_Input(reply,now);
        if(preset_enabled) {
            assert(task_announced);
            snprintf(reply,sizeof(reply),"VISION,%u,QR,123+123+321+231",id); AutoTask_Input(reply,now);
        }
        return;
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
        } else if(strncmp(body,"RAW_READY,",10U)==0) {
            assert(cards/3U==0U||cards/3U==4U);
            assert(raw_preps==raw_waits); int reach;
            assert(sscanf(body+12,"%d",&reach)==1&&reach==1120); ++raw_preps;
        } else if(strcmp(body,"RAW_TAKE\n")==0) {
            assert(raw_waits==raw_preps&&raw_waits>0U); ++cards;
        } else if(strncmp(body,"STATE_V2,",9U)==0) {
            static const unsigned docks[]={5,8,8,11,14,17,17,20}; static const char modes[]="APAPAPAP";
            assert(arm_enabled&&cards<24U&&moves==docks[cards/3U]&&body[9]==modes[cards/3U]);
            int reach; assert(sscanf(body+11,"%d",&reach)==1);
            unsigned pos=physical[cards/3U][cards%3U];
            if(!vision_enabled&&cards/3U==2U) pos=3U-cards%3U;
            if(!vision_enabled&&cards/3U==6U) { const unsigned second[]={2,1,3}; pos=second[cards%3U]; }
            assert(reach==(int)(1100+pos*10)); ++cards;
        } else if(strncmp(body,"STATE,T,",8U)==0) {
            unsigned previous,slot_angle,speed;
            assert(sscanf(body,"STATE,T,%u,%u,%u",&previous,&slot_angle,&speed)==3);
            assert(previous==actual_turn); actual_turn=slot_angle;
            assert(slot_angle==(cards%3U)*90U); /* onboard index is independent */
        } else if(strncmp(body,"ARM_POSE,",9U)==0) { assert(arm_enabled); actual_turn=0U; }
        else assert(arm_enabled);
        snprintf(reply,sizeof(reply),"ACTION,%u,DONE,0",id); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,QR",&id)==1&&strstr(line,",QR")) {
        assert(!preset_enabled&&vision_enabled&&moves==2U&&heading==0U); ++qr;
        snprintf(reply,sizeof(reply),"VISION,%u,QR,156+123+516+231",id); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,TABLE,%u",&id,&target)==2) {
        assert(tables<2U&&moves==(tables?14U:5U)&&target==(tables?516U:156U));
        assert(!arm_enabled||cards==(tables?12U:0U)); ++tables;
        snprintf(reply,sizeof(reply),"VISION,%u,TABLE,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,OCCUPIED,%u",&id,&target)==2) {
        static const unsigned dock[]={8,17,20},angles[]={3,3,2};
        static const unsigned before_cards[]={6,18,21};
        assert(occupied<3U&&moves==dock[occupied]&&heading==angles[occupied]);
        assert(target==(occupied?516U:156U));
        if(arm_enabled) assert(cards==before_cards[occupied]);
        ++occupied;
        snprintf(reply,sizeof(reply),"VISION,%u,OCCUPIED,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,NEAR,%u",&id,&target)==2) {
        static const unsigned colors[2][3]={{1,5,6},{5,1,6}};
        assert(arm_enabled&&vision_enabled&&(cards/3U==0U||cards/3U==4U));
        assert(target==colors[cards>=12U][cards%3U]&&raw_preps==raw_waits+1U);
        snprintf(reply,sizeof(reply),"VISION,%u,NEAR,%u,2",id-1U,target); AutoTask_Input(reply,now);
        snprintf(reply,sizeof(reply),"VISION,%u,NEAR,%u,2",id,target==1U?2U:1U); AutoTask_Input(reply,now);
        snprintf(reply,sizeof(reply),"VISION,%u,PICK,%u,2",id,target); AutoTask_Input(reply,now);
        snprintf(reply,sizeof(reply),"VISION,%u,NEAR,%u,1",id,target); AutoTask_Input(reply,now);
        AutoTask_Tick(now); assert(!outgoing[0]); /* Preparation alone never closes. */
        ++raw_waits;
        snprintf(reply,sizeof(reply),"VISION,%u,NEAR,%u,2",id,target); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,PICK,%u",&id,&target)==2) {
        assert(cards/3U!=0U&&cards/3U!=4U);
        static const unsigned colors[2][3]={{1,5,6},{5,1,6}};
        assert(arm_enabled&&vision_enabled&&target==colors[cards>=12U][cards%3U]);
        unsigned pos=physical[cards/3U][cards%3U];
        snprintf(reply,sizeof(reply),"VISION,%u,PICK,%u,%u",id-1U,target,pos); AutoTask_Input(reply,now);
        snprintf(reply,sizeof(reply),"VISION,%u,PICK,%u,%u",id,target==1U?2U:1U,pos); AutoTask_Input(reply,now);
        AutoTask_Tick(now); assert(!outgoing[0]); /* stale/wrong color cannot select a card */
        snprintf(reply,sizeof(reply),"VISION,%u,PICK,%u,%u",id,target,pos); AutoTask_Input(reply,now); return;
    }
    assert(!strstr(line,",MAT,")); /* Never follow the pickup color for occupied-ring calibration. */
    if(sscanf(line,"CAM,%u,RING,%u",&id,&target)==2) {
        static const unsigned dock[]={8,11,17},angles[]={3,2,3}; unsigned index=ring; assert(target==2U);
        if(arm_enabled) assert(index<3U&&moves==dock[index]&&heading==angles[index]); ++ring;
        snprintf(reply,sizeof(reply),"VISION,%u,RING,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
}
static void test_supported_modes(void)
{
    for(unsigned preset=0;preset<2U;++preset) for(unsigned vision=0;vision<2U;++vision) for(unsigned arm=0;arm<2U;++arm) {
        setup_code(vision!=0,arm!=0,vision&&arm?1200U:45U,preset!=0);
        for(uint32_t now=0;now<200000U&&AutoTask_Running();now+=200U) { respond(now); AutoTask_Tick(now); }
        assert(!AutoTask_Running()&&done==1U&&moves==25U&&heading==2U&&references==1U);
        assert(cards==(arm?24U:0U)&&qr==(vision&&!preset?1U:0U)); assert(vision||detects==0U);
        assert(tables==(vision?2U:0U)); if(preset||vision||arm) assert(task_announced);
        assert(occupied==(vision?3U:0U)&&mat==0U);
        if(vision&&arm) assert(ring==3U&&raw_preps==6U&&raw_waits==6U);
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
    assert(strstr(events,"DRIVE:WAIT_ACTION"));
    unsigned active; assert(sscanf(outgoing,"EXEC,%u,",&active)==1);
    snprintf(line,sizeof(line),"ACTION,%u,ERROR,MOTOR_FAULT",active);
    AutoTask_Input(line,now+101U);
    assert(!AutoTask_Running()&&strstr(events,"DRIVE:MOTOR_FAULT"));
    unsigned before=stops; AutoTask_Command("AUTO_STOP,1",now+102U); assert(!AutoTask_Running()&&stops==before);
}
static void test_camera_corrections(void)
{
    static const float sample[][2]={{253.9f,160},{258.1f,160},{256,162.1f},{256,157.9f}};
    static const char expected_dir[]={'F','B','L','R'};
    for(unsigned c=0;c<4U;++c) {
        setup(true,true,45); uint32_t now; unsigned request,target;
        for(now=0;now<30000U;now+=200U) { if(sscanf(outgoing,"CAM,%u,TABLE,%u",&request,&target)==2) break; respond(now); AutoTask_Tick(now); }
        assert(now<30000U&&moves==5U&&heading==1U); outgoing[0]=0;
        char reply[160];
        snprintf(reply,sizeof(reply),"VISION,%u,MAT,1,256.0,160.0",request); AutoTask_Input(reply,now);
        snprintf(reply,sizeof(reply),"VISION,%u,TABLE,516,256.0,160.0",request); AutoTask_Input(reply,now);
        snprintf(reply,sizeof(reply),"VISION,%u,TABLE,%u,256.0,160.0",request-1U,target); AutoTask_Input(reply,now);
        snprintf(reply,sizeof(reply),"VISION,%u,NONE",request); AutoTask_Input(reply,now);
        AutoTask_Tick(now); assert(!outgoing[0]&&cards==0U); /* No single/stale/wrong-batch fallback. */
        snprintf(reply,sizeof(reply),"VISION,%u,TABLE,%u,%.1f,%.1f",request,target,(double)sample[c][0],(double)sample[c][1]);
        AutoTask_Input(reply,now); AutoTask_Tick(now); unsigned id,pulses,rpm,ppr; char dir;
        assert(sscanf(outgoing,"EXEC,%u,ROUTE_MOVE,%c,%u,%u,%u",&id,&dir,&pulses,&rpm,&ppr)==5);
        assert(dir==expected_dir[c]&&rpm==10U&&ppr==3200U&&pulses==(c<2U?23U:46U)); outgoing[0]=0;
        snprintf(reply,sizeof(reply),"ACTION,%u,DONE,0",id); AutoTask_Input(reply,now); AutoTask_Tick(now+100U);
        assert(strstr(outgoing,",TABLE,")&&heading==1U); AutoTask_Stop("TEST");
    }
}
static void test_raw_wait_timeout_and_stop(void)
{
    setup(true,true,45); uint32_t now; unsigned id,color;
    for(now=0;now<30000U;now+=200U) {
        if(sscanf(outgoing,"CAM,%u,NEAR,%u",&id,&color)==2) break;
        respond(now); AutoTask_Tick(now);
    }
    assert(now<30000U&&cards==0U&&raw_preps==1U);
    outgoing[0]=0; AutoTask_Tick(now+60001U);
    assert(!AutoTask_Running()&&strstr(events,"RAW_WAIT:RAW_TARGET_TIMEOUT")&&stops>=4U);
}
int main(void) { test_supported_modes(); test_raw_wait_timeout_and_stop(); test_interval_and_stale_completion(); test_camera_corrections();
    puts("PASS: exact 25 body steps, six docks, both process retrievals, four supported modes, fixed physical profiles, independent onboard slots, 1200RPM, 100ms gap, stale IDs, body-axis vision"); }
