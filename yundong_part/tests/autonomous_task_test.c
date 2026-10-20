#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../esp32s3/hello_world/main/autonomous_task.h"
static char outgoing[256],events[128];
static unsigned commands,cards,qr,mat,ring,stops,done;
static float x=2250,y=2250;
static unsigned moves;
static unsigned heading,turns;
static void send_line(const char *s) { snprintf(outgoing,sizeof(outgoing),"%s",s); ++commands; if(strncmp(s,"CMD,",4)==0) ++stops; }
static void event(const char *s) { snprintf(events,sizeof(events),"%s",s); if(strstr(s,",DONE,")) ++done; }
static void config(void)
{
    heading=turns=0U;
    assert(AutoTask_Command("AUTO_CFG,10,10000,20000,45,10,256.0,160.0,321,321,0,90,180",0));
    assert(AutoTask_Command("AUTO_DRIVE,10,1000,20,3200",0));
    for(unsigned r=0;r<5;++r) for(unsigned slot=1;slot<=3;++slot) {
        char line[192]; snprintf(line,sizeof(line),"AUTO_PROFILE,10,%c,%u,STATE_V2,10,%c,1100,-50,200,-40,20,60,0,60,0,248,140","RGPTK"[r],slot,r<2?'A':'P');
        assert(AutoTask_Command(line,0)); assert(!strstr(events,"ERROR"));
    }
}
static void respond(uint32_t now)
{
    if(!outgoing[0]) return;
    char s[256]; strcpy(s,outgoing); outgoing[0]=0;
    char reply[128]; unsigned id;
    if(sscanf(s,"QUERY,%u",&id)==1) {
        snprintf(reply,sizeof(reply),"STATUS,%u,IDLE,1,0",id); AutoTask_Input(reply,now); return;
    }
    if(sscanf(s,"EXEC,%u,",&id)==1) {
        char *body=strchr(strchr(s,',')+1,',')+1;
        unsigned angle,turn_rpm;
        if(strcmp(body,"IMU_ZERO\n")==0) heading=0U;
        if(sscanf(body,"HEADING,%u.0,%u",&angle,&turn_rpm)==2) {
            assert(angle<360U&&angle%90U==0U&&turn_rpm==20U);
            heading=angle/90U; ++turns;
        }
        char dir; unsigned distance,rpm,ppm;
        bool drive=sscanf(body,"DRIVE,%c,%u,%u,%u",&dir,&distance,&rpm,&ppm)==4;
        if(drive) { assert(ppm==3200U); distance/=10U; ppm=10000U; }
        if(drive||sscanf(body,"MOVE,%c,%u,%u,%u",&dir,&distance,&rpm,&ppm)==4) {
            if(dir=='C'||dir=='W') {
                assert(false); // Automatic turns must use gyro HEADING, never pulse rotation.
            } else {
            if(moves==1U) {
                assert(!drive&&dir=='R'&&distance==50U&&ppm==20000U&&heading==0U&&turns==0U);
                x-=distance; ++moves;
            } else {
            assert(drive&&dir=='F'&&ppm==10000U);
            if(moves==0U) assert(dir=='F'&&distance==50U);
            if(moves==2U) assert(dir=='F'&&distance==1000U);
            ++moves;
            if(heading==0U) y-=distance;
            if(heading==1U) x-=distance;
            if(heading==2U) y+=distance;
            if(heading==3U) x+=distance;
            }
            }
        }
        if(strncmp(body,"STATE_V2,",9)==0) {
            ++cards;
        }
        snprintf(reply,sizeof(reply),"ACTION,%u,DONE,0",id); AutoTask_Input(reply,now); return;
    }
    unsigned target;
    if(sscanf(s,"CAM,%u,MAT,%u",&id,&target)==2) {
        assert(heading==0U);
        static const unsigned expected[]={1,5,6,1,5,6,5,1,6,5,1,6,5,1,6};
        assert(mat<sizeof(expected)/sizeof(expected[0])); assert(target==expected[mat]); ++mat;
        snprintf(reply,sizeof(reply),"VISION,%u,MAT,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
    if(sscanf(s,"CAM,%u,RING,%u",&id,&target)==2) {
        assert(heading==0U);
        /* Frame order 321; first storage uses 123, second process uses 231. */
        static const unsigned expected[]={3,2,1,3,2,1,2,1,3};
        assert(ring<sizeof(expected)/sizeof(expected[0])); assert(target==expected[ring]); ++ring;
        snprintf(reply,sizeof(reply),"VISION,%u,RING,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
    if(sscanf(s,"CAM,%u,QR",&id)==1) {
        assert(x==2200&&y==1200&&heading==0U); ++qr;
        snprintf(reply,sizeof(reply),"VISION,%u,QR,156+123+516+231",id); AutoTask_Input(reply,now);
    }
}
static void test_forward_corrections(void)
{
    static const float samples[4][2]={{253.9f,160.0f},{258.1f,160.0f},{256.0f,162.1f},{256.0f,157.9f}};
    static const unsigned desired[4]={0U,2U,3U,1U};
    for(unsigned c=0;c<4U;++c) {
        AutoTask_Init(send_line,event); config(); x=y=2250; mat=ring=cards=qr=moves=heading=0;
        AutoTask_Command("AUTO_START,60,10",0);
        uint32_t now; unsigned request,target,id;
        for(now=0;now<40000U;now+=600U) {
            if(sscanf(outgoing,"CAM,%u,MAT,%u",&request,&target)==2) break;
            respond(now); AutoTask_Tick(now);
        }
        assert(now<40000U&&heading==0U);
        char reply[128]; snprintf(reply,sizeof(reply),"VISION,%u,MAT,%u,%.1f,%.1f",request,target,
            (double)samples[c][0],(double)samples[c][1]);
        AutoTask_Input(reply,now); AutoTask_Tick(now);
        if(desired[c]!=0U) {
            assert(sscanf(outgoing,"EXEC,%u,",&id)==1&&strstr(outgoing,",HEADING,")!=NULL);
            unsigned before=commands;
            AutoTask_Tick(now+500U); assert(commands==before); // No forward before turn DONE.
            snprintf(reply,sizeof(reply),"ACTION,%u,DONE,0",id-1U);
            AutoTask_Input(reply,now+500U); assert(commands==before);
            respond(now+600U); AutoTask_Tick(now+1200U); now+=1200U;
        }
        unsigned pulses,rpm; char dir;
        assert(sscanf(outgoing,"EXEC,%u,DRIVE,%c,%u,%u",&id,&dir,&pulses,&rpm)==4);
        assert(dir=='F'&&pulses==23U&&rpm==10U&&heading==desired[c]);
        respond(now); AutoTask_Tick(now+600U); now+=600U;
        if(desired[c]!=0U) {
            assert(strstr(outgoing,",HEADING,")!=NULL);
            respond(now); AutoTask_Tick(now+600U);
        }
        unsigned fresh;
        assert(sscanf(outgoing,"CAM,%u,MAT,%u",&fresh,&target)==2&&fresh>request&&heading==0U);
        AutoTask_Stop("TEST");
    }
}
int main(void)
{
    TaskCode code;
    assert(TaskProtocol_Code("156+123+516+231",&code));
    assert(!TaskProtocol_Code("156+123+234+231",&code));
    assert(!TaskProtocol_Code("116+123+611+231",&code));
    TaskVision v;
    assert(TaskProtocol_Vision("VISION,1,MAT,5,111.2,160.7",&v));
    assert(!TaskProtocol_Vision("VISION,1,MAT,5,512.0,160.7",&v));
    AutoTask_Init(send_line,event); config();
    AutoTask_Command("AUTO_START,20,10",0); assert(AutoTask_Running());
    for(uint32_t now=0;now<300000&&AutoTask_Running();now+=600) {
        respond(now); AutoTask_Tick(now);
    }
    assert(!AutoTask_Running()); assert(done==1); assert(qr==1&&cards==24&&mat==15&&ring==9);
    assert(x==2250&&y==150&&turns>0U);
    assert(strstr(events,",24,24,S2"));
    AutoTask_Command("AUTO_START,20,10",300001); assert(!AutoTask_Running());
    /* A second distinct run must use fresh action IDs. */
    AutoTask_Command("AUTO_START,21,10",300002); assert(AutoTask_Running());
    unsigned id; assert(sscanf(outgoing,"QUERY,%u",&id)==1&&id>21);
    char reply[128]; snprintf(reply,sizeof(reply),"STATUS,%u,IDLE,0,0",id);
    AutoTask_Input(reply,300003); assert(!AutoTask_Running()&&stops>0);
    AutoTask_Init(send_line,event); config(); AutoTask_Command("AUTO_START,30,10",0);
    respond(0); AutoTask_Tick(600); assert(sscanf(outgoing,"EXEC,%u",&id)==1);
    snprintf(reply,sizeof(reply),"ACTION,%u,ERROR,FAULT",id); AutoTask_Input(reply,600); assert(!AutoTask_Running());
    AutoTask_Init(send_line,event); config(); AutoTask_Command("AUTO_START,40,10",0);
    AutoTask_Tick(155001); assert(!AutoTask_Running());
    /* A decimal visual error becomes pulses only at the final conversion.
     * Stale requests cannot move the robot; after arrival request a new frame. */
    AutoTask_Init(send_line,event); config(); x=y=2250; mat=ring=cards=qr=moves=heading=0;
    AutoTask_Command("AUTO_START,50,10",0);
    uint32_t now;
    unsigned request,target;
    for(now=0;now<30000;now+=600) {
        if(sscanf(outgoing,"CAM,%u,MAT,%u",&request,&target)==2) break;
        respond(now); AutoTask_Tick(now);
    }
    assert(now<30000&&target==1U);
    unsigned count=commands;
    snprintf(reply,sizeof(reply),"VISION,%u,MAT,1,253.9,160.0",request-1U); AutoTask_Input(reply,now);
    assert(commands==count);
    snprintf(reply,sizeof(reply),"VISION,%u,MAT,1,253.9,160.0",request); AutoTask_Input(reply,now);
    AutoTask_Tick(now);
    unsigned pulses,rpm; char direction;
    assert(sscanf(outgoing,"EXEC,%u,DRIVE,%c,%u,%u",&id,&direction,&pulses,&rpm)==4);
    assert(direction=='F'&&pulses==23U&&rpm==10U);
    outgoing[0]=0;
    snprintf(reply,sizeof(reply),"ACTION,%u,DONE,0",id); AutoTask_Input(reply,now);
    AutoTask_Tick(now+501U);
    unsigned next_request;
    assert(sscanf(outgoing,"CAM,%u,MAT,%u",&next_request,&target)==2&&next_request>request);
    snprintf(reply,sizeof(reply),"VISION,%u,NONE",next_request); AutoTask_Input(reply,now+502U);
    AutoTask_Tick(now+61000U); assert(!AutoTask_Running());
    test_forward_corrections();
    puts("PASS: rotate then forward, four correction directions with heading restore, four-group task, 24 cards, S2, faults and stale IDs");
}
