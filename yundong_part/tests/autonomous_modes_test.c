#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../esp32s3/hello_world/main/autonomous_task.h"
static char outgoing[256],event_line[160];
static unsigned arm_moves,cards,detects,idles,qr_count,done,origin_ready;
static float robot_x,robot_y;
static unsigned heading,turns;
static unsigned references;
static void send_line(const char *s)
{
    snprintf(outgoing,sizeof(outgoing),"%s",s);
    if(strncmp(s,"CAM,",4U)==0) { if(strstr(s,",IDLE")) ++idles; else ++detects; }
}
static void event(const char *s) { snprintf(event_line,sizeof(event_line),"%s",s); if(strstr(s,",DONE,")) ++done; }
static const unsigned route[20][2]={{2250,2200},{2200,2200},{2200,1200},{2200,2200},{1200,2200},
    {1200,1200},{1200,300},{1200,1200},{300,1200},{300,2200},{1200,2200},{1200,1200},{1200,300},
    {1200,1200},{300,1200},{1200,1200},{2200,1200},{2200,300},{2250,300},{2250,150}};
static void setup(bool vision,bool arm,bool custom_route)
{
    arm_moves=cards=detects=idles=qr_count=done=0; outgoing[0]=event_line[0]=0; robot_x=robot_y=2250;
    origin_ready=arm?1U:0U;
    heading=turns=references=0U;
    AutoTask_Init(send_line,event);
    AutoTask_Command("AUTO_CFG,10,10000,20000,45,10,256.0,160.0,321,321,0,90,180",0);
    AutoTask_Command("AUTO_DRIVE,10,1000,20,3200",0);
    char line[192]; snprintf(line,sizeof(line),"AUTO_OPTIONS,10,%u,%u,%s",vision?1U:0U,arm?1U:0U,
        !vision&&arm?"156+123+516+231":"-"); AutoTask_Command(line,0);
    for(unsigned i=0;i<20U;++i) {
        unsigned y=custom_route&&i<2U?2190U:route[i][1];
        snprintf(line,sizeof(line),"AUTO_ROUTE,10,%u,%u,%u",i,route[i][0],y); AutoTask_Command(line,0);
    }
    if(arm) for(unsigned r=0;r<5U;++r) for(unsigned slot=1;slot<=3;++slot) {
        snprintf(line,sizeof(line),"AUTO_ARM,10,%c,%u,STATE_V2,10,%c,1100,-50,200,-40,20,60,0,60,0,248,140",
            "RGPTK"[r],slot,r<2U?'A':'P'); AutoTask_Command(line,0);
    }
    assert(!strstr(event_line,"ERROR"));
    AutoTask_Command("AUTO_START,20,10",0); assert(AutoTask_Running());
}
static void respond(uint32_t now)
{
    char line[256],reply[128]; strcpy(line,outgoing); outgoing[0]=0;
    unsigned id,target; char direction;
    if(sscanf(line,"QUERY,%u",&id)==1) {
        snprintf(reply,sizeof(reply),"STATUS,%u,IDLE,%u,0",id,origin_ready); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"EXEC,%u,",&id)==1) {
        char *body=strchr(strchr(line,',')+1,',')+1;
        unsigned distance,rpm,ppm;
        unsigned angle;
        if(strcmp(body,"IMU_ZERO\n")==0) { heading=0U; ++references; }
        else if(sscanf(body,"HEADING,%u.0,%u",&angle,&rpm)==2) {
            assert(angle<360U&&angle%90U==0U&&rpm==20U); heading=angle/90U; ++turns;
        } else if(sscanf(body,"DRIVE,%c,%u,%u,%u",&direction,&distance,&rpm,&ppm)==4) {
            assert(direction=='F'&&ppm==3200U); distance/=10U;
            if(heading==0U) robot_y-=distance;
            if(heading==1U) robot_x-=distance;
            if(heading==2U) robot_y+=distance;
            if(heading==3U) robot_x+=distance;
        } else if(sscanf(body,"MOVE,%c,%u,%u,%u",&direction,&distance,&rpm,&ppm)==4) {
            assert(direction=='R'); // Only the explicit first departure strafe stays in position mode.
            if(direction=='C'||direction=='W') {
                assert(false);
            } else {
                if(direction=='R') {
                    assert(robot_x==2250&&robot_y==2200&&heading==0U&&turns==0U&&distance==50U&&ppm==20000U);
                    robot_x-=distance;
                } else {
                assert(direction=='F'&&ppm==10000U);
                if(heading==0U) robot_y-=distance;
                if(heading==1U) robot_x-=distance;
                if(heading==2U) robot_y+=distance;
                if(heading==3U) robot_x+=distance;
                }
            }
        } else {
            ++arm_moves;
            if(strncmp(body,"STATE_V2,",9)==0) ++cards;
        }
        snprintf(reply,sizeof(reply),"ACTION,%u,DONE,0",id); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,MAT,%u",&id,&target)==2) {
        assert(heading==0U);
        snprintf(reply,sizeof(reply),"VISION,%u,MAT,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
    if(sscanf(line,"CAM,%u,RING,%u",&id,&target)==2) {
        assert(heading==0U);
        snprintf(reply,sizeof(reply),"VISION,%u,RING,%u,256.0,160.0",id,target); AutoTask_Input(reply,now); return;
    }
    if(strstr(line,",QR")&&sscanf(line,"CAM,%u,QR",&id)==1) {
        ++qr_count; snprintf(reply,sizeof(reply),"VISION,%u,QR,156+123+516+231",id); AutoTask_Input(reply,now);
    }
}
int main(void)
{
    for(unsigned vision=0;vision<2U;++vision) for(unsigned arm=0;arm<2U;++arm) {
        setup(vision!=0,arm!=0,vision==0&&arm==0);
        for(uint32_t now=0;now<300000U&&AutoTask_Running();now+=600U) { respond(now); AutoTask_Tick(now); }
        assert(!AutoTask_Running()&&done==1U);
        assert(robot_x==2250&&robot_y==150&&turns>0U&&references==1U);
        assert((arm&&arm_moves>0&&cards==24U)||(!arm&&arm_moves==0U&&cards==0U));
        assert((vision&&detects>0&&qr_count==1U)||(!vision&&detects==0U&&qr_count==0U));
        assert(idles>=2U);
    }
    /* Invalid options/route poison the upload until a new config starts. */
    setup(false,false,false); AutoTask_Stop("TEST");
    AutoTask_Command("AUTO_ROUTE,10,0,700,700",0);
    AutoTask_Command("AUTO_START,21,10",0); assert(!AutoTask_Running());
    setup(false,false,false); AutoTask_Stop("TEST");
    AutoTask_Command("AUTO_DRIVE,10,0,20,3200",0);
    AutoTask_Command("AUTO_START,21,10",0); assert(!AutoTask_Running());
    AutoTask_Init(send_line,event);
    AutoTask_Command("AUTO_CFG,10,10000,20000,45,10,256.0,160.0,321,321,0,90,180",0);
    AutoTask_Command("AUTO_START,22,10",0); assert(!AutoTask_Running());
    setup(false,false,false); AutoTask_Stop("TEST");
    AutoTask_Command("AUTO_OPTIONS,10,2,0,-",0);
    AutoTask_Command("AUTO_START,21,10",0); assert(!AutoTask_Running());
    setup(false,false,false); AutoTask_Stop("TEST");
    AutoTask_Command("AUTO_OPTIONS,10,0,1,-",0);
    AutoTask_Command("AUTO_START,21,10",0); assert(!AutoTask_Running());
    puts("PASS: four modes, zero arm output without arm, zero recognition without vision, no-origin chassis, custom route, fail-closed config");
}
