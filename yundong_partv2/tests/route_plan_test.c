#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "route_plan.h"
#include "robot_config.h"
#include "nav_protocol.h"
static uint32_t tick;
static bool running;
static unsigned zone,moves,turns,scan_count;
static char last[160];
uint32_t HAL_GetTick(void) { return tick++; }
bool MissionRunner_Running(void) { return running; }
void StartLink_Service(void) { tick+=10; assert(tick<100000U); }
void Motion_Wait(uint32_t ms) { tick+=ms; }
bool Motion_Move(char d,float mm,uint16_t rpm)
{
    assert(rpm==MISSION_ROUTE_RPM);
    if(moves==0) assert(d==(zone==2?'R':'L')&&mm==125);
    else if(moves==1) assert(d=='F'&&mm==165);
    else assert(d=='F'&&mm==10);
    ++moves; return true;
}
bool Motion_Turn(char d,float deg,uint16_t rpm)
{
    assert(d==(zone==2?'R':'L')&&deg==90&&rpm==MISSION_TURN_RPM&&scan_count==1);
    ++turns; return true;
}
static void upload(uint32_t id)
{
    char lines[8][96],head[64],frame[128];
    snprintf(head,sizeof(head),"%lu,%u,8\n",(unsigned long)id,zone);
    uint32_t crc=Nav_Crc32(UINT32_MAX,head,strlen(head));
    for(unsigned i=0;i<8;++i) {
        snprintf(lines[i],sizeof(lines[i]),"NAV_STEP,%lu,%u,%u,MOVE,F,10\n",(unsigned long)id,i,i+1);
        crc=Nav_Crc32(crc,lines[i],strlen(lines[i]));
        lines[i][strlen(lines[i])-1]=0;
    }
    snprintf(frame,sizeof(frame),"NAV_BEGIN,%lu,%u,8,%08lX",(unsigned long)id,zone,(unsigned long)(crc^UINT32_MAX));
    char bad[128];
    snprintf(bad,sizeof(bad),"NAV_BEGIN,%lu,%u,8,%08lX",(unsigned long)id,zone,(unsigned long)(crc^UINT32_MAX^1U));
    RoutePlan_Command(bad);
    for(unsigned i=0;i<8;++i) RoutePlan_Command(lines[i]);
    snprintf(head,sizeof(head),"NAV_END,%lu",(unsigned long)id);
    RoutePlan_Command(head); assert(strstr(last,"CRC_OR_LEGS"));
    assert(!RoutePlan_Run(1)); /* CRC错误的路线不能执行。 */
    RoutePlan_Command(frame); assert(strstr(last,"NAV_ACK,")&&strstr(last,"BEGIN"));
    RoutePlan_Command(lines[1]); assert(strstr(last,"SEQUENCE"));
    snprintf(head,sizeof(head),"NAV_END,%lu",(unsigned long)id);
    RoutePlan_Command(head); assert(strstr(last,"COUNT"));
    RoutePlan_Command("NAV_STEP,99,0,1,MOVE,F,-1"); assert(strstr(last,"VALUE"));
    for(unsigned i=0;i<8;++i) {
        RoutePlan_Command(lines[i]); assert(strstr(last,"NAV_ACK,"));
        RoutePlan_Command(lines[i]); assert(strstr(last,"NAV_ACK,")); /* 重传不重复累计CRC。 */
        if(i==3) RoutePlan_Command(frame); /* BEGIN确认重传不清空已收到的步骤。 */
    }
    RoutePlan_Command(head); assert(strstr(last,"NAV_READY,"));
    RoutePlan_Command(head); assert(strstr(last,"NAV_READY,"));
    RoutePlan_Command(frame); assert(strstr(last,"STATE")); /* 已提交后不能覆盖。 */
}
void StartLink_Publish(const char *line)
{
    snprintf(last,sizeof(last),"%s",line);
    unsigned long id; unsigned scan,x,h; int y;
    if(sscanf(line,"SCAN_READY,%lu,%u,%u,%d,%u",&id,&scan,&x,&y,&h)==5) {
        assert(x==2125&&y==(zone==2?2085:315));
        assert(h==(scan==1?(zone==2?180U:0U):270U));
        assert(scan==scan_count+1&&moves==2&&turns==scan-1); ++scan_count;
        char ack[64]; snprintf(ack,sizeof(ack),"SCAN_OK,%lu,%u",id,scan);
        RoutePlan_Command("SCAN_OK,999,1"); /* 旧运行确认无效。 */
        RoutePlan_Command(ack);
    } else if(sscanf(line,"NAV_WAIT,%lu",&id)==1) upload((uint32_t)id);
}
int main(void)
{
    assert((Nav_Crc32(UINT32_MAX,"123456789",9)^UINT32_MAX)==0xCBF43926U);
    for(zone=1;zone<=2;++zone) {
        running=false; moves=turns=scan_count=tick=0; RoutePlan_Init();
        assert(!RoutePlan_CanStart());
        RoutePlan_Command("PC_HELLO,-1"); assert(!RoutePlan_CanStart());
        char hello[32]; snprintf(hello,sizeof(hello),"PC_HELLO,%u",zone);
        RoutePlan_Command(hello); assert(RoutePlan_CanStart());
        tick+=NAV_PC_FRESH_MS; assert(!RoutePlan_CanStart());
        RoutePlan_Command(hello); running=true;
        assert(RoutePlan_Prepare(100+zone)&&moves==2&&turns==1&&scan_count==2);
        assert(!RoutePlan_Run(2));
        for(unsigned leg=1;leg<=8;++leg) assert(RoutePlan_Run(leg));
        assert(moves==10&&!RoutePlan_Run(8)&&strstr(last,",8,7,DONE"));
        assert(RoutePlan_Command("NAV_BEGIN,1,1,129,00000000"));
        assert(strstr(last,"STATE"));
    }
    puts("PASS: both entrances, ordered scans, PC readiness, route upload/CRC, duplicate retry, busy protection and eight navigation legs");
}
