#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "mission_runner.h"
#include "motion_control.h"
#include "mission_flow.h"
#include "mission_config.h"
#include "camera_link.h"
#include "task_protocol.h"

static unsigned motion,raw_waits,pick_waits,cards,turntables,stows,references,stops,requests;
static unsigned expected_motion;
static bool stowing;
static bool fail_undo;
static bool undoing;
static float forward_offset,left_offset;
static int32_t arm_r,arm_z;
static uint16_t arm_base;
static bool fail_camera,fail_motion,zero_offsets,fail_fine,fail_refresh,aligning,searching,fail_search;
static unsigned align_samples,align_axes,alignment_reads,alignment_moves,required_samples;
static unsigned misses,search_axis,search_step,search_moves,coarse_queries;
static TaskVisionKind alignment_kind;
static unsigned alignment_target;
static char last_report[160];
void StartLink_Publish(const char *line)
{
    snprintf(last_report,sizeof(last_report),"%s",line);
    if(strstr(line,"CARD_DONE")) ++cards;
    if(strstr(line,",STARTING,")) {
        aligning=false; searching=false; undoing=false;
        forward_offset=left_offset=0;
    }
    if(strstr(line,",RUNNING,")||strstr(line,",TURNING,")) {
        assert(fabsf(forward_offset)<0.001f&&fabsf(left_offset)<0.001f);
        undoing=false;
    }
    if(strstr(line,",UNALIGN\n")) { undoing=true; aligning=false; }
    if(strstr(line,",SEARCH\n")) { searching=true; search_axis=0; }
    if(strstr(line,",ALIGN\n")) {
        assert(fabsf(forward_offset)<0.001f&&fabsf(left_offset)<0.001f);
        undoing=false; aligning=true; align_samples=align_axes=search_step=0;
    }
    if(strstr(line,",PICK\n")||strstr(line,",PUT\n")) {
        assert(align_samples==required_samples);
        assert(align_axes==(required_samples==3?0U:zero_offsets?0U:2U));
        aligning=false;
    }
}
void CameraLink_Idle(void) {}
bool CameraLink_Wait(TaskVisionKind kind,unsigned target,TaskVision *result)
{
    assert(MissionRunner_Running());
    if(fail_camera) return false;
    ++requests;
    char frame[100];
    switch(kind) {
    case TASK_QR: snprintf(frame,sizeof(frame),"VISION,999,QR,156+123+516+231"); break;
    case TASK_NEAR:
        assert(arm_r==110&&arm_z==0&&arm_base==180); /* 先等到位，再伸出下降。 */
        ++raw_waits; snprintf(frame,sizeof(frame),"VISION,999,NEAR,%u,2",target); break;
    case TASK_PICK:
        assert(arm_base==180);
        ++pick_waits; snprintf(frame,sizeof(frame),"VISION,999,PICK,%u,2",target); break;
    default: {
        assert(aligning&&align_samples<3);
        if(!align_samples) required_samples=kind==TASK_OCCUPIED?3U:2U;
        alignment_kind=kind; alignment_target=target;
        assert(align_samples<required_samples);
        if(align_samples) assert(align_axes==(zero_offsets?0U:2U));
        if(fail_fine&&align_samples==1) return false;
        if(fail_refresh&&align_samples==2) return false;
        const char *name=kind==TASK_TABLE?"TABLE":kind==TASK_OCCUPIED?"OCCUPIED":"RING";
        unsigned x=zero_offsets?256:align_samples==0?286:254;
        unsigned y=zero_offsets?160:align_samples==0?150:163;
        if(!zero_offsets&&kind==TASK_TABLE&&align_samples==0&&target==516U) {
            x=226; y=170; /* 第二批原料验证相反方向的固定粗调。 */
        }
        snprintf(frame,sizeof(frame),"VISION,999,%s,%u,%u,%u",name,target,x,y);
        ++align_samples; ++alignment_reads; align_axes=0; break;
    }
    }
    return TaskProtocol_Vision(frame,result);
}
bool CameraLink_Observe(TaskVisionKind kind,unsigned target,TaskVision *result)
{
    ++coarse_queries;
    if(searching) {
        static const float targets[4][2]={{100,0},{-100,0},{0,50},{0,-50}};
        assert(search_axis>=1&&search_axis<=2);
        assert(forward_offset==targets[search_step%4U][0]&&left_offset==targets[search_step%4U][1]);
        ++search_step;
        searching=false;
    }
    if(misses) { --misses; *result=(TaskVision){.kind=TASK_NONE}; return true; }
    return CameraLink_Wait(kind,target,result);
}
bool ServoControl_SetAngle(char c,uint16_t a) { (void)c; (void)a; return true; }
void Motion_Wait(uint32_t ms) { (void)ms; }
bool Motion_Reference(void) { ++references; return true; }
int32_t Motion_Radial(void) { ++stows; stowing=true; return 110; }
bool ServoControl_GetAngle(char c,uint16_t *v) { assert(c=='B'); *v=248; return true; }
static const FlowStep *next_motion(void)
{
    while(expected_motion<FLOW_COUNT(mission_flow)) {
        const FlowStep *s=&mission_flow[expected_motion++];
        if(s->op==FLOW_OP_MOVE||s->op==FLOW_OP_TURN) return s;
    }
    assert(false); return NULL;
}
bool Motion_Move(char d,float mm,uint16_t rpm)
{
    if(undoing) {
        assert(rpm==MISSION_UNALIGN_RPM);
        if(fail_undo) return false;
        if(fabsf(left_offset)>0.001f) {
            assert(d==(left_offset>0?'R':'L')&&fabsf(mm-fabsf(left_offset))<0.001f);
            left_offset=0;
        } else {
            assert(d==(forward_offset>0?'B':'F')&&fabsf(mm-fabsf(forward_offset))<0.001f);
            forward_offset=0;
        }
        if(fabsf(forward_offset)<0.001f&&fabsf(left_offset)<0.001f) undoing=false;
        return true;
    }
    if(searching) {
        static const float targets[4][2]={{100,0},{-100,0},{0,50},{0,-50}};
        float df=targets[search_step%4U][0]-forward_offset;
        float dl=targets[search_step%4U][1]-left_offset;
        assert(rpm==MISSION_ALIGN_COARSE_RPM&&search_axis<2);
        if(df!=0) assert(d==(df>0?'F':'B')&&mm==fabsf(df));
        else assert(dl!=0&&d==(dl>0?'L':'R')&&mm==fabsf(dl));
        ++search_axis; ++search_moves;
        if(fail_search) return false;
        if(d=='F') forward_offset+=mm;
        if(d=='B') forward_offset-=mm;
        if(d=='L') left_offset+=mm;
        if(d=='R') left_offset-=mm;
        return true;
    }
    if(aligning) {
        if(d=='F') forward_offset+=mm;
        if(d=='B') forward_offset-=mm;
        if(d=='L') left_offset+=mm;
        if(d=='R') left_offset-=mm;
    }
    if(aligning) {
        assert(!zero_offsets&&align_samples>=1&&align_samples<=2&&align_axes<2);
        static const char directions[2][2]={{'B','R'},{'F','L'}};
        static const float pixels[2][2]={{30,10},{2,3}};
        unsigned pass=align_samples-1U;
        if(alignment_kind==TASK_TABLE&&pass==0) {
            static const char raw_directions[2][2]={{'B','R'},{'F','L'}};
            assert(d==raw_directions[alignment_target==516U?1:0][align_axes]);
            float fixed=align_axes==0?RAW_COARSE_FORWARD_MM:RAW_COARSE_LATERAL_MM;
            assert(mm==fixed);
        } else {
            assert(d==directions[pass][align_axes]);
            assert(fabsf(mm-pixels[pass][align_axes]*MISSION_ALIGN_MM_PER_PIXEL)<0.001f);
        }
        assert(rpm==(pass==0?MISSION_ALIGN_COARSE_RPM:MISSION_ALIGN_FINE_RPM));
        ++align_axes; ++alignment_moves; return true;
    }
    const FlowStep *s=next_motion();
    assert(s->op==FLOW_OP_MOVE&&d==(char)s->direction&&mm==s->amount&&rpm==MISSION_ROUTE_RPM);
    ++motion; return !fail_motion;
}
bool Motion_Turn(char d,float deg,uint16_t rpm)
{
    const FlowStep *s=next_motion();
    assert(s->op==FLOW_OP_TURN&&d==(char)s->direction&&deg==s->amount&&rpm==MISSION_TURN_RPM);
    ++motion; return true;
}
bool Motion_MoveAll(int32_t r,int32_t z,uint16_t b,uint16_t rpm)
{
    assert(r>=0&&r<=1850&&z<=0&&b<=360&&rpm==(stowing?MOTION_STOW_RPM:MOTION_ARM_RPM));
    arm_r=r; arm_z=z; arm_base=b;
    return true;
}
bool Motion_Servo(char c,uint16_t a,uint16_t dps)
{
    if(c=='B'&&aligning&&align_samples==0) assert(a==180);
    if(c=='T') {
        assert(a==0||a==120||a==240);
        if(stowing) { assert(a==0); stowing=false; }
        else ++turntables;
    }
    if(c=='B') arm_base=a;
    assert(dps>0); return true;
}
void Motion_Stop(void) { ++stops; }
int main(void)
{
    MissionRunner_Run(100);
    assert(!MissionRunner_Running()&&strstr(last_report,"AUTO,100,DONE,25,3,24,24,S2"));
    assert(alignment_reads==19&&alignment_moves==32);
    assert(motion==25&&cards==24&&raw_waits==6&&pick_waits==9&&turntables==24&&references==1&&stows==9&&stops==0);
    expected_motion=0; fail_motion=true;
    MissionRunner_Run(101); assert(!MissionRunner_Running()&&strstr(last_report,"STOPPED")&&stops==1);
    expected_motion=0; fail_motion=false; fail_camera=true;
    MissionRunner_Run(102); assert(!MissionRunner_Running()&&strstr(last_report,"STOPPED")&&stops==2);
    expected_motion=0; fail_camera=false; zero_offsets=true;
    unsigned reads=alignment_reads,moves=alignment_moves;
    MissionRunner_Run(103);
    assert(strstr(last_report,",DONE,")&&alignment_reads==reads+19&&alignment_moves==moves);
    expected_motion=0; zero_offsets=false; fail_fine=true;
    MissionRunner_Run(104);
    assert(!MissionRunner_Running()&&strstr(last_report,"STOPPED")&&stops==3);
    expected_motion=0; fail_fine=false; misses=6;
    unsigned search_before=search_moves,coarse_before=coarse_queries;
    MissionRunner_Run(105);
    assert(strstr(last_report,",DONE,")&&search_moves==search_before+8&&coarse_queries==coarse_before+14);
    expected_motion=0; misses=1; fail_search=true;
    MissionRunner_Run(106);
    assert(!MissionRunner_Running()&&strstr(last_report,"STOPPED")&&stops==4&&search_axis==1);
    expected_motion=0; fail_search=false; misses=2;
    search_before=search_moves;
    MissionRunner_Run(107);
    assert(strstr(last_report,",DONE,")&&search_moves==search_before+2); /* 后移后找到目标，不再走左/右。 */
    assert(requests>0);
    expected_motion=0; fail_undo=true;
    unsigned old_stops=stops;
    MissionRunner_Run(108);
    assert(!MissionRunner_Running()&&strstr(last_report,",STOPPED,")&&stops==old_stops+1);
    expected_motion=0; fail_undo=false; fail_refresh=true;
    unsigned old_cards=cards;
    old_stops=stops;
    MissionRunner_Run(109);
    assert(strstr(last_report,",STOPPED,")&&cards==old_cards+6&&stops==old_stops+1);
    puts("PASS: unchanged 25 route moves, raw fixed 1000/5 mm coarse moves in both directions, two raw samples, occupied refresh, 24 transfers, fixed-origin NONE search and net-offset undo");
}
