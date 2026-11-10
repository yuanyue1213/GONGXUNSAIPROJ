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
static unsigned expected_leg=1;
static unsigned parallel_reads,parallel_turns;
static bool parallel_not_converging;
static bool parallel_converging;
static unsigned parallel_sample;
static unsigned yaw_reads;
static unsigned start_cards,stack_puts;
static bool stowing;
static bool fail_undo;
static bool undoing;
static float forward_offset,left_offset;
static int32_t arm_r,arm_z;
static uint16_t arm_base;
static bool fail_camera,fail_motion,zero_offsets,fail_fine,fail_refresh,aligning,searching,fail_search;
static unsigned align_samples,align_axes,alignment_reads,alignment_moves,required_samples,expected_axes;
static unsigned fine_corrections=2;
static bool fine_one_axis;
static float align_dx,align_dy;
static unsigned misses,search_axis,search_step,search_moves,coarse_queries;
static TaskVisionKind alignment_kind;
static char last_report[160];
static unsigned task_codes;
void StartLink_Publish(const char *line)
{
    snprintf(last_report,sizeof(last_report),"%s",line);
    if(strncmp(line,"TASK,",5U)==0) {
        assert(expected_leg==2U); /* 走完NAV_QR才加载/读取任务码。 */
#ifdef FIXED_QR
        assert(strstr(line,"," MISSION_FIXED_QR_TEXT "\n"));
#else
        assert(strstr(line,",156+123+516+231\n"));
#endif
        ++task_codes;
    }
    if(strstr(line,"CARD_DONE")) ++cards;
    if(strstr(line,",PARALLEL\n")) parallel_sample=0;
    if(strstr(line,",STARTING,")) {
        start_cards=cards; stack_puts=0;
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
        assert(align_axes==0); /* 最后复核后没有再移动，包括超差继续的情况。 */
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
    case TASK_PARALLEL: {
        ++parallel_reads;
        unsigned sample=parallel_sample++;
        if(parallel_not_converging) sample=5U;
        const char *points=sample==0?"100,150,100,150,300,150":
            sample==2||parallel_not_converging?"100,150,200,151.5,300,153":
            sample==3?"100,153,200,151.5,300,150":
            sample==1?"100,150,200,160,300,150":"100,150,200,150,300,150";
        if(parallel_converging) {
            static const char *residuals[]={"100,140,200,150,300,160",
                "100,148,200,150,300,152", "100,149.4,200,150,300,150.6"};
            assert(sample<3U); /* 第三次修正后不应再请求结果。 */
            points=residuals[sample];
        }
        snprintf(frame,sizeof(frame),"VISION,999,PARALLEL,%u,%s",target,points);
        break;
    }
    case TASK_QR:
#ifdef FIXED_QR
        assert(false); /* 固定任务码固件不能向相机请求扫码。 */
#endif
        snprintf(frame,sizeof(frame),"VISION,999,QR,156+123+516+231"); break;
    case TASK_NEAR:
        assert(arm_r==110&&arm_z==0&&arm_base==180); /* 先等到位，再伸出下降。 */
        ++raw_waits; snprintf(frame,sizeof(frame),"VISION,999,NEAR,%u,2",target); break;
    case TASK_PICK:
        ++pick_waits; assert(false); return false;
    default: {
        assert(aligning&&align_samples<=MISSION_ALIGN_FINE_MAX_MOVES+1U);
        if(!align_samples) {
            unsigned corrections=fine_corrections>MISSION_ALIGN_FINE_MAX_MOVES?
                MISSION_ALIGN_FINE_MAX_MOVES:fine_corrections;
            required_samples=zero_offsets?2U:corrections+2U;
        }
        alignment_kind=kind;
        assert(align_samples<required_samples);
        if(align_samples) assert(align_axes==expected_axes);
        if(fail_fine&&align_samples==1) return false;
        if(fail_refresh&&kind==TASK_OCCUPIED&&align_samples==fine_corrections+1U) return false;
        const char *name=kind==TASK_TABLE?"TABLE":kind==TASK_OCCUPIED?"OCCUPIED":"RING";
        int dx=0,dy=0;
        if(!zero_offsets) {
            if(align_samples==0) { dx=30; dy=-10; }
            else if(align_samples<=fine_corrections) {
                dx=align_samples%2U?-2:4;
                dy=fine_one_axis?1:align_samples%2U?3:-2;
            } else { dx=1; dy=-1; } /* 误差恰好1像素也应通过。 */
        }
#ifdef FIXED_QR
        const unsigned second_colors=132U;
#else
        const unsigned second_colors=516U;
#endif
        if(!zero_offsets&&kind==TASK_TABLE&&align_samples==0&&target==second_colors) {
            dx=-30; dy=10; /* 第二批原料验证相反方向的固定粗调。 */
        }
        align_dx=(float)dx; align_dy=(float)dy;
        float tolerance=align_samples?MISSION_ALIGN_FINE_TOLERANCE_PX:0;
        expected_axes=(fabsf(align_dx)>tolerance)+(fabsf(align_dy)>tolerance);
        snprintf(frame,sizeof(frame),"VISION,999,%s,%u,%d,%d",name,target,256+dx,160+dy);
        ++align_samples; ++alignment_reads; align_axes=0; break;
    }
    }
    return TaskProtocol_Vision(frame,result);
}
bool CameraLink_Observe(TaskVisionKind kind,unsigned target,TaskVision *result)
{
    if(kind==TASK_PARALLEL) return CameraLink_Wait(kind,target,result);
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
float Motion_Yaw(void)
{
    if(yaw_reads++%2U==0U) return 0;
    /* 人工90°反馈检验回原位的轴变换，不用理想轮子模型模拟实机。 */
    float f=forward_offset;
    forward_offset=-left_offset; left_offset=f;
    return 90;
}
bool Motion_TurnFine(float degrees,uint16_t rpm)
{
    if(parallel_converging) {
        static const float expected[]={5.710593f,1.145763f,.343771f};
        assert(parallel_sample>=1U&&parallel_sample<=3U);
        assert(fabsf(degrees-expected[parallel_sample-1U])<.0001f);
        assert(rpm==(parallel_sample==1U?PARALLEL_RPM:PARALLEL_FINE_RPM));
        ++parallel_turns; return true;
    }
    assert(rpm==PARALLEL_FINE_RPM);
    float expected=0.859372f; /* atan(3/200)转换为度；正斜率应右转。 */
    assert(fabsf(degrees-(parallel_not_converging?expected:(parallel_turns%2U?-expected:expected)))<.0001f);
    ++parallel_turns; return true;
}
int32_t Motion_Radial(void) { ++stows; stowing=true; return 110; }
bool ServoControl_GetAngle(char c,uint16_t *v) { assert(c=='B'); *v=248; return true; }
bool RoutePlan_Prepare(uint32_t id) { (void)id; return true; }
bool RoutePlan_Run(unsigned leg)
{
    assert(leg==expected_leg++&&fabsf(forward_offset)<0.001f&&fabsf(left_offset)<0.001f);
    ++motion; return !fail_motion;
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
        assert(!zero_offsets&&align_samples>=1&&align_axes<expected_axes);
        unsigned pass=align_samples-1U;
        float tolerance=pass?MISSION_ALIGN_FINE_TOLERANCE_PX:0;
        bool x_axis=fabsf(align_dx)>tolerance&&align_axes==0;
        float pixels=x_axis?align_dx:align_dy;
        assert(d==(x_axis?(pixels>0?'B':'F'):(pixels>0?'L':'R')));
        float distance=alignment_kind==TASK_TABLE&&pass==0?
            (x_axis?RAW_COARSE_FORWARD_MM:RAW_COARSE_LATERAL_MM):fabsf(pixels)*MISSION_ALIGN_MM_PER_PIXEL;
        assert(fabsf(mm-distance)<0.001f);
        assert(rpm==(pass==0?MISSION_ALIGN_COARSE_RPM:MISSION_ALIGN_FINE_RPM));
        ++align_axes; ++alignment_moves; return true;
    }
    assert(false); return false;
}
bool Motion_Turn(char d,float deg,uint16_t rpm)
{
    (void)d; (void)deg; (void)rpm; assert(false); return false;
}
bool Motion_MoveAll(int32_t r,int32_t z,uint16_t b,uint16_t rpm)
{
    assert(r>=0&&r<=1850&&z<=0&&b<=360&&rpm==(stowing?MOTION_STOW_RPM:MOTION_ARM_RPM));
    if(cards-start_cards>=21U&&cards-start_cards<24U&&z<0&&r!=ARM_SLOT_R) {
        static const StationPose poses[]=ARM_STACK_POSES;
        /* 第二批顺序对应第一批同色工位，不再询问视觉。 */
#ifdef FIXED_QR
        static const unsigned positions[]={0,2,1};
#else
        static const unsigned positions[]={1,2,0};
#endif
        const StationPose *p=&poses[positions[cards-start_cards-21U]];
        assert(r==p->put_r&&z==p->z+ARM_STACK_HEIGHT&&b==p->base);
        ++stack_puts;
    }
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
    unsigned direct_gets=0;
    for(size_t i=0;i<FLOW_COUNT(mission_flow);++i) {
        if(mission_flow[i].op!=FLOW_OP_PROCESS_PUT) continue;
        assert(i+3<FLOW_COUNT(mission_flow));
        assert(mission_flow[i+1].op==FLOW_OP_PROCESS_GET&&mission_flow[i+1].batch==mission_flow[i].batch);
        assert(mission_flow[i+2].op==FLOW_OP_STOW&&mission_flow[i+3].op==FLOW_OP_UNALIGN);
        ++direct_gets;
    }
    assert(direct_gets==2);
    MissionRunner_Run(100);
    assert(task_codes==1U);
    assert(!MissionRunner_Running()&&strstr(last_report,"AUTO,100,DONE,8,3,24,24,S2"));
    assert(alignment_reads==24&&alignment_moves==36);
    assert(stack_puts==3);
    assert(motion==8&&cards==24&&raw_waits==6&&pick_waits==0&&turntables==24&&references==5&&stows==7&&stops==0);
    assert(parallel_reads==20&&parallel_turns==8);
    expected_leg=1; fail_motion=true;
    MissionRunner_Run(101); assert(!MissionRunner_Running()&&strstr(last_report,"STOPPED")&&stops==1);
    expected_leg=1; fail_motion=false; fail_camera=true;
    MissionRunner_Run(102); assert(!MissionRunner_Running()&&strstr(last_report,"STOPPED")&&stops==2);
    expected_leg=1; fail_camera=false; zero_offsets=true;
    unsigned reads=alignment_reads,moves=alignment_moves;
    MissionRunner_Run(103);
    assert(strstr(last_report,",DONE,")&&alignment_reads==reads+12&&alignment_moves==moves);
    expected_leg=1; zero_offsets=false; fail_fine=true;
    MissionRunner_Run(104);
    assert(!MissionRunner_Running()&&strstr(last_report,"STOPPED")&&stops==3);
    expected_leg=1; fail_fine=false; misses=6;
    unsigned search_before=search_moves,coarse_before=coarse_queries;
    MissionRunner_Run(105);
    assert(strstr(last_report,",DONE,")&&search_moves==search_before+8&&coarse_queries==coarse_before+12);
    expected_leg=1; misses=1; fail_search=true;
    MissionRunner_Run(106);
    assert(!MissionRunner_Running()&&strstr(last_report,"STOPPED")&&stops==4&&search_axis==1);
    expected_leg=1; fail_search=false; misses=2;
    search_before=search_moves;
    MissionRunner_Run(107);
    assert(strstr(last_report,",DONE,")&&search_moves==search_before+2); /* 后移后找到目标，不再走左/右。 */
    assert(requests>0);
    expected_leg=1; fail_undo=true;
    unsigned old_stops=stops;
    MissionRunner_Run(108);
    assert(!MissionRunner_Running()&&strstr(last_report,",STOPPED,")&&stops==old_stops+1);
    expected_leg=1; fail_undo=false; fail_refresh=true;
    unsigned old_cards=cards;
    old_stops=stops;
    MissionRunner_Run(109);
    assert(strstr(last_report,",STOPPED,")&&cards==old_cards+21&&stops==old_stops+1);
    expected_leg=1; fail_refresh=false; fine_corrections=3;
    reads=alignment_reads; moves=alignment_moves;
    MissionRunner_Run(110);
    assert(strstr(last_report,",DONE,")&&alignment_reads==reads+30&&alignment_moves==moves+48);
    expected_leg=1; fine_corrections=1; fine_one_axis=true;
    reads=alignment_reads; moves=alignment_moves;
    MissionRunner_Run(111);
    assert(strstr(last_report,",DONE,")&&alignment_reads==reads+18&&alignment_moves==moves+18);
    expected_leg=1; fine_one_axis=false; fine_corrections=4;
    reads=alignment_reads; moves=alignment_moves; old_cards=cards; old_stops=stops;
    MissionRunner_Run(112);
    assert(strstr(last_report,",DONE,")&&!MissionRunner_Running()&&cards==old_cards+24&&stops==old_stops);
    /* 六组均三次后仍超差：不做第四次，继续全部取放并正常撤销净位移。 */
    assert(alignment_reads==reads+30&&alignment_moves==moves+48&&stack_puts==3);
    assert(fabsf(forward_offset)<0.001f&&fabsf(left_offset)<0.001f);
    /* 平行始终未达标：每站只转3次，不复核，直接完成全部取放。 */
    expected_leg=1; fine_corrections=2; parallel_not_converging=true;
    unsigned turns_before=parallel_turns,parallel_before=parallel_reads;
    old_stops=stops;
    MissionRunner_Run(113);
    assert(strstr(last_report,",DONE,")&&!MissionRunner_Running()&&stops==old_stops);
    assert(parallel_turns==turns_before+12U&&parallel_reads==parallel_before+12U);
    expected_leg=1; yaw_reads=0; parallel_not_converging=false; parallel_converging=true;
    turns_before=parallel_turns; parallel_before=parallel_reads; old_stops=stops;
    MissionRunner_Run(114);
    assert(strstr(last_report,",DONE,")&&!MissionRunner_Running()&&stops==old_stops);
    assert(parallel_turns==turns_before+12U&&parallel_reads==parallel_before+12U);
    puts("PASS: 8 downloaded navigation legs, 1-pixel fine alignment, up to 3 corrections with final verification, one-axis skip, continue after nonconvergence, 24 transfers and net-offset undo");
}
