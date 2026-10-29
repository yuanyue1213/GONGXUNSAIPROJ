/* mission_flow.h -> 顺序 switch -> 直接动作函数。
 * 站点按三件物料顺序执行，不再通过字符串动作协议或分阶段任务状态机中转。 */
#include "mission_runner.h"
#include "mission_flow.h"
#include "arm_flow.h"
#include "motion_control.h"
#include "mission_config.h"
#include "start_link.h"
#include "camera_link.h"
#include "task_protocol.h"
#include <math.h>
#include <stdio.h>

static bool running;
static uint32_t run;
static unsigned node,item,transfers;
static TaskCode code;
static TaskVision reply;
static bool aligned,record_alignment;
static float align_forward,align_left;

/* 只记录成功下发并等待结束的校准位移；单位 mm，含搜索动作。 */
static bool align_move(char direction,float mm,uint16_t rpm)
{
    if(!Motion_Move(direction,mm,rpm)) return false;
    if(record_alignment) {
        if(direction=='F') align_forward+=mm;
        else if(direction=='B') align_forward-=mm;
        else if(direction=='L') align_left+=mm;
        else if(direction=='R') align_left-=mm;
    }
    return true;
}

static void report(const char *state,const char *reason)
{
    char line[128];
    snprintf(line,sizeof(line),"AUTO,%lu,%s,%u,%u,%u,%u,%s\n",
        (unsigned long)run,state,node,item,transfers,transfers,reason);
    StartLink_Publish(line);
}
bool MissionRunner_Running(void) { return running; }
static bool align_axis(float mm,char positive,uint16_t rpm)
{
    if(mm==0.0f) return true;
    char opposite=positive=='F'?'B':positive=='B'?'F':positive=='L'?'R':'L';
    return align_move(mm>0?positive:opposite,fabsf(mm),rpm);
}
static bool align_xy(float x,float y,uint16_t rpm)
{
    /* 以车体轴为顺序：先前后，再左右；每轴函数等待运动结束后才返回。
     * 即使以后调整相机安装方向，也不把图像 X 固定当作第一轴。 */
    if(MISSION_IMAGE_RIGHT_DIRECTION=='F'||MISSION_IMAGE_RIGHT_DIRECTION=='B')
        return align_axis(x,MISSION_IMAGE_RIGHT_DIRECTION,rpm)&&align_axis(y,MISSION_IMAGE_DOWN_DIRECTION,rpm);
    return align_axis(y,MISSION_IMAGE_DOWN_DIRECTION,rpm)&&align_axis(x,MISSION_IMAGE_RIGHT_DIRECTION,rpm);
}
static bool align_once(TaskVisionKind kind,unsigned target,uint16_t rpm,bool search)
{
    unsigned search_step=0;
    float search_forward=0,search_left=0;
    for(;;) {
        bool ok=search?CameraLink_Observe(kind,target,&reply):CameraLink_Wait(kind,target,&reply);
        if(!ok) return false;
        CameraLink_Idle();
        if(reply.kind!=TASK_NONE) break;
        /* 只在粗调收到明确 NONE 后移动。通信无回复不会被当作没看到目标。 */
        report("WORK","SEARCH");
        /* 目标相对搜索起点：(+100,0)、(-100,0)、(0,+50)、(0,-50)。
         * 先前后、再左右；到达完整目标后重新识别，不在换轴途中识别。 */
        float target_forward=search_step==0U?MISSION_ALIGN_SEARCH_FORWARD_MM:
            search_step==1U?-MISSION_ALIGN_SEARCH_FORWARD_MM:0;
        float target_left=search_step==2U?MISSION_ALIGN_SEARCH_LATERAL_MM:
            search_step==3U?-MISSION_ALIGN_SEARCH_LATERAL_MM:0;
        if(!align_axis(target_forward-search_forward,'F',rpm)) return false;
        if(!align_axis(target_left-search_left,'L',rpm)) return false;
        search_forward=target_forward;
        search_left=target_left;
        search_step=(search_step+1U)%4U;
    }
    /* 同一帧算出两轴距离，依次移动；完成后由下一次请求重新识别。 */
    float x=(reply.center.x-MISSION_TARGET_X)*MISSION_ALIGN_MM_PER_PIXEL;
    float y=(reply.center.y-MISSION_TARGET_Y)*MISSION_ALIGN_MM_PER_PIXEL;
    if(kind==TASK_TABLE&&search) {
        /* 原料粗调：图像偏差只决定方向，车体前后/左右使用固定步长。
         * 精调及其他站点仍按像素偏差换算距离。 */
        bool x_forward=MISSION_IMAGE_RIGHT_DIRECTION=='F'||MISSION_IMAGE_RIGHT_DIRECTION=='B';
        if(x!=0.0f) x=copysignf(x_forward?RAW_COARSE_FORWARD_MM:RAW_COARSE_LATERAL_MM,x);
        if(y!=0.0f) y=copysignf(x_forward?RAW_COARSE_LATERAL_MM:RAW_COARSE_FORWARD_MM,y);
    }
    return align_xy(x,y,rpm);
}
static bool align_station(TaskVisionKind kind,unsigned batch)
{
    unsigned target;
    if(aligned) return false;
    if(kind!=TASK_TABLE&&kind!=TASK_OCCUPIED&&kind!=TASK_RING) return false;
    if(kind!=TASK_RING&&batch>=2U) return false;
    align_forward=align_left=0;
    aligned=true;
    record_alignment=true;
    report("WORK","ALIGN");
    target=kind==TASK_RING?2U:code.colors[batch][0]*100U+code.colors[batch][1]*10U+code.colors[batch][2];
    if(!Motion_Servo('B',ARM_ALIGN_BASE,MOTION_BASE_ALIGN_DPS)) return false;
    /* 参考项目的粗调 + 精调：各取一帧、各修正两轴；不循环追逐像素阈值。
     * 保留本车相机分辨率和比例，两轮分别使用路线速度与微调速度。 */
    if(!align_once(kind,target,MISSION_ALIGN_COARSE_RPM,true)) return false;
    if(!align_once(kind,target,MISSION_ALIGN_FINE_RPM,false)) return false;
    /* 加工区最后一次移动后刷新三工位锚点；原料抓取直接看固定像素位置。 */
    if(kind==TASK_OCCUPIED) {
        if(!CameraLink_Wait(kind,target,&reply)) return false;
        CameraLink_Idle();
    }
    return true;
}
static bool unalign(void)
{
    if(!aligned) return false;
    record_alignment=false;
    report("WORK","UNALIGN");
    /* 先撤销横向、再撤销前后；两者抵消累计净位移，不是重走完整搜索路径。 */
    if(!align_axis(-align_left,'L',MISSION_UNALIGN_RPM)) return false;
    if(!align_axis(-align_forward,'F',MISSION_UNALIGN_RPM)) return false;
    aligned=false;
    align_forward=align_left=0;
    return true;
}
static bool pick_material(ArmArea area,unsigned position,unsigned color)
{
    const ArmStep approach[]={ARM_PICK_APPROACH(area,position)};
    const ArmStep finish[]={ARM_PICK_FINISH(area,position)};
    if(!ArmFlow_Run(approach,ARM_COUNT(approach))) return false;
    if(area==ARM_AREA_RAW) {
        if(!CameraLink_Wait(TASK_NEAR,color,&reply)) return false;
        CameraLink_Idle();
    }
    return ArmFlow_Run(finish,ARM_COUNT(finish));
}
static bool put_material(ArmArea area,unsigned position)
{
    const ArmStep steps[]={ARM_PUT_STEPS(area,position)};
    return ArmFlow_Run(steps,ARM_COUNT(steps));
}
static bool station(const FlowStep *step)
{
    unsigned batch=(unsigned)step->batch-1U;
    /* 仅保护二维码数组访问；流程校验不解释批次，也不限制站点顺序。 */
    if(batch>=sizeof(code.colors)/sizeof(code.colors[0])) return false;
    bool raw=step->op==FLOW_OP_PICK_RAW;
    bool pick=raw||step->op==FLOW_OP_PROCESS_GET;
    ArmArea area;
    if(raw) area=ARM_AREA_RAW;
    else if(step->op==FLOW_OP_STORE) area=ARM_AREA_STORE;
    else if(step->op==FLOW_OP_STACK) area=ARM_AREA_STACK;
    else area=ARM_AREA_PROCESS;
    for(item=0;item<3;++item) {
        unsigned color=code.colors[batch][item],position;
        if(raw) position=1U;
        else if(pick||step->op==FLOW_OP_STACK) {
            /* 上一次取放会改变底座方向，每次识别都回到约定观察姿态。 */
            if(!Motion_Servo('B',ARM_ALIGN_BASE,MOTION_BASE_ALIGN_DPS)) return false;
            if(!CameraLink_Wait(TASK_PICK,color,&reply)) return false;
            position=reply.position-1U;
            CameraLink_Idle();
        } else {
            /* 工位的物理左/中/右编号为 3/2/1；与车载槽位分开。 */
            position=3U-code.places[batch][item];
        }
        static const uint16_t slots[]=ARM_SLOT_ANGLES;
        if(!Motion_Servo('T',slots[item],MOTION_TURNTABLE_DPS)) return false;
        report("WORK",pick?"PICK":"PUT");
        bool ok;
        if(pick)
            ok=pick_material(area,position,color);
        else
            ok=put_material(area,position);
        if(!ok) return false;
        /* 每张取放动作均完成一次抓取和一次释放，与 App 原有统计语义一致。 */
        ++transfers;
        report("WORK","CARD_DONE");
    }
    return true;
}
static bool prepare(void)
{
    /* 上电机械臂姿态作为逻辑零点，与参考代码一样不读取编码器建立原点。
     * 启动前须将机械臂放在约定初始姿态；每轮流程的 STOW 会回到逻辑零点。 */
    const ArmStep steps[]={ARM_PREPARE_STEPS};
    return ArmFlow_Run(steps,ARM_COUNT(steps));
}
void MissionRunner_Run(uint32_t id)
{
    if(running) return;
    run=id; node=item=transfers=0;
    aligned=record_alignment=false;
    align_forward=align_left=0;
    const char *error=Flow_Check(mission_flow,FLOW_COUNT(mission_flow));
    if(error) {
        report("ERROR",error);
        return;
    }
    running=true;
    report("STARTING","PREPARE");
    /* 流程表就是执行顺序，无需另一个执行器对象维护当前位置。 */
    for(size_t i=0;i<FLOW_COUNT(mission_flow);++i) {
        const FlowStep *step=&mission_flow[i];
        bool ok=false;
        switch(step->op) {
        case FLOW_OP_PREPARE:
            ok=prepare();
            break;
        case FLOW_OP_IMU_ZERO:
            ok=Motion_Reference();
            break;
        case FLOW_OP_STOW:
            ok=ArmFlow_Stow();
            break;
        case FLOW_OP_ALIGN:
            ok=align_station((TaskVisionKind)step->vision,(unsigned)step->batch-1U);
            record_alignment=false;
            break;
        case FLOW_OP_UNALIGN:
            ok=unalign();
            break;
        case FLOW_OP_ARM:
            report("WORK","ARM");
            ok=ArmFlow_Run(step->arm,step->arm_count);
            break;
        case FLOW_OP_MOVE:
            report("RUNNING","MOVE");
            ok=Motion_Move((char)step->direction,(float)step->amount,MISSION_ROUTE_RPM);
            break;
        case FLOW_OP_TURN:
            report("TURNING","TURN");
            ok=Motion_Turn((char)step->direction,(float)step->amount,MISSION_TURN_RPM);
            break;
        case FLOW_OP_SCAN:
            report("SCANNING","QR");
            ok=CameraLink_Wait(TASK_QR,0,&reply);
            if(ok) code=reply.code;
            if(ok) {
                char text[96];
                snprintf(text,sizeof(text),"TASK,%lu,%s\n",(unsigned long)run,reply.text);
                StartLink_Publish(text);
                CameraLink_Idle();
            }
            break;
        case FLOW_OP_PICK_RAW: case FLOW_OP_PROCESS_PUT: case FLOW_OP_PROCESS_GET:
        case FLOW_OP_STORE: case FLOW_OP_STACK:
            ok=station(step);
            break;
        case FLOW_OP_END:
            CameraLink_Idle();
            report("DONE","S2");
            running=false;
            return;
        default:
            break;
        }
        if(!ok) break;
        Motion_Wait(MISSION_STEP_SETTLE_MS);
        if(step->op==FLOW_OP_MOVE||step->op==FLOW_OP_TURN) ++node;
    }
    /* 参数或发包失败仍退出本轮并发出停车指令；不增加反馈监控或停车重试状态机。 */
    Motion_Stop();
    CameraLink_Idle();
    report("STOPPED","COMMAND_FAILED");
    running=false;
}
