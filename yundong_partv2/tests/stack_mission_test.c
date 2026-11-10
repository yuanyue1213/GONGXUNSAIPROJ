#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mission_flow.h"
#include "mission_runner.h"
#include "start_link.h"
#include "camera_link.h"
#include "motion_control.h"

static UART_HandleTypeDef uart;
static uint8_t *rx;
static unsigned puts_count,picks,stops,starts,grips;
static unsigned parallel_frames,chassis_moves;
static float offset;
static int32_t radial;
static uint16_t base;
static bool fail_pick,checked_busy;
static char status[160];
static void feed(const char *s)
{
    while(*s) { *rx=(uint8_t)*s++; HAL_UART_RxCpltCallback(&uart); }
    StartLink_Service();
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ assert(u==&uart&&n==1); rx=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t timeout)
{
    (void)timeout; assert(u==&uart&&n<sizeof(status));
    memcpy(status,p,n); status[n]=0;
    if(strstr(status,",STARTING,")) ++starts;
    return HAL_OK;
}
void RoutePlan_Init(void) {}
bool RoutePlan_Command(const char *frame) { (void)frame; return false; }
bool RoutePlan_CanStart(void) { assert(false); return false; }
bool RoutePlan_Prepare(uint32_t id) { (void)id; assert(false); return false; }
bool RoutePlan_Run(unsigned leg) { (void)leg; assert(false); return false; }
void ImuLink_Process(void) {}
void CameraLink_Process(void) {}
bool ZDT_Motor_RxCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ImuLink_RxCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool CameraLink_RxCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ZDT_Motor_ErrorCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ImuLink_ErrorCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool CameraLink_ErrorCallback(UART_HandleTypeDef *u) { (void)u; return false; }
void CameraLink_Idle(void) {}
bool CameraLink_Observe(TaskVisionKind kind,unsigned target,TaskVision *v)
{
    assert(target==132U);
    *v=(TaskVision){.kind=kind,.target=target,.center={260,160},
                   .rings={{100,160},{256,160},{400,160}}};
    if(kind==TASK_PARALLEL) ++parallel_frames;
    assert(kind==TASK_OCCUPIED||kind==TASK_PARALLEL); return true;
}
bool CameraLink_Wait(TaskVisionKind kind,unsigned target,TaskVision *v)
{
    if(kind==TASK_OCCUPIED) {
        assert(target==132U);
        *v=(TaskVision){.kind=kind,.target=target,.center={256,160}}; return true;
    }
    ++picks; assert(false); return false; /* 校准后不能再请求视觉确认。 */
}
bool Motion_Reference(void) { return true; }
float Motion_Yaw(void) { return 0; }
bool Motion_Move(char d,float mm,uint16_t rpm)
{
    assert((d=='B'||d=='F')&&fabsf(mm-4*MISSION_ALIGN_MM_PER_PIXEL)<.001f);
    assert(rpm==MISSION_ALIGN_COARSE_RPM||rpm==MISSION_UNALIGN_RPM);
    offset+=d=='F'?mm:-mm; ++chassis_moves; return true;
}
bool Motion_Turn(char d,float deg,uint16_t rpm)
{ (void)d; (void)deg; (void)rpm; assert(false); return false; }
bool Motion_TurnPulses(int32_t p,uint16_t rpm)
{ (void)p; (void)rpm; assert(false); return false; }
bool Motion_TurnFine(float degrees,uint16_t rpm)
{ (void)degrees; (void)rpm; assert(false); return false; }
void Motion_Stop(void) { ++stops; }
int32_t Motion_Radial(void) { return radial; }
bool ServoControl_GetAngle(char c,uint16_t *a) { assert(c=='B'); *a=base; return true; }
bool ServoControl_SetAngle(char c,uint16_t a) { if(c=='B') base=a; return true; }
bool Motion_Servo(char c,uint16_t a,uint16_t dps)
{ assert(dps>0); if(c=='B') base=a; if(c=='G'&&a==ARM_GRIP_CLOSED) ++grips; return true; }
bool Motion_MoveAll(int32_t r,int32_t z,uint16_t b,uint16_t rpm)
{
    assert(rpm==MOTION_ARM_RPM||rpm==MOTION_STOW_RPM);
    if(fail_pick&&r==ARM_SLOT_R&&z==ARM_SLOT_Z) return false;
    if(z!=0&&r!=ARM_SLOT_R) {
        static const StationPose poses[]=ARM_STACK_POSES;
        static const unsigned positions[]={0,2,1};
        const StationPose *p=&poses[positions[puts_count%3U]];
        assert(r==p->put_r&&z==p->z+ARM_STACK_HEIGHT&&b==p->base); ++puts_count;
    }
    radial=r; base=b; return true;
}
void Motion_Wait(uint32_t ms)
{
    (void)ms;
    if(!checked_busy) {
        checked_busy=true; feed("START,99\n");
        assert(strstr(status,",BUSY,"));
        StartLink_Tick(); assert(starts==1U);
    }
}
int main(void)
{
    assert(!Flow_Check(mission_flow,FLOW_COUNT(mission_flow)));
    StartLink_Init(&uart); StartLink_Tick(); assert(starts==0U&&puts_count==0U);
    feed("START,10\n"); StartLink_Tick();
    assert(starts==1U&&puts_count==3U&&picks==0U&&grips==3U&&stops==0U);
    assert(parallel_frames==1U&&chassis_moves==2U&&fabsf(offset)<.001f);
    assert(strstr(status,",DONE,")&&!MissionRunner_Running());
    feed("START,10\n"); StartLink_Tick(); assert(starts==1U&&strstr(status,"DUPLICATE_START"));
    fail_pick=true;
    feed("START,11\n"); StartLink_Tick();
    assert(starts==2U&&puts_count==3U&&stops==1U&&strstr(status,",STOPPED,"));
    puts("PASS: App-started stacking without PC/QR/navigation, same-color order, second-layer height, BUSY and failure exit");
}
