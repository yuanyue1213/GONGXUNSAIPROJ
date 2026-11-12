#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "start_link.h"
#include "mission_runner.h"

static UART_HandleTypeDef uart;
static uint8_t *rx;
static unsigned moves,stops;
static bool move_ok=true;
static char status[192];
void RoutePlan_Init(void) {}
bool RoutePlan_Command(const char *frame) { (void)frame; return false; }
bool RoutePlan_CanStart(void) { assert(!"ForwardTest must not require PC"); return false; }
void ImuLink_Process(void) {}
bool ZDT_Motor_RxCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ImuLink_RxCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ZDT_Motor_ErrorCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ImuLink_ErrorCallback(UART_HandleTypeDef *u) { (void)u; return false; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t count)
{ assert(u==&uart&&count==1); rx=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *p,uint16_t count,uint32_t timeout)
{
    (void)timeout; assert(u==&uart&&count<sizeof(status));
    memcpy(status,p,count); status[count]=0; return HAL_OK;
}
static void feed(const char *s)
{
    while(*s) { *rx=(uint8_t)*s++; HAL_UART_RxCpltCallback(&uart); }
    StartLink_Service();
}
bool Motion_MoveStraight(char direction,float distance,uint16_t rpm)
{
    assert(MissionRunner_Running()&&direction=='F'&&distance==3000&&rpm==200);
    ++moves;
    feed("START,99\n"); assert(strstr(status,",BUSY,"));
    MissionRunner_Run(999); StartLink_Tick(); assert(moves==stops+1);
    ++stops; /* 直行控制内部负责成功归零或失败立即停车。 */
    return move_ok;
}
void Motion_Stop(void) { ++stops; }
int main(void)
{
    StartLink_Init(&uart); StartLink_Service(); StartLink_Tick();
    assert(!moves&&!MissionRunner_Running()); /* 上电不自动运动。 */
    feed("START,42\n"); assert(!moves); StartLink_Tick();
    assert(moves==1&&stops==1&&!MissionRunner_Running());
    assert(strstr(status,"AUTO,42,DONE,1,0,0,0,FORWARD_TEST"));
    StartLink_Tick(); feed("START,42\n"); StartLink_Tick();
    assert(moves==1&&strstr(status,"DUPLICATE_START"));
    move_ok=false; feed("START,43\n"); StartLink_Tick();
    assert(moves==2&&stops==2&&!MissionRunner_Running());
    assert(strstr(status,"AUTO,43,STOPPED,1,0,0,0,COMMAND_FAILED"));
    puts("PASS: App-started pure position-mode 3000mm at 200RPM without IMU, no PC/camera/arm, no automatic run, BUSY/duplicate protection and stop after success/failure");
}
