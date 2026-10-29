#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "start_link.h"
#include "mission_runner.h"

static UART_HandleTypeDef uart;
static uint8_t *rx;
static unsigned runs;
static bool running;
static char published[192];
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t count)
{ assert(u==&uart&&count==1); rx=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *p,uint16_t count,uint32_t timeout)
{ (void)timeout; assert(u==&uart&&count<sizeof(published)); memcpy(published,p,count); published[count]=0; return HAL_OK; }
static void feed(const char *s) { while(*s) { *rx=(uint8_t)*s++; HAL_UART_RxCpltCallback(&uart); } StartLink_Service(); }
void ImuLink_Process(void) {}
void CameraLink_Process(void) {}
bool ZDT_Motor_RxCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ImuLink_RxCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool CameraLink_RxCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ZDT_Motor_ErrorCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool ImuLink_ErrorCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool CameraLink_ErrorCallback(UART_HandleTypeDef *u) { (void)u; return false; }
bool MissionRunner_Running(void) { return running; }
void MissionRunner_Run(uint32_t id)
{
    assert(id==10||id==11); running=true; ++runs;
    feed("START,99\n"); assert(strstr(published,",BUSY,"));
    StartLink_Tick(); assert(runs==(id==10?1U:2U)); /* 等待服务不得递归重入流程。 */
    running=false;
}
int main(void)
{
    StartLink_Init(&uart);
    feed("EXEC,10,TURN\nAUTO_CFG,10,1\nSTART,0\nSTART,4294967296\n");
    StartLink_Tick(); assert(!runs);
    feed("START,10\n"); StartLink_Tick(); assert(runs==1);
    feed("START,10\n"); StartLink_Tick(); assert(runs==1&&strstr(published,"DUPLICATE_START"));
    feed("START,11\n"); StartLink_Tick(); assert(runs==2);
    for(unsigned i=0;i<50;++i) { *rx='A'; HAL_UART_RxCpltCallback(&uart); }
    feed("START,12\n"); StartLink_Tick(); assert(runs==2);
    puts("PASS: start-only UART boundary, duplicate suppression, BUSY during sequential waits, no recursive mission and oversized-frame rejection");
}
