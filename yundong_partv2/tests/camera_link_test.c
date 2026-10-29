#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "camera_link.h"
#include "mission_config.h"

static UART_HandleTypeDef uart;
static uint8_t *rx;
static char sent[100];
static const char **incoming;
static unsigned service_count;
static bool fail_tx;
static unsigned tx_count;
static unsigned ack_count;
static unsigned active_request;
static bool check_repeat,fail_retry_once;
static char first_frame[100];
static uint32_t tick,step=100;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ assert(u==&uart&&n==1); rx=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *p,uint16_t n,uint32_t timeout)
{
    (void)timeout; assert(u==&uart&&n<sizeof(sent));
    memcpy(sent,p,n); sent[n]=0;
    if(strncmp(sent,"ACK,",4)==0) { ++ack_count; return fail_tx?HAL_ERROR:HAL_OK; }
    assert(sscanf(sent,"CAM,%u,",&active_request)==1);
    ++tx_count;
    if(check_repeat) {
        if(tx_count==1) strcpy(first_frame,sent);
        else assert(strcmp(first_frame,sent)==0);
    }
    if(fail_retry_once&&tx_count==2) { fail_retry_once=false; return HAL_ERROR; }
    return fail_tx?HAL_ERROR:HAL_OK;
}
static void feed(const char *s)
{ while(*s) { *rx=(uint8_t)*s++; assert(CameraLink_RxCallback(&uart)); } CameraLink_Process(); }
static void service(void)
{
    TaskVision nested;
    assert(!CameraLink_Wait(TASK_QR,0,&nested)); /* 等待不能重入或覆盖当前结果。 */
    assert(incoming&&*incoming&&++service_count<20);
    tick+=step;
    const char *line=*incoming++;
    unsigned request;
    int offset=0;
    /* 普通夹具跟随当前请求；999特意保留，用于验证旧编号被丢弃。 */
    if(sscanf(line,"VISION,%u,%n",&request,&offset)==1&&offset&&request!=999) {
        char frame[160];
        snprintf(frame,sizeof(frame),"VISION,%u,%s",active_request,line+offset);
        feed(frame);
    } else feed(line);
}
static void replies(const char **lines) { incoming=lines; service_count=0; }
int main(void)
{
    TaskVision v;
    CameraLink_Init(&uart,service);
    const char *qr[]={"old fragment\n", "VISION,99,NONE\n", "VISION,99,NEAR,1,2\n",
        "VISION,999,QR,156+123+516+231\n", "VISION,1,QR,156+123+516+231\r", "\n", NULL};
    replies(qr);
    assert(CameraLink_Wait(TASK_QR,0,&v)&&v.kind==TASK_QR&&service_count==6);
    assert(strcmp(sent,"ACK,1\n")==0&&v.request==1);
    unsigned acknowledged=ack_count;
    fail_tx=true; feed("VISION,1,QR,156+123+516+231\n");
    fail_tx=false; feed("VISION,1,QR,156+123+516+231\n");
    assert(ack_count==acknowledged+2); /* 确认发送失败后，重复结果可触发再次确认。 */
    const char *near[]={"\n", "VISION,2,NEAR,2,2\n", "VISION,2,NEAR,1,3\n", "VISION,2,NEAR,1,2\n", NULL};
    replies(near);
    assert(CameraLink_Wait(TASK_NEAR,1,&v)&&v.position==2&&service_count==4);
    char long_frame[200]; memset(long_frame,'x',110);
    strcpy(long_frame+110,"\nVISION,3,RING,2,256.0,160.0\n");
    const char *ring[]={"\n",long_frame,NULL}; replies(ring);
    assert(CameraLink_Wait(TASK_RING,2,&v)&&v.center.x==256&&service_count==2);
    fail_tx=true; assert(!CameraLink_Wait(TASK_QR,0,&v));
    fail_tx=false; replies(qr); assert(CameraLink_Wait(TASK_QR,0,&v)); /* 发包失败释放等待状态。 */
    assert(!CameraLink_Wait(TASK_NONE,0,&v)&&!CameraLink_Wait(TASK_QR,0,NULL));
    assert(!CameraLink_Wait(TASK_NEAR,0,&v));
    step=500;
    const char *missing[]={"\n", "VISION,5,NONE\n", "VISION,5,NONE\n", NULL}; replies(missing);
    assert(CameraLink_Observe(TASK_RING,2,&v)&&v.kind==TASK_NONE&&service_count==3);
    const char *recover[]={"\n", "VISION,5,NONE\n", "VISION,5,RING,2,256,160\n", NULL}; replies(recover);
    assert(CameraLink_Observe(TASK_RING,2,&v)&&v.kind==TASK_RING&&service_count==3);
    const char *silence[]={"\n", "", "", "VISION,5,RING,2,256,160\n", NULL}; replies(silence);
    assert(CameraLink_Observe(TASK_RING,2,&v)&&v.kind==TASK_RING&&service_count==4);
    const char *fine[]={"\n", "VISION,5,NONE\n", "VISION,5,NONE\n", "VISION,5,RING,2,256,160\n", NULL}; replies(fine);
    assert(CameraLink_Wait(TASK_RING,2,&v)&&v.kind==TASK_RING&&service_count==4);
    assert(!CameraLink_Observe(TASK_QR,0,&v)&&!CameraLink_Observe(TASK_NEAR,1,&v));
    /* 相机晚启动：前两秒无任何回复，补发必须沿用同一编号。 */
    const char *late[]={"", "", "", "", "\n", "VISION,7,QR,156+123+516+231\n", NULL};
    tx_count=0; check_repeat=true; fail_retry_once=true; replies(late);
    assert(CameraLink_Wait(TASK_QR,0,&v)&&v.kind==TASK_QR&&service_count==6);
    assert(tx_count==3&&!fail_retry_once); /* 一次补发失败后仍能继续恢复。 */
    check_repeat=false;
    CameraLink_Idle(); assert(strstr(sent,",IDLE\n"));
    unsigned stopped_tx=tx_count; tick+=CAMERA_REQUEST_RETRY_MS; CameraLink_Process();
    assert(tx_count==stopped_tx); /* 结束等待后不再补发旧任务。 */
    assert(!TaskProtocol_Vision("VISION,1,RING,2,NaN,160",&v));
    assert(!TaskProtocol_Vision("VISION,1,RING,2,512,160",&v));
    assert(!TaskProtocol_Vision("VISION,1,QR,111+123+111+231",&v));
    assert(!TaskProtocol_Vision("VISION,4294967296,NEAR,1,2",&v));
    assert(!TaskProtocol_Vision("VISION,1,NEAR,1,3",&v));
    CameraLink_Init(&uart,NULL); assert(!CameraLink_Wait(TASK_QR,0,&v));
    puts("PASS: camera result ACK/retry, request/type/target matching, CRLF, oversized frames, no re-entry, TX recovery and coarse-only confirmed NONE");
}
