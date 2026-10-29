#include "robot_config.h"
/* MaixCAM 串口链路：发送视觉任务，前台从接收缓冲中还原并校验视觉帧。
 * 图像识别在相机端完成，STM32 使用识别结果执行任务决策和位置修正。 */
#include "camera_link.h"
#include "mission_config.h"
#include <stdio.h>
/* UART4 中断收字节；前台组帧后交给顺序流程的等待标志。ESP32 使用 UART5。 */
static UART_HandleTypeDef *s_uart;
static uint8_t s_byte;
static volatile uint8_t s_buffer[CAMERA_RX_SIZE];
static volatile uint16_t s_head, s_tail;
static volatile bool s_fault, s_restart;
static char s_frame[96];
static unsigned s_length;
static bool s_drop;
static CameraService s_service;
static uint32_t s_request;
static uint32_t s_ack_request;
static uint32_t s_sent_at;
static TaskVision *s_result;
static TaskVisionKind s_kind;
static unsigned s_target;
static bool s_allow_none;
static uint32_t s_wait_started;
static void arm_receive(void)
{
    if (s_uart == NULL) return;
    HAL_StatusTypeDef result = HAL_UART_Receive_IT(s_uart, &s_byte, 1U);
    s_restart = result != HAL_OK && result != HAL_BUSY;
}
void CameraLink_Init(UART_HandleTypeDef *uart,CameraService service)
{
    s_uart = uart; s_head = s_tail = 0U; s_fault = false;
    s_service=service; s_request=0; s_ack_request=0; s_result=NULL;
    s_length = 0U; s_drop = false; s_restart = false;
    if (uart != NULL) {
        HAL_NVIC_SetPriority(UART4_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
        arm_receive();
    }
}
/* 每个新任务使用新编号；补发保持编号，结果和确认都匹配该编号。 */
static bool send_request(TaskVisionKind kind,unsigned target,bool repeat)
{
    static const char *names[]={"QR","RING","IDLE","TABLE","OCCUPIED","PICK","NEAR"};
    if(!s_uart||(unsigned)kind>TASK_NEAR) return false;
    if(kind!=TASK_QR&&kind!=TASK_NONE&&!TaskProtocol_Target(kind,target)) return false;
    if(!repeat&&++s_request==0) s_request=1;
    char frame[80]; int n;
    if(kind==TASK_QR||kind==TASK_NONE)
        n=snprintf(frame,sizeof(frame),"CAM,%lu,%s\n",(unsigned long)s_request,names[kind]);
    else if(kind==TASK_TABLE||kind==TASK_OCCUPIED)
        n=snprintf(frame,sizeof(frame),"CAM,%lu,%s,%u,%.1f,%.1f\n",(unsigned long)s_request,names[kind],target,
            (double)MISSION_TARGET_X,(double)MISSION_TARGET_Y);
    else n=snprintf(frame,sizeof(frame),"CAM,%lu,%s,%u\n",(unsigned long)s_request,names[kind],target);
    s_sent_at=HAL_GetTick();
    return n>0&&(size_t)n<sizeof(frame)&&
        HAL_UART_Transmit(s_uart,(uint8_t *)frame,(uint16_t)n,UART_TX_TIMEOUT_MS)==HAL_OK;
}
static void discard(void)
{
    uint32_t interrupts = __get_PRIMASK(); __disable_irq();
    s_tail = s_head; s_fault = false; __set_PRIMASK(interrupts);
    s_length = 0U; s_drop = false;
}
/* 唯一等待入口，结果直接写入调用方变量；等待中继续服务串口和 IMU。 */
static bool wait_result(TaskVisionKind kind,unsigned target,TaskVision *result,bool allow_none)
{
    if(!result||!s_service||s_result||kind==TASK_NONE) return false;
    discard(); s_kind=kind; s_target=target; s_result=result;
    s_allow_none=allow_none; s_wait_started=HAL_GetTick();
    if(!send_request(kind,target,false)) { s_result=NULL; return false; }
    while(s_result) s_service();
    return true;
}
bool CameraLink_Wait(TaskVisionKind kind,unsigned target,TaskVision *result)
{
    return wait_result(kind,target,result,false);
}
bool CameraLink_Observe(TaskVisionKind kind,unsigned target,TaskVision *result)
{
    if(kind!=TASK_TABLE&&kind!=TASK_OCCUPIED&&kind!=TASK_RING) return false;
    return wait_result(kind,target,result,true);
}
void CameraLink_Idle(void)
{
    (void)send_request(TASK_NONE,0,false);
}
static void acknowledge(uint32_t request)
{
    char frame[24];
    int n=snprintf(frame,sizeof(frame),"ACK,%lu\n",(unsigned long)request);
    if(n>0&&(size_t)n<sizeof(frame))
        (void)HAL_UART_Transmit(s_uart,(uint8_t *)frame,(uint16_t)n,UART_TX_TIMEOUT_MS);
}
static void accept_frame(void)
{
    TaskVision v={0};
    if(!TaskProtocol_Vision(s_frame,&v)) return;
    /* 确认丢失时，相机仍会回复；再次确认，不重复交付结果。 */
    if(s_ack_request&&v.request==s_ack_request) { acknowledge(v.request); return; }
    if(!s_result||v.request!=s_request) return;
    if(v.kind==TASK_NONE) {
        if(!s_allow_none||(uint32_t)(HAL_GetTick()-s_wait_started)<MISSION_ALIGN_NONE_SETTLE_MS) return;
    } else if(v.kind!=s_kind||(v.kind!=TASK_QR&&v.target!=s_target)) return;
    *s_result=v; s_result=NULL;
    s_ack_request=v.request;
    acknowledge(v.request);
}
bool CameraLink_RxCallback(UART_HandleTypeDef *uart)
{
    if (s_uart == NULL || uart != s_uart) return false;
    uint16_t next = (uint16_t)((s_head + 1U) % CAMERA_RX_SIZE);
    if (next == s_tail) s_fault = true;
    else { s_buffer[s_head] = s_byte; s_head = next; }
    arm_receive(); return true;
}
bool CameraLink_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (s_uart == NULL || uart != s_uart) return false;
    s_fault = true;
    __HAL_UART_CLEAR_OREFLAG(uart); __HAL_UART_CLEAR_NEFLAG(uart); __HAL_UART_CLEAR_FEFLAG(uart);
    arm_receive(); return true;
}
/* 前台处理接收缓存和帧边界，避免在 UART 接收中断中执行上层任务逻辑。 */
void CameraLink_Process(void)
{
    if (s_uart == NULL) return;
    if (s_restart || s_uart->RxState == HAL_UART_STATE_READY) arm_receive();
    if (s_fault) { discard(); return; }
    while (s_tail != s_head) {
        if (s_fault) { discard(); return; }
        char c = (char)s_buffer[s_tail]; s_tail = (uint16_t)((s_tail + 1U) % CAMERA_RX_SIZE);
        if (c == '\n') {
            if (!s_drop) {
                if (s_length > 0U && s_frame[s_length - 1U] == '\r') --s_length;
                s_frame[s_length] = '\0';
                accept_frame();
            }
            s_length = 0U; s_drop = false;
        } else if (!s_drop) {
            if (s_length < sizeof(s_frame) - 1U) s_frame[s_length++] = c;
            else s_drop = true;
        }
    }
    /* 相机晚启动或漏掉请求时自行恢复；编号和 NONE 观察起点保持不变。
     * 无回复仍持续等待，不据此移动底盘；补发失败留待下次重试。 */
    if(s_result&&(uint32_t)(HAL_GetTick()-s_sent_at)>=CAMERA_REQUEST_RETRY_MS)
        (void)send_request(s_kind,s_target,true);
}
