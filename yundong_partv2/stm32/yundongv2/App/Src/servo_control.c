#include "robot_config.h"
/* 舵机硬件层：TIM1/TIM3 输出 PWM，TIM6 每 20 ms 推进 G/T/B 三路独立轨迹。
 * 前台与中断共享的轨迹状态通过短临界区保护；中断内不得进行电机串口等待。 */
#include "servo_control.h"
#include "servo_trajectory.h"
#include <string.h>

/* 舵机 PWM：20 ms 周期即 50 Hz，脉宽 500~2500 us 对应角度范围。
 * G 夹子 PC8/TIM3_CH3，T 转盘 PA8/TIM1_CH1，B 基座 PC6/TIM3_CH1。 */

static TIM_HandleTypeDef *s_tim1;
static TIM_HandleTypeDef *s_tim3;
static bool s_tim1_ready;
static bool s_tim3_ready;
static bool s_gripper_started;
static bool s_turntable_started;
static bool s_base_started;
static TIM_HandleTypeDef s_scheduler;
static bool s_scheduler_ready;
static ServoTrajectory s_jobs[3]; /* G, T, B; accessed under interrupt exclusion. */
static int servo_index(char channel)
{
    return channel=='G'?0:channel=='T'?1:channel=='B'?2:-1;
}
static bool servo_write(char channel,uint16_t angle,bool allow_start);

/* TIM1/TIM3 由 CubeMX 初始化一次；这里只绑定句柄、启动 TIM6 轨迹时钟。 */
bool ServoControl_Init(TIM_HandleTypeDef *tim1,TIM_HandleTypeDef *tim3)
{
    RCC_ClkInitTypeDef clocks = {0};
    uint32_t latency;
    uint32_t tim3_clock;

    if(!tim1||!tim3||tim1->Instance!=TIM1||tim3->Instance!=TIM3) return false;
    s_tim1=tim1;
    s_tim3=tim3;
    s_gripper_started = false;
    s_turntable_started = false;
    s_base_started = false;
    s_scheduler_ready=false;
    memset(s_jobs,0,sizeof(s_jobs));
    /* APB1/APB2 are divided by two in SystemClock_Config. With TIMPRE=0,
       each timer receives twice its APB clock. */
    HAL_RCC_GetClockConfig(&clocks, &latency);
    tim3_clock = HAL_RCC_GetPCLK1Freq() *
                 ((clocks.APB1CLKDivider == RCC_APB1_DIV1) ? 1U : 2U);
    uint32_t tim1_clock=HAL_RCC_GetPCLK2Freq() *
        ((clocks.APB2CLKDivider==RCC_APB2_DIV1)?1U:2U);
    s_tim1_ready=(tim1_clock/(tim1->Init.Prescaler+1U)==1000000U)&&
        tim1->Init.Period==SERVO_PERIOD_US-1U;
    s_tim3_ready=(tim3_clock/(tim3->Init.Prescaler+1U)==1000000U)&&
        tim3->Init.Period==SERVO_PERIOD_US-1U;
    /* TIM1/TIM3 retain the existing PWM outputs; TIM6 only clocks trajectories.
     * No UART access, motor polling or delay is allowed in this interrupt. */
    if(!s_tim1_ready||!s_tim3_ready) return false;
    __HAL_RCC_TIM6_CLK_ENABLE();
    s_scheduler.Instance=TIM6;
    s_scheduler.Init.Prescaler=tim3_clock/1000000U-1U;
    s_scheduler.Init.CounterMode=TIM_COUNTERMODE_UP;
    s_scheduler.Init.Period=SERVO_PERIOD_US-1U;
    s_scheduler.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&s_scheduler)!=HAL_OK) return false;
    __HAL_TIM_CLEAR_FLAG(&s_scheduler,TIM_FLAG_UPDATE);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,6U,0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    s_scheduler_ready=HAL_TIM_Base_Start_IT(&s_scheduler)==HAL_OK;
    return s_scheduler_ready;
}

/* G/T 使用 0~270°，B 使用 0~360°；先校验再线性换算 PWM 脉宽。
 * 每个通道首次收到角度才启动 PWM，避免上电初始化就改变舵机姿态。 */
static bool servo_write(char channel, uint16_t angle,bool allow_start)
{
    TIM_HandleTypeDef *timer;
    uint32_t tim_channel;
    uint16_t max_angle;
    bool *started;
    uint32_t pulse;

    switch (channel)
    {
    case 'G':
        timer = s_tim3;
        tim_channel = TIM_CHANNEL_3;
        max_angle = SERVO_GRIPPER_MAX_ANGLE;
        started = &s_gripper_started;
        if (!s_tim3_ready) return false;
        break;
    case 'T':
        timer = s_tim1;
        tim_channel = TIM_CHANNEL_1;
        max_angle = SERVO_TURNTABLE_MAX_ANGLE;
        started = &s_turntable_started;
        if (!s_tim1_ready) return false;
        break;
    case 'B':
        timer = s_tim3;
        tim_channel = TIM_CHANNEL_1;
        max_angle = SERVO_BASE_MAX_ANGLE;
        started = &s_base_started;
        if (!s_tim3_ready) return false;
        break;
    default:
        return false;
    }
    if (angle > max_angle) return false;
    if(!*started&&!allow_start) return false;

    /* pulse = 500 + round(angle / max_angle × 2000)，单位 us。 */
    pulse = SERVO_MIN_US + ((uint32_t)angle *
            (SERVO_MAX_US - SERVO_MIN_US) + max_angle / 2U) / max_angle;
    __HAL_TIM_SET_COMPARE(timer, tim_channel, pulse);
    if (!*started)
    {
        if (HAL_TIM_PWM_Start(timer, tim_channel) != HAL_OK)
        {
            return false;
        }
        *started = true;
    }
    return true;
}

bool ServoControl_SetAngle(char channel,uint16_t angle)
{
    int i=servo_index(channel); if(i<0) return false;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    bool ok=servo_write(channel,angle,true);
    if(ok) ServoTrajectory_Set(&s_jobs[i],angle);
    __set_PRIMASK(irq); return ok;
}
/* 只登记轨迹，不等待舵机运动；必须先有已知输出角，才能从当前角平滑插值。 */
bool ServoControl_MoveTo(char channel,uint16_t target,uint32_t duration_ms)
{
    int i=servo_index(channel);
    if(i<0||target>(channel=='B'?SERVO_BASE_MAX_ANGLE:channel=='T'?SERVO_TURNTABLE_MAX_ANGLE:SERVO_GRIPPER_MAX_ANGLE)||!s_scheduler_ready) return false;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    bool ok=s_jobs[i].known;
    if(ok) {
        ServoTrajectory_Start(&s_jobs[i],target,duration_ms,HAL_GetTick());
        ok=servo_write(channel,s_jobs[i].angle,false);
        if(!ok) s_jobs[i].active=false;
    }
    __set_PRIMASK(irq); return ok;
}
/* 返回最近计算的 PWM 指令角度，不是读取舵机的机械位置。 */
bool ServoControl_GetAngle(char channel,uint16_t *angle)
{
    int i=servo_index(channel); if(i<0||!angle) return false;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    bool known=s_jobs[i].known;
    if(known) *angle=s_jobs[i].angle;
    __set_PRIMASK(irq); return known;
}
void ServoControl_Cancel(char channel)
{
    int i=servo_index(channel); if(i<0) return;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    /* Hold the most recently output angle; cancellation does not jump to target. */
    s_jobs[i].active=false;
    __set_PRIMASK(irq);
}
/* 中断中只推进三路轨迹并更新 PWM 比较值；不等待串口、不处理整条任务流程。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if(timer->Instance!=TIM6) return;
    uint32_t now=HAL_GetTick();
    for(unsigned i=0;i<3U;++i) if(s_jobs[i].active)
        (void)servo_write("GTB"[i],ServoTrajectory_Update(&s_jobs[i],now),false);
}
void ServoControl_TimerIRQ(void) { HAL_TIM_IRQHandler(&s_scheduler); }
