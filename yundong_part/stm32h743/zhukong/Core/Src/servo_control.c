#include "servo_control.h"

#define SERVO_PERIOD_US  20000U
#define SERVO_MIN_US       500U
#define SERVO_MAX_US      2500U

static TIM_HandleTypeDef s_tim1;
static TIM_HandleTypeDef s_tim3;
static bool s_tim1_ready;
static bool s_tim3_ready;
static bool s_gripper_started;
static bool s_turntable_started;
static bool s_base_started;

static bool ServoControl_InitTimer(TIM_HandleTypeDef *timer,
                                   TIM_TypeDef *instance,
                                   uint32_t timer_clock)
{
    TIM_OC_InitTypeDef channel = {0};

    if ((timer_clock < 1000000U) || (timer_clock % 1000000U != 0U))
    {
        return false;
    }
    timer->Instance = instance;
    timer->Init.Prescaler = timer_clock / 1000000U - 1U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = SERVO_PERIOD_US - 1U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK)
    {
        return false;
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (instance == TIM1)
    {
        channel.OCNPolarity = TIM_OCNPOLARITY_HIGH;
        channel.OCIdleState = TIM_OCIDLESTATE_RESET;
        channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        return HAL_TIM_PWM_ConfigChannel(timer, &channel,
                                         TIM_CHANNEL_1) == HAL_OK;
    }
    return (HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_1) == HAL_OK) &&
           (HAL_TIM_PWM_ConfigChannel(timer, &channel, TIM_CHANNEL_3) == HAL_OK);
}

void ServoControl_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_ClkInitTypeDef clocks = {0};
    uint32_t latency;
    uint32_t tim1_clock;
    uint32_t tim3_clock;

    s_gripper_started = false;
    s_turntable_started = false;
    s_base_started = false;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_8;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_8;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* APB1/APB2 are divided by two in SystemClock_Config. With TIMPRE=0,
       each timer receives twice its APB clock. */
    HAL_RCC_GetClockConfig(&clocks, &latency);
    tim1_clock = HAL_RCC_GetPCLK2Freq() *
                 ((clocks.APB2CLKDivider == RCC_APB2_DIV1) ? 1U : 2U);
    tim3_clock = HAL_RCC_GetPCLK1Freq() *
                 ((clocks.APB1CLKDivider == RCC_APB1_DIV1) ? 1U : 2U);
    s_tim1_ready = ServoControl_InitTimer(&s_tim1, TIM1, tim1_clock);
    s_tim3_ready = ServoControl_InitTimer(&s_tim3, TIM3, tim3_clock);
}

bool ServoControl_SetAngle(char channel, uint16_t angle)
{
    TIM_HandleTypeDef *timer;
    uint32_t tim_channel;
    uint16_t max_angle;
    bool *started;
    uint32_t pulse;

    switch (channel)
    {
    case 'G':
        timer = &s_tim3;
        tim_channel = TIM_CHANNEL_3;
        max_angle = 270U;
        started = &s_gripper_started;
        if (!s_tim3_ready) return false;
        break;
    case 'T':
        timer = &s_tim1;
        tim_channel = TIM_CHANNEL_1;
        max_angle = 270U;
        started = &s_turntable_started;
        if (!s_tim1_ready) return false;
        break;
    case 'B':
        timer = &s_tim3;
        tim_channel = TIM_CHANNEL_1;
        max_angle = 360U;
        started = &s_base_started;
        if (!s_tim3_ready) return false;
        break;
    default:
        return false;
    }
    if (angle > max_angle) return false;

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
