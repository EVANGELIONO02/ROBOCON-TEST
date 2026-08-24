/**
 * @file TB6612.c
 * @brief TB6612双H桥电机驱动器控制实现
 */

#include <TB6612.h>
#include "tim.h"

#define PWM_A_CH TIM_CHANNEL_1
#define PWM_A_TIM &htim2

/** @brief 最大速度值 */
#define MAX_SPEED 100

/** @brief 当前衰减模式 */
static DecayMode currentDecayMode = SLOW_DECAY;

/**
 * @brief 设置PWMA引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetPWMA(uint8_t duty)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(PWM_A_TIM);
    uint32_t compare = ((uint32_t)duty * (period + 1U)) / MAX_SPEED;

    __HAL_TIM_SET_COMPARE(PWM_A_TIM, PWM_A_CH, compare);
}

/**
 * @brief 初始化TB6612
 */
void TB6612_Init(void)
{
    HAL_TIM_PWM_Start(PWM_A_TIM, PWM_A_CH);
}

/**
 * @brief 设置衰减模式
 * @param mode 衰减模式
 */
void TB6612_SetDecayMode(DecayMode mode)
{
    currentDecayMode = mode;
}

/**
 * @brief 获取当前衰减模式
 * @return 当前衰减模式
 */
DecayMode TB6612_GetDecayMode(void)
{
    return currentDecayMode;
}

/**
 * @brief 控制电机前进
 * @param speed 速度值（0-100）
 */
void TB6612_Forward(uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    /* 零占空比等同于停止，交由衰减模式决定制动还是滑行 */
    if (speed == 0)
    {
        TB6612_Stop();
        return;
    }

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    __SetPWMA(speed);
}

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void TB6612_Backward(uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    /* 零占空比等同于停止，交由衰减模式决定制动还是滑行 */
    if (speed == 0)
    {
        TB6612_Stop();
        return;
    }

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    __SetPWMA(speed);
}

/**
 * @brief 按当前衰减模式停止电机
 *
 * SLOW_DECAY 走短制动，FAST_DECAY 走全桥关断滑行。
 */
void TB6612_Stop(void)
{
    if (currentDecayMode == SLOW_DECAY)
        TB6612_Brake();
    else
        TB6612_Coast();
}

/**
 * @brief 电机刹车
 */
void TB6612_Brake(void)
{
    __SetPWMA(MAX_SPEED);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}

/**
 * @brief 电机滑行
 */
void TB6612_Coast(void)
{
    __SetPWMA(0);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}
