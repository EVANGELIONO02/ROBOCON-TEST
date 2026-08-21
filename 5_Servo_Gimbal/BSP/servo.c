/**
  ******************************************************************************
  * @file    servo.c
  * @brief   SG90舵机PWM驱动实现
  * @author  Cloud Platform Team
  * @date    2026-08-21
  ******************************************************************************
  * @attention
  *
  * 使用TIM2的两个通道驱动SG90舵机
  * 定时器配置：PSC=72-1, ARR=20000-1, 输出50Hz PWM
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "servo.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// 舵机状态标志
static uint8_t servo_initialized = 0;

/* Private function prototypes -----------------------------------------------*/
static uint16_t Servo_AngleToPulse(uint8_t angle);
static uint32_t Servo_GetTimerChannel(ServoChannel_t channel);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  将角度转换为PWM脉宽
 * @param  angle: 角度值 (0~180°)
 * @retval 脉宽值（微秒）
 */
static uint16_t Servo_AngleToPulse(uint8_t angle)
{
    // 限幅
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }

    // 线性映射：0°→500us, 180°→2500us
    // pulse = 500 + (angle / 180.0) * 2000
    uint16_t pulse = SERVO_PULSE_MIN +
                     ((uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / SERVO_ANGLE_MAX;

    return pulse;
}

/**
 * @brief  获取定时器通道
 * @param  channel: 舵机通道枚举
 * @retval HAL定时器通道宏
 */
static uint32_t Servo_GetTimerChannel(ServoChannel_t channel)
{
    switch(channel) {
        case SERVO_YAW:
            return TIM_CHANNEL_1;   // PA15
        case SERVO_PITCH:
            return TIM_CHANNEL_2;   // PB3
        default:
            return TIM_CHANNEL_1;
    }
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化舵机PWM
 * @param  None
 * @retval 0: 成功, -1: 失败
 */
int8_t Servo_Init(void)
{
    HAL_StatusTypeDef status;

    // 启动TIM2的两个PWM通道
    status = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    if (status != HAL_OK) {
        return -1;
    }

    status = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    if (status != HAL_OK) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // 回滚
        return -1;
    }

    // 初始化到中心位置（90°）
    Servo_SetAngle(SERVO_YAW, SERVO_ANGLE_CENTER);
    Servo_SetAngle(SERVO_PITCH, SERVO_ANGLE_CENTER);

    servo_initialized = 1;

    return 0;
}

/**
 * @brief  设置舵机角度
 * @param  channel: 舵机通道 (SERVO_YAW 或 SERVO_PITCH)
 * @param  angle: 目标角度 (0~180°)
 * @retval 0: 成功, -1: 失败
 */
int8_t Servo_SetAngle(ServoChannel_t channel, uint8_t angle)
{
    // 参数检查
    if (channel >= SERVO_CHANNEL_MAX) {
        return -1;
    }

    // 角度限幅
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }

    // 转换为脉宽
    uint16_t pulse_us = Servo_AngleToPulse(angle);

    // 设置脉宽
    return Servo_SetPulse(channel, pulse_us);
}

/**
 * @brief  设置舵机脉宽（高级功能）
 * @param  channel: 舵机通道
 * @param  pulse_us: 脉宽（单位：微秒，500~2500）
 * @retval 0: 成功, -1: 失败
 */
int8_t Servo_SetPulse(ServoChannel_t channel, uint16_t pulse_us)
{
    // 参数检查
    if (channel >= SERVO_CHANNEL_MAX) {
        return -1;
    }

    // 脉宽限幅
    if (pulse_us < SERVO_PULSE_MIN) {
        pulse_us = SERVO_PULSE_MIN;
    } else if (pulse_us > SERVO_PULSE_MAX) {
        pulse_us = SERVO_PULSE_MAX;
    }

    // 获取定时器通道
    uint32_t tim_channel = Servo_GetTimerChannel(channel);

    // 设置PWM占空比
    // TIM2的ARR=19999, 所以CCR值直接等于脉宽（微秒）
    __HAL_TIM_SET_COMPARE(&htim2, tim_channel, pulse_us);

    return 0;
}

/**
 * @brief  停止舵机PWM输出
 * @param  channel: 舵机通道
 * @retval 0: 成功, -1: 失败
 */
int8_t Servo_Stop(ServoChannel_t channel)
{
    if (channel >= SERVO_CHANNEL_MAX) {
        return -1;
    }

    uint32_t tim_channel = Servo_GetTimerChannel(channel);

    HAL_TIM_PWM_Stop(&htim2, tim_channel);

    return 0;
}

/**
 * @brief  停止所有舵机
 * @param  None
 * @retval None
 */
void Servo_StopAll(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    servo_initialized = 0;
}

/**
 * @brief  将所有舵机复位到中心位置（90°）
 * @param  None
 * @retval None
 */
void Servo_ResetToCenter(void)
{
    Servo_SetAngle(SERVO_YAW, SERVO_ANGLE_CENTER);
    Servo_SetAngle(SERVO_PITCH, SERVO_ANGLE_CENTER);
}
