/**
  ******************************************************************************
  * @file    servo.h
  * @brief   SG90舵机PWM驱动头文件
  * @author  Cloud Platform Team
  * @date    2026-08-21
  ******************************************************************************
  * @attention
  *
  * 舵机型号：SG90
  * 控制信号：PWM 50Hz (20ms周期)
  * 脉宽范围：0.5ms~2.5ms 对应 0°~180°
  *
  * 硬件连接：
  *   - 水平舵机(Yaw)：  TIM2_CH1 (PA15)
  *   - 俯仰舵机(Pitch)：TIM2_CH2 (PB3)
  *
  ******************************************************************************
  */

#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 舵机通道枚举
 */
typedef enum {
    SERVO_YAW = 0,      // 水平舵机（TIM2_CH1）
    SERVO_PITCH = 1,    // 俯仰舵机（TIM2_CH2）
    SERVO_CHANNEL_MAX
} ServoChannel_t;

/* Exported constants --------------------------------------------------------*/

/* 舵机PWM参数 */
#define SERVO_PWM_FREQ          50          // 频率：50Hz
#define SERVO_PWM_PERIOD        20000       // 周期：20ms (20000us)

/* 舵机脉宽参数（单位：微秒） */
#define SERVO_PULSE_MIN         500         // 最小脉宽：0.5ms (0°)
#define SERVO_PULSE_MAX         2500        // 最大脉宽：2.5ms (180°)
#define SERVO_PULSE_CENTER      1500        // 中心脉宽：1.5ms (90°)

/* 舵机角度范围 */
#define SERVO_ANGLE_MIN         0           // 最小角度：0°
#define SERVO_ANGLE_MAX         180         // 最大角度：180°
#define SERVO_ANGLE_CENTER      90          // 中心角度：90°

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化舵机PWM
 * @param  None
 * @retval 0: 成功, -1: 失败
 */
int8_t Servo_Init(void);

/**
 * @brief  设置舵机角度
 * @param  channel: 舵机通道 (SERVO_YAW 或 SERVO_PITCH)
 * @param  angle: 目标角度 (0~180°)
 * @retval 0: 成功, -1: 失败
 */
int8_t Servo_SetAngle(ServoChannel_t channel, uint8_t angle);

/**
 * @brief  设置舵机脉宽（高级功能）
 * @param  channel: 舵机通道
 * @param  pulse_us: 脉宽（单位：微秒，500~2500）
 * @retval 0: 成功, -1: 失败
 */
int8_t Servo_SetPulse(ServoChannel_t channel, uint16_t pulse_us);

/**
 * @brief  停止舵机PWM输出
 * @param  channel: 舵机通道
 * @retval 0: 成功, -1: 失败
 */
int8_t Servo_Stop(ServoChannel_t channel);

/**
 * @brief  停止所有舵机
 * @param  None
 * @retval None
 */
void Servo_StopAll(void);

/**
 * @brief  将所有舵机复位到中心位置（90°）
 * @param  None
 * @retval None
 */
void Servo_ResetToCenter(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
