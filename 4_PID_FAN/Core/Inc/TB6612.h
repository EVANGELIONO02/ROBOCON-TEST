
#ifndef __TB6612_H_
#define __TB6612_H_

#include <stdint.h>
#include "main.h"
#include "gpio.h"

/**
 * @brief 衰减模式枚举
 *
 * 决定电机停止时线圈电流的泄放方式。运行时的斩波衰减方式由接线决定
 * （PWM 在 PWMA 上、IN1/IN2 保持静态电平，关断相即短制动），软件无法改变。
 */
typedef enum {
    SLOW_DECAY,  /**< 慢衰减：两个低边同时导通，电流经桥臂环流，制动 */
    FAST_DECAY   /**< 快衰减：全桥关断，电流经体二极管泄放，滑行 */
} DecayMode;

/**
 * @brief 初始化TB6612
 */
void TB6612_Init(void);

/**
 * @brief 设置衰减模式
 * @param mode 衰减模式
 */
void TB6612_SetDecayMode(DecayMode mode);

/**
 * @brief 控制电机前进
 * @param speed 速度值（0-100）
 */
void TB6612_Forward(uint8_t speed);

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void TB6612_Backward(uint8_t speed);

/**
 * @brief 获取当前衰减模式
 * @return 当前衰减模式
 */
DecayMode TB6612_GetDecayMode(void);

/**
 * @brief 按当前衰减模式停止电机
 *
 * SLOW_DECAY 走短制动，FAST_DECAY 走全桥关断滑行。
 */
void TB6612_Stop(void);

/**
 * @brief 电机刹车
 */
void TB6612_Brake(void);

/**
 * @brief 电机滑行
 */
void TB6612_Coast(void);

#endif /* __TB6612_H_ */
