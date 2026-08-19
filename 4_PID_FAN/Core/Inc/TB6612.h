
#ifndef __TB6612_H_
#define __TB6612_H_

#include <stdint.h>
#include "main.h"
#include "gpio.h"

/**
 * @brief 衰减模式枚举
 */
typedef enum {
    SLOW_DECAY,  /**< 慢衰减模式 */
    FAST_DECAY   /**< 快衰减模式 */
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
 * @brief 电机刹车
 */
void TB6612_Brake(void);

/**
 * @brief 电机滑行
 */
void TB6612_Coast(void);

#endif /* __TB6612_H_ */
