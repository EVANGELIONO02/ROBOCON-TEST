/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Mode_switch_Pin GPIO_PIN_0
#define Mode_switch_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* ==================== 设备角色配置 ==================== */
/**
 * @brief 设备角色选择
 * @note 烧录主控端时使用 DEVICE_MASTER
 *       烧录从机端时使用 DEVICE_SLAVE
 *       切换时注释一个，取消注释另一个，然后重新编译
 */
#define DEVICE_MASTER    // 主控端（读取传感器，发送数据）
//#define DEVICE_SLAVE   // 从机端（接收数据，控制舵机）

/* 编译时检查：确保只定义了一个角色 */
#if defined(DEVICE_MASTER) && defined(DEVICE_SLAVE)
    #error "Cannot define both DEVICE_MASTER and DEVICE_SLAVE!"
#endif

#if !defined(DEVICE_MASTER) && !defined(DEVICE_SLAVE)
    #error "Must define either DEVICE_MASTER or DEVICE_SLAVE!"
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
