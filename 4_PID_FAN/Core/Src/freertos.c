/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "TB6612.h"
#include "usart.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_FULL_SCALE       4095U
#define MOTOR_SPEED_MAX      100U
#define ADC_CENTER           2048U
#define ADC_DEADBAND         120U
#define VOFA_SEND_PERIOD_MS  50U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile uint32_t g_pot_adc_value = 0U;
static volatile uint8_t g_motor_speed = 0U;
static volatile int8_t g_motor_direction = 0;
static osThreadId_t vofaTelemetryTaskHandle;
static const osThreadAttr_t vofaTelemetryTask_attributes = {
  .name = "vofaTelemetryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE END Variables */
/* Definitions for motorControlTask */
osThreadId_t motorControlTaskHandle;
const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint8_t Potentiometer_ToSpeed(uint32_t adc_value);
static int8_t Potentiometer_ToDirection(uint32_t adc_value, uint8_t speed);
static void Potentiometer_ToMotor(uint32_t adc_value, uint8_t speed);
static void VOFA_SendMotorData(uint32_t adc_value, uint8_t speed,
                               int8_t direction);
static void StartVofaTelemetryTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartMotorControlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motorControlTask */
  motorControlTaskHandle = osThreadNew(StartMotorControlTask, NULL,
                                       &motorControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  vofaTelemetryTaskHandle = osThreadNew(StartVofaTelemetryTask, NULL,
                                        &vofaTelemetryTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
  * @brief  Function implementing the motorControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  uint32_t adc_value;
  uint8_t speed;
  int8_t direction;

  HAL_ADCEx_Calibration_Start(&hadc1);
  TB6612_Init();

  /* Infinite loop */
  for(;;)
  {
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
      if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
      {
        adc_value = HAL_ADC_GetValue(&hadc1);
        speed = Potentiometer_ToSpeed(adc_value);
        direction = Potentiometer_ToDirection(adc_value, speed);

        g_pot_adc_value = adc_value;
        g_motor_speed = speed;
        g_motor_direction = direction;

        Potentiometer_ToMotor(adc_value, speed);
      }

      HAL_ADC_Stop(&hadc1);
    }

    osDelay(10);
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartVofaTelemetryTask */
/**
  * @brief  Function implementing the vofaTelemetryTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartVofaTelemetryTask */
static void StartVofaTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartVofaTelemetryTask */
  uint32_t adc_value;
  uint8_t speed;
  int8_t direction;

  for(;;)
  {
    adc_value = g_pot_adc_value;
    speed = g_motor_speed;
    direction = g_motor_direction;

    VOFA_SendMotorData(adc_value, speed, direction);
    osDelay(VOFA_SEND_PERIOD_MS);
  }
  /* USER CODE END StartVofaTelemetryTask */
}

static uint8_t Potentiometer_ToSpeed(uint32_t adc_value)
{
  uint32_t offset;
  uint32_t effective_range;
  uint32_t half_range;
  uint32_t speed;

  if ((adc_value >= (ADC_CENTER - ADC_DEADBAND)) &&
      (adc_value <= (ADC_CENTER + ADC_DEADBAND)))
  {
    return 0U;
  }

  if (adc_value < ADC_CENTER)
  {
    offset = (ADC_CENTER - ADC_DEADBAND) - adc_value;
    effective_range = ADC_CENTER - ADC_DEADBAND;
  }
  else
  {
    offset = adc_value - (ADC_CENTER + ADC_DEADBAND);
    effective_range = ADC_FULL_SCALE - (ADC_CENTER + ADC_DEADBAND);
  }

  if (effective_range == 0U)
  {
    return 0U;
  }

  half_range = effective_range / 2U;

  if (offset <= half_range)
  {
    speed = (offset * MOTOR_SPEED_MAX * 2U) / effective_range;
  }
  else
  {
    speed = MOTOR_SPEED_MAX
          - (((offset - half_range) * MOTOR_SPEED_MAX * 2U) / effective_range);
  }

  if (speed > MOTOR_SPEED_MAX)
  {
    speed = MOTOR_SPEED_MAX;
  }

  return (uint8_t)speed;
}

static int8_t Potentiometer_ToDirection(uint32_t adc_value, uint8_t speed)
{
  if (speed == 0U)
  {
    return 0;
  }

  return (adc_value < ADC_CENTER) ? -1 : 1;
}

static void Potentiometer_ToMotor(uint32_t adc_value, uint8_t speed)
{
  if (speed == 0U)
  {
    TB6612_Coast();
    return;
  }

  if (adc_value < ADC_CENTER)
  {
    TB6612_Backward(speed);
  }
  else
  {
    TB6612_Forward(speed);
  }
}

static void VOFA_SendMotorData(uint32_t adc_value, uint8_t speed,
                               int8_t direction)
{
  char tx_buffer[40];
  int length = snprintf(tx_buffer, sizeof(tx_buffer), "%lu,%u,%d\r\n",
                        adc_value, speed, direction);

  if (length > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer,
                      (uint16_t)length, 10U);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

