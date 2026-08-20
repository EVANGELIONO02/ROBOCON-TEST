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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "tim.h"
#include "TB6612.h"
#include "usart.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  CONTROL_MODE_SPEED = 0,
  CONTROL_MODE_POSITION = 1
} ControlMode;

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float integral_limit;
} PidController;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_FULL_SCALE          4095U

#define INPUT_PERIOD_MS         10U
#define MODE_SCAN_PERIOD_MS     20U
#define MODE_BUTTON_DEBOUNCE_MS 50U
#define SPEED_PID_PERIOD_MS     10U
#define POSITION_PID_PERIOD_MS  10U
#define VOFA_SEND_PERIOD_MS     50U

#define TARGET_SPEED_MAX_CPS    1000.0f
#define PID_OUTPUT_MAX          100.0f
#define ENCODER_COUNTS_PER_REV  10000L
#define POSITION_TOLERANCE_DEG  2U
#define POSITION_MIN_PWM        1U

#define SPEED_PID_KP            0.00f
#define SPEED_PID_KI            0.00f
#define SPEED_PID_KD            0.00f
#define POSITION_PID_KP         0.40f
#define POSITION_PID_KI         0.00f
#define POSITION_PID_KD         0.02f
#define PID_INTEGRAL_LIMIT      3000.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile ControlMode g_control_mode = CONTROL_MODE_SPEED;
static volatile uint8_t g_reset_controllers = 1U;

/* InputTask writes these target values. PID tasks read them. */
static volatile int32_t g_target_speed_cps = 0;
static volatile uint16_t g_target_position_degrees = 0U;

/* PID tasks write these feedback/output values. VofaTelemetryTask reads them. */
static volatile int32_t g_feedback_speed_cps = 0;
static volatile uint16_t g_feedback_position_degrees = 0U;
static volatile uint8_t g_pwm_output = 0U;

static int16_t g_encoder_speed_last_count = 0;
static int32_t g_encoder_position_counts = 0;
static PidController g_speed_pid = {
  .kp = SPEED_PID_KP,
  .ki = SPEED_PID_KI,
  .kd = SPEED_PID_KD,
  .integral = 0.0f,
  .last_error = 0.0f,
  .integral_limit = PID_INTEGRAL_LIMIT,
};
static PidController g_position_pid = {
  .kp = POSITION_PID_KP,
  .ki = POSITION_PID_KI,
  .kd = POSITION_PID_KD,
  .integral = 0.0f,
  .last_error = 0.0f,
  .integral_limit = PID_INTEGRAL_LIMIT,
};

/* USER CODE END Variables */
/* Definitions for inputTask */
osThreadId_t inputTaskHandle;
const osThreadAttr_t inputTask_attributes = {
  .name = "inputTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for modeSwitchTask */
osThreadId_t modeSwitchTaskHandle;
const osThreadAttr_t modeSwitchTask_attributes = {
  .name = "modeSwitchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for speedPidTask */
osThreadId_t speedPidTaskHandle;
const osThreadAttr_t speedPidTask_attributes = {
  .name = "speedPidTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for positionPidTask */
osThreadId_t positionPidTaskHandle;
const osThreadAttr_t positionPidTask_attributes = {
  .name = "positionPidTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vofaTelemetryTask */
osThreadId_t vofaTelemetryTaskHandle;
const osThreadAttr_t vofaTelemetryTask_attributes = {
  .name = "vofaTelemetryTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static int32_t Adc_ToTargetSpeedCountsPerSecond(uint32_t adc_value);
static uint16_t Adc_ToTargetPositionDegrees(uint32_t adc_value);
static GPIO_PinState ModeButton_Read(void);
static int16_t Encoder_UpdatePositionCounts(void);
static uint16_t Encoder_CountsToDegrees(int32_t counts);
static int32_t Encoder_DeltaToCountsPerSecond(int16_t delta_count);
static void Pid_Reset(PidController *pid);
static uint8_t Pid_Update(PidController *pid, float target, float feedback);
static float Position_GetShortestErrorDegrees(uint16_t target,
                                              uint16_t current);
static void Controllers_ResetAll(void);
static void Motor_SetOutput(int8_t direction, uint8_t speed);
static void Motor_Stop(void);
static void Vofa_SendTelemetry(void);

/* USER CODE END FunctionPrototypes */

void StartInputTask(void *argument);
void StartModeSwitchTask(void *argument);
void StartSpeedPidTask(void *argument);
void StartPositionPidTask(void *argument);
void StartVofaTelemetryTask(void *argument);

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of inputTask */
  inputTaskHandle = osThreadNew(StartInputTask, NULL, &inputTask_attributes);

  /* creation of modeSwitchTask */
  modeSwitchTaskHandle = osThreadNew(StartModeSwitchTask, NULL, &modeSwitchTask_attributes);

  /* creation of speedPidTask */
  speedPidTaskHandle = osThreadNew(StartSpeedPidTask, NULL, &speedPidTask_attributes);

  /* creation of positionPidTask */
  positionPidTaskHandle = osThreadNew(StartPositionPidTask, NULL, &positionPidTask_attributes);

  /* creation of vofaTelemetryTask */
  vofaTelemetryTaskHandle = osThreadNew(StartVofaTelemetryTask, NULL, &vofaTelemetryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInputTask */
/**
  * @brief  Reads potentiometer input and publishes target commands.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInputTask */
void StartInputTask(void *argument)
{
  /* USER CODE BEGIN StartInputTask */
  uint32_t adc_value;

  HAL_ADCEx_Calibration_Start(&hadc1);

  for(;;)
  {
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
      if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
      {
        adc_value = HAL_ADC_GetValue(&hadc1);
        g_target_speed_cps = Adc_ToTargetSpeedCountsPerSecond(adc_value);
        g_target_position_degrees = Adc_ToTargetPositionDegrees(adc_value);
      }

      HAL_ADC_Stop(&hadc1);
    }

    osDelay(INPUT_PERIOD_MS);
  }
  /* USER CODE END StartInputTask */
}

/* USER CODE BEGIN Header_StartModeSwitchTask */
/**
  * @brief  Reads PA4 and publishes the active control mode.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartModeSwitchTask */
void StartModeSwitchTask(void *argument)
{
  /* USER CODE BEGIN StartModeSwitchTask */
  GPIO_PinState last_raw_state = GPIO_PIN_SET;
  GPIO_PinState stable_state = GPIO_PIN_SET;
  uint32_t last_debounce_tick = HAL_GetTick();

  for(;;)
  {
    GPIO_PinState raw_state = ModeButton_Read();
    uint32_t now = HAL_GetTick();

    if (raw_state != last_raw_state)
    {
      last_debounce_tick = now;
      last_raw_state = raw_state;
    }

    if ((now - last_debounce_tick) >= MODE_BUTTON_DEBOUNCE_MS)
    {
      if (stable_state != raw_state)
      {
        stable_state = raw_state;

        /* Active-low button: toggle mode on each stable press event. */
        if (stable_state == GPIO_PIN_RESET)
        {
          g_control_mode = (g_control_mode == CONTROL_MODE_SPEED)
                         ? CONTROL_MODE_POSITION
                         : CONTROL_MODE_SPEED;
          g_reset_controllers = 1U;
        }
      }
    }

    osDelay(MODE_SCAN_PERIOD_MS);
  }
  /* USER CODE END StartModeSwitchTask */
}

/* USER CODE BEGIN Header_StartSpeedPidTask */
/**
  * @brief  Runs speed PID and owns motor output in speed mode.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSpeedPidTask */
void StartSpeedPidTask(void *argument)
{
  /* USER CODE BEGIN StartSpeedPidTask */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  TB6612_Init();
  g_encoder_speed_last_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  g_encoder_position_counts = 0;

  for(;;)
  {
    if (g_reset_controllers != 0U)
    {
      Controllers_ResetAll();
      g_reset_controllers = 0U;
    }

    if (g_control_mode == CONTROL_MODE_SPEED)
    {
      int32_t target_cps = g_target_speed_cps;

      if (target_cps == 0)
      {
        Motor_Stop();
        Pid_Reset(&g_speed_pid);
        g_feedback_speed_cps = 0;
      }
      else
      {
        int32_t feedback_cps =
            Encoder_DeltaToCountsPerSecond(Encoder_UpdatePositionCounts());
        uint8_t pwm = Pid_Update(&g_speed_pid, (float)target_cps,
                                 (float)feedback_cps);

        g_feedback_speed_cps = feedback_cps;
        g_pwm_output = pwm;
        Motor_SetOutput(1, pwm);
      }
    }

    osDelay(SPEED_PID_PERIOD_MS);
  }
  /* USER CODE END StartSpeedPidTask */
}

/* USER CODE BEGIN Header_StartPositionPidTask */
/**
 * @brief  Runs position PID using the ADC target and encoder feedback.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPositionPidTask */
void StartPositionPidTask(void *argument)
{
  /* USER CODE BEGIN StartPositionPidTask */
  for(;;)
  {
    if (g_control_mode == CONTROL_MODE_POSITION)
    {
      float position_error;
      uint8_t pwm;

      Encoder_UpdatePositionCounts();
      g_feedback_position_degrees =
          Encoder_CountsToDegrees(g_encoder_position_counts);
      g_feedback_speed_cps = 0;

      position_error = Position_GetShortestErrorDegrees(
          g_target_position_degrees, g_feedback_position_degrees);

      if ((position_error >= -(float)POSITION_TOLERANCE_DEG) &&
          (position_error <= (float)POSITION_TOLERANCE_DEG))
      {
        Motor_Stop();
        Pid_Reset(&g_position_pid);
      }
      else
      {
        /* The sign selects direction; PID controls the PWM magnitude. */
        pwm = Pid_Update(&g_position_pid,
                         (position_error < 0.0f) ? -position_error : position_error,
                         0.0f);
        if (pwm < POSITION_MIN_PWM)
        {
          pwm = POSITION_MIN_PWM;
        }

        g_pwm_output = pwm;
        Motor_SetOutput((position_error < 0.0f) ? -1 : 1, pwm);
      }
    }

    osDelay(POSITION_PID_PERIOD_MS);
  }
  /* USER CODE END StartPositionPidTask */
}

/* USER CODE BEGIN Header_StartVofaTelemetryTask */
/**
  * @brief  Sends control data to VOFA+ through USART1.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartVofaTelemetryTask */
void StartVofaTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartVofaTelemetryTask */
  for(;;)
  {
    Vofa_SendTelemetry();
    osDelay(VOFA_SEND_PERIOD_MS);
  }
  /* USER CODE END StartVofaTelemetryTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static int32_t Adc_ToTargetSpeedCountsPerSecond(uint32_t adc_value)
{
  if (adc_value > ADC_FULL_SCALE)
  {
    adc_value = ADC_FULL_SCALE;
  }

  return (int32_t)((adc_value * TARGET_SPEED_MAX_CPS) / ADC_FULL_SCALE);
}

static uint16_t Adc_ToTargetPositionDegrees(uint32_t adc_value)
{
  if (adc_value > ADC_FULL_SCALE)
  {
    adc_value = ADC_FULL_SCALE;
  }

  return (uint16_t)((adc_value * 360U) / ADC_FULL_SCALE);
}

static GPIO_PinState ModeButton_Read(void)
{
  return HAL_GPIO_ReadPin(Mode_switch_GPIO_Port, Mode_switch_Pin);
}

static int16_t Encoder_UpdatePositionCounts(void)
{
  int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t delta_count = current_count - g_encoder_speed_last_count;

  g_encoder_speed_last_count = current_count;
  g_encoder_position_counts += delta_count;
  return delta_count;
}

static uint16_t Encoder_CountsToDegrees(int32_t counts)
{
  int32_t angle_counts = counts % ENCODER_COUNTS_PER_REV;

  if (angle_counts < 0)
  {
    angle_counts += ENCODER_COUNTS_PER_REV;
  }

  return (uint16_t)((angle_counts * 360L) / ENCODER_COUNTS_PER_REV);
}

static int32_t Encoder_DeltaToCountsPerSecond(int16_t delta_count)
{
  if (delta_count < 0)
  {
    delta_count = -delta_count;
  }

  return ((int32_t)delta_count * 1000) / (int32_t)SPEED_PID_PERIOD_MS;
}

static void Pid_Reset(PidController *pid)
{
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

static uint8_t Pid_Update(PidController *pid, float target, float feedback)
{
  float error = target - feedback;
  float derivative = error - pid->last_error;
  float output;

  pid->integral += error;
  if (pid->integral > pid->integral_limit)
  {
    pid->integral = pid->integral_limit;
  }
  else if (pid->integral < -pid->integral_limit)
  {
    pid->integral = -pid->integral_limit;
  }

  pid->last_error = error;
  output = (pid->kp * error)
         + (pid->ki * pid->integral)
         + (pid->kd * derivative);

  if (output < 0.0f)
  {
    output = 0.0f;
  }
  else if (output > PID_OUTPUT_MAX)
  {
    output = PID_OUTPUT_MAX;
  }

  return (uint8_t)output;
}

static float Position_GetShortestErrorDegrees(uint16_t target,
                                              uint16_t current)
{
  float error = (float)target - (float)current;

  if (error > 180.0f)
  {
    error -= 360.0f;
  }
  else if (error < -180.0f)
  {
    error += 360.0f;
  }

  return error;
}

static void Controllers_ResetAll(void)
{
  Pid_Reset(&g_speed_pid);
  Pid_Reset(&g_position_pid);
  g_feedback_speed_cps = 0;
  g_pwm_output = 0U;
  Motor_Stop();
}

static void Motor_SetOutput(int8_t direction, uint8_t speed)
{
  if ((direction == 0) || (speed == 0U))
  {
    Motor_Stop();
  }
  else if (direction < 0)
  {
    TB6612_Backward(speed);
  }
  else
  {
    TB6612_Forward(speed);
  }
}

static void Motor_Stop(void)
{
  g_pwm_output = 0U;
  TB6612_Coast();
}

static void Vofa_SendTelemetry(void)
{
  char tx_buffer[96];
  int length = snprintf(tx_buffer, sizeof(tx_buffer),
                        /*
                         * FireWater data columns:
                         * 1 mode, 2 target speed, 3 feedback speed,
                         * 4 PWM output, 5 target position degrees,
                         * 6 feedback position degrees.
                         */
                        "d: %u,%ld,%ld,%u,%u,%u\r\n",
                        (uint8_t)g_control_mode,
                        (long)g_target_speed_cps,
                        (long)g_feedback_speed_cps,
                        g_pwm_output,
                        g_target_position_degrees,
                        g_feedback_position_degrees);

  if (length > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer,
                            (uint16_t)length, 10U);
  }
}

/* USER CODE END Application */

