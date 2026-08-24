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
  CONTROL_MODE_STOP = 0,
  CONTROL_MODE_SPEED = 1,
  CONTROL_MODE_POSITION = 2
} ControlMode;

typedef struct {
  float kp;
  float ki;
  float kd;
  float dt;             /**< Sample period in seconds. */
  float integral;
  float last_error;
  float integral_limit; /**< Limit on the integral term, in error * s. */
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

#define TARGET_SPEED_MAX_CPS    270000.0f  /* ~400 RPM for MG310 1:20 */
#define PID_OUTPUT_MAX          100.0f
#define ENCODER_COUNTS_PER_REV  40000L
#define POSITION_TOLERANCE_DEG  5U

/* Speed gains are expressed against the full-scale target so they follow
 * TARGET_SPEED_MAX_CPS instead of having to be retuned when it changes.
 * Kp: a full-scale error commands 60%% duty. Ki: integral time 0.25 s.
 * Kd stays 0 - the feedback is a 10 ms count delta, too coarse to differentiate. */
#define SPEED_PID_KP            (0.6f * PID_OUTPUT_MAX / TARGET_SPEED_MAX_CPS)
#define SPEED_PID_KI            (4.0f * SPEED_PID_KP)
#define SPEED_PID_KD            0.00f
/* Sized so the integral term alone can reach full output. Unit: counts. */
#define SPEED_PID_INTEGRAL_LIMIT (PID_OUTPUT_MAX / SPEED_PID_KI)

/* Position error is in counts, unlimited range (can exceed ±180°).
 * Kp scaled down because error magnitude is now ~111x larger (counts vs degrees).
 * Ki pulls out static friction. Kd = 0: raw counts have no quantization noise. */
#define POSITION_PID_KP         0.003f
#define POSITION_PID_KI         0.000f
#define POSITION_PID_KD         0.000f
/* Caps the integral term at 50%% duty. Unit: counts * s. */
#define POSITION_PID_INTEGRAL_LIMIT 15000.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile ControlMode g_control_mode = CONTROL_MODE_STOP;
static volatile uint8_t g_reset_controllers = 1U;

/* InputTask writes these target values. PID tasks read them. */
static volatile int32_t g_target_speed_cps = 0;
static volatile int32_t g_target_position_counts = 0;  /* Unlimited累积角度，单位counts */

/* PID tasks write these feedback/output values. VofaTelemetryTask reads them. */
static volatile int32_t g_feedback_speed_cps = 0;
static volatile int32_t g_feedback_position_counts = 0;  /* 累积counts */
/* Signed: the sign carries the commanded direction, the magnitude the duty. */
static volatile int8_t g_pwm_output = 0;

static uint16_t g_encoder_speed_last_count = 0;
static int32_t g_encoder_position_counts = 0;
static PidController g_speed_pid = {
  .kp = SPEED_PID_KP,
  .ki = SPEED_PID_KI,
  .kd = SPEED_PID_KD,
  .dt = (float)SPEED_PID_PERIOD_MS / 1000.0f,
  .integral = 0.0f,
  .last_error = 0.0f,
  .integral_limit = SPEED_PID_INTEGRAL_LIMIT,
};
static PidController g_position_pid = {
  .kp = POSITION_PID_KP,
  .ki = POSITION_PID_KI,
  .kd = POSITION_PID_KD,
  .dt = (float)POSITION_PID_PERIOD_MS / 1000.0f,
  .integral = 0.0f,
  .last_error = 0.0f,
  .integral_limit = POSITION_PID_INTEGRAL_LIMIT,
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
static GPIO_PinState ModeButton_Read(void);
static int32_t Encoder_UpdatePositionCounts(void);
static int32_t Encoder_DeltaToCountsPerSecond(int32_t delta_count);
static void Pid_Reset(PidController *pid);
static float Pid_Update(PidController *pid, float error);
static uint8_t Pid_OutputToPwm(float output);
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
        /* Position target: map ADC 0-4095 to ±2 revs = ±80000 counts */
        g_target_position_counts =
            (int32_t)(((int64_t)adc_value - 2048) * ENCODER_COUNTS_PER_REV * 2 / 4096);
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

        /* Active-low button: cycle through stop -> speed -> position. */
        if (stable_state == GPIO_PIN_RESET)
        {
          if (g_control_mode == CONTROL_MODE_STOP)
            g_control_mode = CONTROL_MODE_SPEED;
          else if (g_control_mode == CONTROL_MODE_SPEED)
            g_control_mode = CONTROL_MODE_POSITION;
          else
            g_control_mode = CONTROL_MODE_STOP;

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
  /* Coast on stop, as before: FAST_DECAY. Switch to SLOW_DECAY to brake and
   * hold position instead. */
  TB6612_SetDecayMode(FAST_DECAY);
  g_encoder_speed_last_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  g_encoder_position_counts = 0;

  for(;;)
  {
    if (g_reset_controllers != 0U)
    {
      Controllers_ResetAll();
      g_reset_controllers = 0U;
    }

    /* Sampled every cycle in all modes: skipping it would let the counter
     * drift more than one int16 wrap and corrupt the next delta. */
    int32_t feedback_cps =
        Encoder_DeltaToCountsPerSecond(Encoder_UpdatePositionCounts());

    if (g_control_mode == CONTROL_MODE_STOP)
    {
      Motor_Stop();
      Pid_Reset(&g_speed_pid);
      g_feedback_speed_cps = feedback_cps;
    }
    else if (g_control_mode == CONTROL_MODE_SPEED)
    {
      int32_t target_cps = g_target_speed_cps;

      g_feedback_speed_cps = feedback_cps;

      if (target_cps == 0)
      {
        Motor_Stop();
        Pid_Reset(&g_speed_pid);
      }
      else
      {
        float output = Pid_Update(&g_speed_pid,
                                 (float)target_cps - (float)feedback_cps);
        uint8_t pwm = Pid_OutputToPwm(output);

        g_pwm_output = (int8_t)((output < 0.0f) ? -(int8_t)pwm : (int8_t)pwm);
        Motor_SetOutput((output < 0.0f) ? -1 : 1, pwm);
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
    if (g_control_mode == CONTROL_MODE_STOP)
    {
      /* Keep feedback updated for telemetry even when stopped. */
      g_feedback_position_counts = g_encoder_position_counts;
      Pid_Reset(&g_position_pid);
    }
    else if (g_control_mode == CONTROL_MODE_POSITION)
    {
      int32_t position_error;

      /* speedPidTask owns the encoder sampling; read its accumulated count. */
      g_feedback_position_counts = g_encoder_position_counts;

      position_error = g_target_position_counts - g_feedback_position_counts;

      /* Tolerance in counts: 5 degrees = 5 * 40000 / 360 ~= 556 counts */
      if ((position_error >= -556) && (position_error <= 556))
      {
        Motor_Stop();
        Pid_Reset(&g_position_pid);
      }
      else
      {
        /* Signed effort: the sign is the direction, the magnitude the duty. */
        float output = Pid_Update(&g_position_pid, (float)position_error);
        uint8_t pwm = Pid_OutputToPwm(output);

        g_pwm_output = (int8_t)((output < 0.0f) ? -(int8_t)pwm : (int8_t)pwm);
        Motor_SetOutput((output < 0.0f) ? -1 : 1, pwm);
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

static GPIO_PinState ModeButton_Read(void)
{
  return HAL_GPIO_ReadPin(Mode_switch_GPIO_Port, Mode_switch_Pin);
}

static int32_t Encoder_UpdatePositionCounts(void)
{
  uint16_t current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t delta_count = (int16_t)(current_count - g_encoder_speed_last_count);

  g_encoder_speed_last_count = current_count;
  g_encoder_position_counts += (int32_t)delta_count;
  return (int32_t)delta_count;
}

/* Signed: negative means the shaft is turning backwards. */
static int32_t Encoder_DeltaToCountsPerSecond(int32_t delta_count)
{
  return (delta_count * 1000) / (int32_t)SPEED_PID_PERIOD_MS;
}

static void Pid_Reset(PidController *pid)
{
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

/* Returns a signed effort in [-PID_OUTPUT_MAX, PID_OUTPUT_MAX]: the sign is
 * the direction, the magnitude is the PWM duty. Ki and Kd are scaled by dt so
 * they keep their meaning if a task period changes. */
static float Pid_Update(PidController *pid, float error)
{
  float derivative = (error - pid->last_error) / pid->dt;
  float output;

  pid->integral += error * pid->dt;
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

  if (output > PID_OUTPUT_MAX)
  {
    output = PID_OUTPUT_MAX;
  }
  else if (output < -PID_OUTPUT_MAX)
  {
    output = -PID_OUTPUT_MAX;
  }

  return output;
}

static uint8_t Pid_OutputToPwm(float output)
{
  return (uint8_t)((output < 0.0f) ? -output : output);
}

static void Controllers_ResetAll(void)
{
  Pid_Reset(&g_speed_pid);
  Pid_Reset(&g_position_pid);
  g_feedback_speed_cps = 0;
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
  g_pwm_output = 0;
  /* Brake or coast according to the decay mode the driver is configured for. */
  TB6612_Stop();
}

static void Vofa_SendTelemetry(void)
{
  char tx_buffer[96];
  int length = snprintf(tx_buffer, sizeof(tx_buffer),
                        /*
                         * FireWater data columns:
                         * 1 mode, 2 target speed, 3 feedback speed,
                         * 4 PWM output, 5 target position counts,
                         * 6 feedback position counts.
                         */
                        "d: %u,%ld,%ld,%d,%ld,%ld\r\n",
                        (uint8_t)g_control_mode,
                        (long)g_target_speed_cps,
                        (long)g_feedback_speed_cps,
                        g_pwm_output,
                        (long)g_target_position_counts,
                        (long)g_feedback_position_counts);

  if (length > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer,
                            (uint16_t)length, 10U);
  }
}

/* USER CODE END Application */

