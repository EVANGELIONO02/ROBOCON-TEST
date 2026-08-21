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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 控制模式枚举 */
typedef enum {
    MODE_POTENTIOMETER = 0x01,  // 电位器控制模式
    MODE_GYROSCOPE = 0x02       // 陀螺仪控制模式
} ControlMode_t;

/* 控制数据结构 */
typedef struct {
    uint8_t mode;      // 控制模式
    uint8_t yaw;       // 水平角度 (0-180)
    uint8_t pitch;     // 俯仰角度 (0-180)
    uint8_t checksum;  // 校验和
} ControlData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 蓝牙数据包协议 */
#define BT_FRAME_HEADER    0xAA
#define BT_FRAME_TAIL      0x55
#define BT_PACKET_SIZE     6

/* 舵机角度范围 */
#define SERVO_ANGLE_MIN    0
#define SERVO_ANGLE_MAX    180

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ==================== 主控端变量 ==================== */
#ifdef DEVICE_MASTER

/* 控制模式 */
static ControlMode_t g_control_mode = MODE_POTENTIOMETER;

/* ADC数据 */
static uint16_t g_adc_yaw_raw = 0;      // 水平电位器ADC原始值
static uint16_t g_adc_pitch_raw = 0;    // 俯仰电位器ADC原始值
static uint8_t g_adc_yaw_angle = 90;    // 水平角度
static uint8_t g_adc_pitch_angle = 90;  // 俯仰角度

/* MPU6050数据 */
static float g_mpu_yaw = 0.0f;          // 陀螺仪水平角度
static float g_mpu_pitch = 0.0f;        // 陀螺仪俯仰角度
static uint8_t g_mpu_yaw_angle = 90;
static uint8_t g_mpu_pitch_angle = 90;

/* 任务句柄 */
osThreadId_t AdcTaskHandle;
osThreadId_t Mpu6050TaskHandle;
osThreadId_t DataSendTaskHandle;
osThreadId_t ModeSwitchTaskHandle;

/* 信号量句柄 */
osSemaphoreId_t ModeSwitchSemHandle;

#endif  // DEVICE_MASTER

/* ==================== 从机端变量 ==================== */
#ifdef DEVICE_SLAVE

/* 接收数据缓冲区 */
static uint8_t g_rx_buffer[BT_PACKET_SIZE];
static uint8_t g_rx_index = 0;

/* 控制数据 */
static ControlData_t g_control_data = {0};

/* 任务句柄 */
osThreadId_t DataReceiveTaskHandle;
osThreadId_t ServoControlTaskHandle;

/* 消息队列句柄 */
osMessageQueueId_t ControlDataQueueHandle;

#endif  // DEVICE_SLAVE

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* ==================== 主控端任务函数 ==================== */
#ifdef DEVICE_MASTER
void Task_ADC_Read(void *argument);           // ADC采集任务
void Task_MPU6050_Read(void *argument);       // MPU6050读取任务
void Task_DataSend(void *argument);           // 数据发送任务
void Task_ModeSwitch(void *argument);         // 模式切换任务
#endif

/* ==================== 从机端任务函数 ==================== */
#ifdef DEVICE_SLAVE
void Task_DataReceive(void *argument);        // 数据接收任务
void Task_ServoControl(void *argument);       // 舵机控制任务
#endif

/* ==================== 通用工具函数 ==================== */
uint8_t CalculateChecksum(uint8_t *data, uint8_t len);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  #ifdef DEVICE_MASTER
  /* 创建模式切换信号量 */
  ModeSwitchSemHandle = osSemaphoreNew(1, 0, NULL);
  #endif
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  #ifdef DEVICE_SLAVE
  /* 创建控制数据消息队列 */
  ControlDataQueueHandle = osMessageQueueNew(5, sizeof(ControlData_t), NULL);
  #endif
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* ==================== 主控端任务创建 ==================== */
  #ifdef DEVICE_MASTER

  /* ADC采集任务 */
  const osThreadAttr_t adc_task_attributes = {
    .name = "AdcTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  AdcTaskHandle = osThreadNew(Task_ADC_Read, NULL, &adc_task_attributes);

  /* MPU6050读取任务 */
  const osThreadAttr_t mpu_task_attributes = {
    .name = "Mpu6050Task",
    .stack_size = 512 * 4,  // 需要更大的栈，因为有浮点运算
    .priority = (osPriority_t) osPriorityNormal,
  };
  Mpu6050TaskHandle = osThreadNew(Task_MPU6050_Read, NULL, &mpu_task_attributes);

  /* 数据发送任务 */
  const osThreadAttr_t send_task_attributes = {
    .name = "DataSendTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };
  DataSendTaskHandle = osThreadNew(Task_DataSend, NULL, &send_task_attributes);

  /* 模式切换任务 */
  const osThreadAttr_t mode_task_attributes = {
    .name = "ModeSwitchTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  ModeSwitchTaskHandle = osThreadNew(Task_ModeSwitch, NULL, &mode_task_attributes);

  #endif  // DEVICE_MASTER

  /* ==================== 从机端任务创建 ==================== */
  #ifdef DEVICE_SLAVE

  /* 数据接收任务 */
  const osThreadAttr_t recv_task_attributes = {
    .name = "DataReceiveTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };
  DataReceiveTaskHandle = osThreadNew(Task_DataReceive, NULL, &recv_task_attributes);

  /* 舵机控制任务 */
  const osThreadAttr_t servo_task_attributes = {
    .name = "ServoControlTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityRealtime,
  };
  ServoControlTaskHandle = osThreadNew(Task_ServoControl, NULL, &servo_task_attributes);

  #endif  // DEVICE_SLAVE

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

