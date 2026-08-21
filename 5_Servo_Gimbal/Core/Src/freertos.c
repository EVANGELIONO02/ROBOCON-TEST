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
#include "servo.h"
#include "bluetooth.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 控制模式枚举（使用蓝牙驱动的定义） */
typedef BT_ControlMode_t ControlMode_t;
#define MODE_POTENTIOMETER BT_MODE_POTENTIOMETER
#define MODE_GYROSCOPE BT_MODE_GYROSCOPE

/* 控制数据结构（使用蓝牙驱动的定义） */
typedef BT_ControlPacket_t ControlData_t;

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

  #ifdef DEVICE_SLAVE
  /* 从机端：初始化舵机和蓝牙 */
  if(Servo_Init() == 0) {
      // 初始化成功，复位到中心位置
      Servo_ResetToCenter();
  }
  BT_Init();
  #endif

  #ifdef DEVICE_MASTER
  /* 主控端：初始化蓝牙 */
  BT_Init();
  #endif

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ==================== 从机端任务实现 ==================== */
#ifdef DEVICE_SLAVE

/**
 * @brief 数据接收任务 - 接收并解析蓝牙数据
 * @param argument: 未使用
 */
void Task_DataReceive(void *argument)
{
    uint8_t rx_byte;
    ControlData_t ctrl_data;

    for(;;)
    {
        // 非阻塞接收单字节
        if(BT_ReceiveByte(&rx_byte) == 0)
        {
            // 状态机解析数据包
            BT_RxState_t state = BT_ParseByte(rx_byte, &ctrl_data);

            if(state == BT_RX_STATE_COMPLETE)
            {
                // 数据包接收完成，发送到舵机控制任务
                osMessageQueuePut(ControlDataQueueHandle, &ctrl_data, 0, 0);

                // 重置状态机，准备接收下一包
                BT_ResetRxState();
            }
            else if(state == BT_RX_STATE_ERROR)
            {
                // 接收出错，重置状态机
                BT_ResetRxState();
            }
        }

        osDelay(1);
    }
}

/**
 * @brief 舵机控制任务 - 控制舵机运动
 * @param argument: 未使用
 */
void Task_ServoControl(void *argument)
{
    ControlData_t ctrl_data;

    for(;;)
    {
        // 等待控制数据
        if(osMessageQueueGet(ControlDataQueueHandle, &ctrl_data, NULL, osWaitForever) == osOK)
        {
            // 限幅检查
            if(ctrl_data.yaw > SERVO_ANGLE_MAX) {
                ctrl_data.yaw = SERVO_ANGLE_MAX;
            }
            if(ctrl_data.pitch > SERVO_ANGLE_MAX) {
                ctrl_data.pitch = SERVO_ANGLE_MAX;
            }

            // 设置舵机角度
            Servo_SetAngle(SERVO_YAW, ctrl_data.yaw);
            Servo_SetAngle(SERVO_PITCH, ctrl_data.pitch);
        }
    }
}

#endif  // DEVICE_SLAVE

/* ==================== 主控端任务实现 ==================== */
#ifdef DEVICE_MASTER

/**
 * @brief ADC采集任务 - 读取两个电位器的值
 * @param argument: 未使用
 */
void Task_ADC_Read(void *argument)
{
    extern ADC_HandleTypeDef hadc1;
    uint16_t adc_values[2];  // 存储ADC1的双通道数据

    // 启动ADC的DMA循环采集
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 2);

    for(;;)
    {
        // ADC已配置为DMA循环模式，数据自动更新到adc_values
        // 将ADC值(0-4095)映射到角度(0-180)
        g_adc_yaw_raw = adc_values[0];
        g_adc_pitch_raw = adc_values[1];

        g_adc_yaw_angle = (uint8_t)((g_adc_yaw_raw * 180) / 4095);
        g_adc_pitch_angle = (uint8_t)((g_adc_pitch_raw * 180) / 4095);

        osDelay(20);  // 50Hz采样率
    }
}

/**
 * @brief MPU6050读取任务 - 读取陀螺仪姿态
 * @param argument: 未使用
 */
void Task_MPU6050_Read(void *argument)
{
    // TODO: 初始化MPU6050
    // MPU6050_Init();

    for(;;)
    {
        // TODO: 读取MPU6050数据并进行姿态解算
        // MPU6050_Read_All(&ax, &ay, &az, &gx, &gy, &gz);
        // 进行互补滤波或卡尔曼滤波
        // g_mpu_yaw = ...;
        // g_mpu_pitch = ...;

        // 映射到0-180度
        // g_mpu_yaw_angle = (uint8_t)((g_mpu_yaw + 90.0f));
        // g_mpu_pitch_angle = (uint8_t)((g_mpu_pitch + 90.0f));

        // 临时测试值
        g_mpu_yaw_angle = 90;
        g_mpu_pitch_angle = 90;

        osDelay(10);  // 100Hz采样率
    }
}

/**
 * @brief 数据发送任务 - 通过蓝牙发送控制数据
 * @param argument: 未使用
 */
void Task_DataSend(void *argument)
{
    uint8_t yaw, pitch;
    BT_ControlMode_t mode;

    for(;;)
    {
        // 根据当前模式选择数据源
        if(g_control_mode == MODE_POTENTIOMETER) {
            mode = BT_MODE_POTENTIOMETER;
            yaw = g_adc_yaw_angle;
            pitch = g_adc_pitch_angle;
        } else {
            mode = BT_MODE_GYROSCOPE;
            yaw = g_mpu_yaw_angle;
            pitch = g_mpu_pitch_angle;
        }

        // 发送数据包
        BT_SendPacket(mode, yaw, pitch);

        osDelay(50);  // 20Hz发送频率
    }
}

/**
 * @brief 模式切换任务 - 处理电位器/陀螺仪模式切换
 * @param argument: 未使用
 */
void Task_ModeSwitch(void *argument)
{
    for(;;)
    {
        // 等待按钮信号量（在GPIO中断中释放）
        if(osSemaphoreAcquire(ModeSwitchSemHandle, osWaitForever) == osOK)
        {
            // 切换模式
            if(g_control_mode == MODE_POTENTIOMETER) {
                g_control_mode = MODE_GYROSCOPE;
            } else {
                g_control_mode = MODE_POTENTIOMETER;
            }

            // TODO: LED指示模式
            // 例如：LED闪烁表示当前模式
        }
    }
}

#endif  // DEVICE_MASTER

/* USER CODE END Application */

