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
#include "servo.h"
#include "bluetooth.h"
#include "mpu6050.h"
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
/* 控制数据 */
static ControlData_t g_control_data = {0};

/* 任务句柄 */
osThreadId_t DataReceiveTaskHandle;
osThreadId_t ServoYawTaskHandle;
osThreadId_t ServoPitchTaskHandle;

/* 消息队列句柄 */
osMessageQueueId_t YawAngleQueueHandle;    // 水平角度队列
osMessageQueueId_t PitchAngleQueueHandle;  // 俯仰角度队列

#endif  // DEVICE_SLAVE

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
void Task_ServoYaw(void *argument);           // 水平舵机控制任务
void Task_ServoPitch(void *argument);         // 俯仰舵机控制任务
#endif

/* ==================== 通用工具函数 ==================== */
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
  /* 创建角度控制消息队列 */
  YawAngleQueueHandle = osMessageQueueNew(10, sizeof(uint8_t), NULL);
  PitchAngleQueueHandle = osMessageQueueNew(10, sizeof(uint8_t), NULL);

  // 检查队列创建是否成功
  if(YawAngleQueueHandle == NULL || PitchAngleQueueHandle == NULL) {
    // 队列创建失败，进入死循环
    while(1);
  }
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
    .priority = (osPriority_t) osPriorityRealtime,
  };
  DataReceiveTaskHandle = osThreadNew(Task_DataReceive, NULL, &recv_task_attributes);

  /* 水平舵机控制任务 */
  const osThreadAttr_t yaw_task_attributes = {
    .name = "ServoYawTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };
  ServoYawTaskHandle = osThreadNew(Task_ServoYaw, NULL, &yaw_task_attributes);

  /* 俯仰舵机控制任务 */
  const osThreadAttr_t pitch_task_attributes = {
    .name = "ServoPitchTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };
  ServoPitchTaskHandle = osThreadNew(Task_ServoPitch, NULL, &pitch_task_attributes);

  // 检查任务创建是否成功
  if(DataReceiveTaskHandle == NULL || ServoYawTaskHandle == NULL || ServoPitchTaskHandle == NULL) {
    // 任务创建失败，进入死循环
    while(1);
  }

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
  Servo_Init();
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

    osDelay(100);  // 等待初始化完成

    for(;;)
    {
        // 非阻塞接收单字节
        if(BT_ReceiveByte(&rx_byte) == 0)
        {
            // 状态机解析数据包
            BT_RxState_t state = BT_ParseByte(rx_byte, &ctrl_data);

            if(state == BT_RX_STATE_COMPLETE)
            {
                // 数据包接收完成，直接控制舵机
                Servo_SetAngle(SERVO_YAW, ctrl_data.yaw);
                Servo_SetAngle(SERVO_PITCH, ctrl_data.pitch);

                // 发送到队列供其他任务使用
                osMessageQueuePut(YawAngleQueueHandle, &ctrl_data.yaw, 0, 0);
                osMessageQueuePut(PitchAngleQueueHandle, &ctrl_data.pitch, 0, 0);

                // 重置状态机
                BT_ResetRxState();
            }
            else if(state == BT_RX_STATE_ERROR)
            {
                // 接收出错，重置
                BT_ResetRxState();
            }
        }

        // 高速接收，减少延时避免DMA缓冲区溢出
        osDelay(0);  // 让出CPU但不强制延时
    }
}

/**
 * @brief 水平舵机控制任务 - 独立控制Yaw舵机
 * @param argument: 未使用
 */
void Task_ServoYaw(void *argument)
{
    uint8_t yaw_angle;
    osStatus_t status;

    for(;;)
    {
        // 等待水平角度数据
        status = osMessageQueueGet(YawAngleQueueHandle, &yaw_angle, NULL, 100);

        if(status == osOK)
        {
            Servo_SetAngle(SERVO_YAW, yaw_angle);
        }

        osDelay(10);
    }
}

/**
 * @brief 俯仰舵机控制任务 - 独立控制Pitch舵机
 * @param argument: 未使用
 */
void Task_ServoPitch(void *argument)
{
    uint8_t pitch_angle;
    osStatus_t status;

    for(;;)
    {
        // 等待俯仰角度数据
        status = osMessageQueueGet(PitchAngleQueueHandle, &pitch_angle, NULL, 100);

        if(status == osOK)
        {
            Servo_SetAngle(SERVO_PITCH, pitch_angle);
        }

        osDelay(10);
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
    MPU6050_RawData_t raw_data;
    MPU6050_Attitude_t attitude;

    // 初始化MPU6050
    if(MPU6050_Init() != 0) {
        // 初始化失败，使用默认值
        g_mpu_yaw_angle = 90;
        g_mpu_pitch_angle = 90;

        for(;;) {
            osDelay(100);
        }
    }

    for(;;)
    {
        // 读取原始数据
        if(MPU6050_ReadRawData(&raw_data) == 0) {
            // 计算姿态角（dt=10ms=0.01s）
            MPU6050_CalculateAttitude(&raw_data, &attitude, 0.01f);

            // 根据实际安装方向调整映射：
            // ROLL角控制Pitch舵机（前后俯仰）：-90° ~ +90° 映射到 30° ~ 140°
            // PITCH角控制Yaw舵机（左右倾斜）：-90° ~ +90° 映射到 0° ~ 180°

            // 限幅并映射ROLL角到Pitch舵机
            float roll_limited = attitude.roll;
            if(roll_limited < -90.0f) roll_limited = -90.0f;
            if(roll_limited > 90.0f) roll_limited = 90.0f;
            g_mpu_pitch_angle = (uint8_t)((roll_limited + 90.0f) * 110.0f / 180.0f + 30.0f);

            // 限幅并映射PITCH角到Yaw舵机
            float pitch_limited = attitude.pitch;
            if(pitch_limited < -90.0f) pitch_limited = -90.0f;
            if(pitch_limited > 90.0f) pitch_limited = 90.0f;
            g_mpu_yaw_angle = (uint8_t)((pitch_limited + 90.0f));
        } else {
            // 读取失败，保持上次的值
        }

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
        }
    }
}

#endif  // DEVICE_MASTER

/* USER CODE END Application */

