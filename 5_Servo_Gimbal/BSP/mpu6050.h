/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050陀螺仪加速度计驱动
  * @author  Cloud Platform Team
  * @date    2026-08-25
  ******************************************************************************
  * @attention
  *
  * MPU6050 6轴姿态传感器驱动
  * - I2C通信（100kHz）
  * - 加速度计：±2g
  * - 陀螺仪：±250°/s
  * - 互补滤波姿态解算
  *
  * 控制映射：
  *   PITCH角（前后俯仰） → 云台Pitch舵机
  *   ROLL角（左右倾斜）  → 云台Yaw舵机
  *
  ******************************************************************************
  */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief MPU6050原始数据结构
 */
typedef struct {
    int16_t accel_x;    // 加速度X轴原始值
    int16_t accel_y;    // 加速度Y轴原始值
    int16_t accel_z;    // 加速度Z轴原始值
    int16_t temp;       // 温度原始值
    int16_t gyro_x;     // 陀螺仪X轴原始值
    int16_t gyro_y;     // 陀螺仪Y轴原始值
    int16_t gyro_z;     // 陀螺仪Z轴原始值
} MPU6050_RawData_t;

/**
 * @brief MPU6050姿态角结构
 */
typedef struct {
    float pitch;        // 俯仰角 (°) - 前后翻转（点头/仰头）
    float roll;         // 横滚角 (°) - 左右翻转（侧倾）
} MPU6050_Attitude_t;

/* Exported constants --------------------------------------------------------*/

/* MPU6050 I2C地址（AD0=GND时为0x68，AD0=VCC时为0x69） */
#define MPU6050_ADDR            0x68
#define MPU6050_I2C_ADDR        (MPU6050_ADDR << 1)  // HAL库使用的地址：0xD0

/* MPU6050寄存器地址 */
#define MPU6050_REG_SMPLRT_DIV      0x19    // 采样率分频
#define MPU6050_REG_CONFIG          0x1A    // 配置
#define MPU6050_REG_GYRO_CONFIG     0x1B    // 陀螺仪配置
#define MPU6050_REG_ACCEL_CONFIG    0x1C    // 加速度计配置
#define MPU6050_REG_ACCEL_XOUT_H    0x3B    // 加速度X高字节
#define MPU6050_REG_TEMP_OUT_H      0x41    // 温度高字节
#define MPU6050_REG_GYRO_XOUT_H     0x43    // 陀螺仪X高字节
#define MPU6050_REG_PWR_MGMT_1      0x6B    // 电源管理1
#define MPU6050_REG_WHO_AM_I        0x75    // 器件ID

/* 配置参数 */
#define MPU6050_GYRO_FS_250         0x00    // 陀螺仪量程 ±250°/s
#define MPU6050_ACCEL_FS_2G         0x00    // 加速度计量程 ±2g

/* 灵敏度（根据量程计算） */
#define MPU6050_GYRO_SENSITIVITY    131.0f  // ±250°/s → 131 LSB/(°/s)
#define MPU6050_ACCEL_SENSITIVITY   16384.0f // ±2g → 16384 LSB/g

/* 互补滤波系数（0-1之间，越大越信任陀螺仪） */
#define MPU6050_FILTER_ALPHA        0.98f

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化MPU6050
 * @retval 0: 成功, -1: 失败
 */
int8_t MPU6050_Init(void);

/**
 * @brief  读取MPU6050原始数据
 * @param  data: 原始数据结构指针
 * @retval 0: 成功, -1: 失败
 */
int8_t MPU6050_ReadRawData(MPU6050_RawData_t *data);

/**
 * @brief  计算姿态角（互补滤波）
 * @param  raw_data: 原始数据
 * @param  attitude: 姿态角结构指针
 * @param  dt: 时间间隔（秒）
 * @retval None
 */
void MPU6050_CalculateAttitude(MPU6050_RawData_t *raw_data, MPU6050_Attitude_t *attitude, float dt);

/**
 * @brief  测试MPU6050通信
 * @retval 0: 成功, -1: 失败
 */
int8_t MPU6050_TestConnection(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */
