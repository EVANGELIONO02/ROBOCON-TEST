/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050陀螺仪加速度计驱动实现
  * @author  Cloud Platform Team
  * @date    2026-08-25
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "cmsis_os.h"  // 添加FreeRTOS支持
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static MPU6050_Attitude_t g_attitude = {0};  // 姿态角

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  写入单个寄存器
 */
static int8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  读取单个寄存器
 */
static int8_t MPU6050_ReadReg(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10);
    return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  读取多个寄存器
 */
static int8_t MPU6050_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 20);
    return (status == HAL_OK) ? 0 : -1;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  测试MPU6050通信
 */
int8_t MPU6050_TestConnection(void)
{
    uint8_t who_am_i;

    if(MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &who_am_i) != 0) {
        return -1;
    }

    // MPU6050的WHO_AM_I寄存器应该返回0x68
    return (who_am_i == 0x68) ? 0 : -1;
}

/**
 * @brief  初始化MPU6050
 */
int8_t MPU6050_Init(void)
{
    // 测试通信（快速失败）
    if(MPU6050_TestConnection() != 0) {
        return -1;  // 如果设备不存在，立即返回
    }

    // 1. 复位设备
    if(MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80) != 0) {
        return -1;
    }
    osDelay(50);  // 减少复位等待时间到50ms，使用osDelay避免阻塞其他任务

    // 2. 唤醒设备（关闭睡眠模式，使用内部8MHz时钟）
    if(MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00) != 0) {
        return -1;
    }
    osDelay(5);  // 减少到5ms

    // 3. 配置采样率分频（1kHz / (1 + 9) = 100Hz）
    if(MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 9) != 0) {
        return -1;
    }

    // 4. 配置数字低通滤波器（带宽94Hz，延迟2.9ms）
    if(MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x02) != 0) {
        return -1;
    }

    // 5. 配置陀螺仪量程（±250°/s）
    if(MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250 << 3) != 0) {
        return -1;
    }

    // 6. 配置加速度计量程（±2g）
    if(MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G << 3) != 0) {
        return -1;
    }

    // 初始化姿态角为0
    g_attitude.pitch = 0.0f;
    g_attitude.roll = 0.0f;

    return 0;
}

/**
 * @brief  读取MPU6050原始数据
 */
int8_t MPU6050_ReadRawData(MPU6050_RawData_t *data)
{
    uint8_t buffer[14];

    // 从ACCEL_XOUT_H开始连续读取14字节
    // ACCEL_X(2) + ACCEL_Y(2) + ACCEL_Z(2) + TEMP(2) + GYRO_X(2) + GYRO_Y(2) + GYRO_Z(2)
    if(MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buffer, 14) != 0) {
        return -1;
    }

    // 组合高低字节（大端序）
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->temp    = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);

    return 0;
}

/**
 * @brief  计算姿态角（互补滤波）
 * @note   PITCH: 前后俯仰，ROLL: 左右倾斜
 */
void MPU6050_CalculateAttitude(MPU6050_RawData_t *raw_data, MPU6050_Attitude_t *attitude, float dt)
{
    // 1. 转换为物理单位
    float acc_x = raw_data->accel_x / MPU6050_ACCEL_SENSITIVITY;  // g
    float acc_y = raw_data->accel_y / MPU6050_ACCEL_SENSITIVITY;  // g
    float acc_z = raw_data->accel_z / MPU6050_ACCEL_SENSITIVITY;  // g

    float gyro_x = raw_data->gyro_x / MPU6050_GYRO_SENSITIVITY;   // °/s
    float gyro_y = raw_data->gyro_y / MPU6050_GYRO_SENSITIVITY;   // °/s

    // 2. 从加速度计计算角度（静态）
    float accel_pitch = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 180.0f / M_PI;
    float accel_roll  = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0f / M_PI;

    // 3. 从陀螺仪计算角度增量（动态）
    float gyro_pitch = g_attitude.pitch + gyro_x * dt;
    float gyro_roll  = g_attitude.roll  + gyro_y * dt;

    // 4. 互补滤波融合
    g_attitude.pitch = MPU6050_FILTER_ALPHA * gyro_pitch + (1.0f - MPU6050_FILTER_ALPHA) * accel_pitch;
    g_attitude.roll  = MPU6050_FILTER_ALPHA * gyro_roll  + (1.0f - MPU6050_FILTER_ALPHA) * accel_roll;

    // 5. 输出结果
    attitude->pitch = g_attitude.pitch;
    attitude->roll  = g_attitude.roll;
}
