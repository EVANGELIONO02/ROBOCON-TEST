/**
  ******************************************************************************
  * @file    bluetooth.h
  * @brief   蓝牙通信驱动头文件
  * @author  Cloud Platform Team
  * @date    2026-08-21
  ******************************************************************************
  * @attention
  *
  * 支持HC-05/HC-06蓝牙模块
  * USART1配置：9600bps, 8N1
  * 数据包格式：[0xAA] [模式] [Yaw] [Pitch] [校验和] [0x55]
  *
  * 硬件连接：
  *   - USART1_TX (PA9)  → 蓝牙RX
  *   - USART1_RX (PA10) → 蓝牙TX
  *
  ******************************************************************************
  */

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 控制模式枚举
 */
typedef enum {
    BT_MODE_POTENTIOMETER = 0x01,  // 电位器控制模式
    BT_MODE_GYROSCOPE = 0x02       // 陀螺仪控制模式
} BT_ControlMode_t;

/**
 * @brief 蓝牙控制数据包结构
 */
typedef struct {
    uint8_t mode;       // 控制模式
    uint8_t yaw;        // 水平角度 (0-180)
    uint8_t pitch;      // 俯仰角度 (0-180)
    uint8_t checksum;   // 校验和
} BT_ControlPacket_t;

/**
 * @brief 蓝牙接收状态枚举
 */
typedef enum {
    BT_RX_STATE_IDLE = 0,       // 空闲状态
    BT_RX_STATE_RECEIVING,      // 接收中
    BT_RX_STATE_COMPLETE,       // 接收完成
    BT_RX_STATE_ERROR           // 接收错误
} BT_RxState_t;

/* Exported constants --------------------------------------------------------*/

/* 蓝牙数据包协议 */
#define BT_FRAME_HEADER         0xAA        // 帧头
#define BT_FRAME_TAIL           0x55        // 帧尾
#define BT_PACKET_SIZE          6           // 数据包总长度

/* 数据包字段索引 */
#define BT_INDEX_HEADER         0
#define BT_INDEX_MODE           1
#define BT_INDEX_YAW            2
#define BT_INDEX_PITCH          3
#define BT_INDEX_CHECKSUM       4
#define BT_INDEX_TAIL           5

/* 接收超时时间（ms） */
#define BT_RX_TIMEOUT           100

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化蓝牙通信
 * @param  None
 * @retval 0: 成功, -1: 失败
 */
int8_t BT_Init(void);

/**
 * @brief  发送控制数据包（主控端使用）
 * @param  mode: 控制模式
 * @param  yaw: 水平角度 (0-180)
 * @param  pitch: 俯仰角度 (0-180)
 * @retval 0: 成功, -1: 失败
 */
int8_t BT_SendPacket(BT_ControlMode_t mode, uint8_t yaw, uint8_t pitch);

/**
 * @brief  接收并解析数据包（从机端使用，阻塞式）
 * @param  packet: 接收到的数据包指针
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval 0: 成功, -1: 失败, -2: 超时
 */
int8_t BT_ReceivePacket(BT_ControlPacket_t *packet, uint32_t timeout_ms);

/**
 * @brief  接收单字节数据（从机端使用，非阻塞式）
 * @param  data: 接收数据指针
 * @retval 0: 成功, -1: 无数据
 */
int8_t BT_ReceiveByte(uint8_t *data);

/**
 * @brief  数据包状态机解析（从机端使用）
 * @param  byte: 接收到的单字节数据
 * @param  packet: 解析成功后存储的数据包指针
 * @retval BT_RxState_t: 当前接收状态
 */
BT_RxState_t BT_ParseByte(uint8_t byte, BT_ControlPacket_t *packet);

/**
 * @brief  重置接收状态机
 * @param  None
 * @retval None
 */
void BT_ResetRxState(void);

/**
 * @brief  计算校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验和（8位）
 */
uint8_t BT_CalculateChecksum(uint8_t *data, uint8_t len);

/**
 * @brief  验证数据包校验和
 * @param  packet: 数据包指针
 * @retval 0: 校验成功, -1: 校验失败
 */
int8_t BT_VerifyChecksum(BT_ControlPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* __BLUETOOTH_H__ */
