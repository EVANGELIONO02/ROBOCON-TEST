/**
  ******************************************************************************
  * @file    bluetooth.c
  * @brief   蓝牙通信驱动实现
  * @author  Cloud Platform Team
  * @date    2026-08-21
  ******************************************************************************
  * @attention
  *
  * 提供蓝牙数据包的发送、接收、解析功能
  * 支持阻塞式和非阻塞式接收
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bluetooth.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// 接收状态机变量
static uint8_t rx_buffer[BT_PACKET_SIZE];
static uint8_t rx_index = 0;
static BT_RxState_t rx_state = BT_RX_STATE_IDLE;

// DMA循环接收缓冲区
#define DMA_RX_BUFFER_SIZE  64
static uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];
static uint16_t dma_rx_read_pos = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化蓝牙通信
 * @param  None
 * @retval 0: 成功, -1: 失败
 */
int8_t BT_Init(void)
{
    // 清空接收缓冲区
    memset(rx_buffer, 0, BT_PACKET_SIZE);
    memset(dma_rx_buffer, 0, DMA_RX_BUFFER_SIZE);
    rx_index = 0;
    rx_state = BT_RX_STATE_IDLE;
    dma_rx_read_pos = 0;

    // 启动UART DMA循环接收
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, DMA_RX_BUFFER_SIZE);

    if(status != HAL_OK) {
        return -1;
    }

    return 0;
}

/**
 * @brief  发送控制数据包（主控端使用）
 * @param  mode: 控制模式
 * @param  yaw: 水平角度 (0-180)
 * @param  pitch: 俯仰角度 (0-180)
 * @retval 0: 成功, -1: 失败
 */
int8_t BT_SendPacket(BT_ControlMode_t mode, uint8_t yaw, uint8_t pitch)
{
    static uint8_t tx_buffer[BT_PACKET_SIZE];  // 改为static，避免DMA传输时栈被释放
    uint8_t data_for_checksum[3];

    // 角度限幅
    if (yaw > 180) yaw = 180;
    if (pitch > 180) pitch = 180;

    // 构建数据包
    tx_buffer[BT_INDEX_HEADER] = BT_FRAME_HEADER;
    tx_buffer[BT_INDEX_MODE] = mode;
    tx_buffer[BT_INDEX_YAW] = yaw;
    tx_buffer[BT_INDEX_PITCH] = pitch;

    // 计算校验和（模式+Yaw+Pitch）
    data_for_checksum[0] = mode;
    data_for_checksum[1] = yaw;
    data_for_checksum[2] = pitch;
    tx_buffer[BT_INDEX_CHECKSUM] = BT_CalculateChecksum(data_for_checksum, 3);

    tx_buffer[BT_INDEX_TAIL] = BT_FRAME_TAIL;

    // 使用DMA发送数据包（不等待完成，避免阻塞）
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, tx_buffer, BT_PACKET_SIZE);

    return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  接收并解析数据包（从机端使用，阻塞式）
 * @param  packet: 接收到的数据包指针
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval 0: 成功, -1: 失败, -2: 超时
 */
int8_t BT_ReceivePacket(BT_ControlPacket_t *packet, uint32_t timeout_ms)
{
    uint8_t rx_byte;
    uint32_t start_tick = HAL_GetTick();

    // 重置状态机
    BT_ResetRxState();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        // 尝试接收一个字节
        if (HAL_UART_Receive(&huart1, &rx_byte, 1, 10) == HAL_OK)
        {
            // 状态机解析
            BT_RxState_t state = BT_ParseByte(rx_byte, packet);

            if (state == BT_RX_STATE_COMPLETE) {
                return 0;  // 接收成功
            }
            else if (state == BT_RX_STATE_ERROR) {
                BT_ResetRxState();  // 出错重置
            }
        }
    }

    return -2;  // 超时
}

/**
 * @brief  接收单字节数据（从机端使用，非阻塞式）
 * @param  data: 接收数据指针
 * @retval 0: 成功, -1: 无数据
 */
int8_t BT_ReceiveByte(uint8_t *data)
{
    // 获取DMA当前写入位置
    uint16_t dma_write_pos = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    // 检查是否有新数据
    if(dma_rx_read_pos != dma_write_pos)
    {
        // 读取一个字节
        *data = dma_rx_buffer[dma_rx_read_pos];

        // 更新读指针（循环缓冲区）
        dma_rx_read_pos = (dma_rx_read_pos + 1) % DMA_RX_BUFFER_SIZE;

        return 0;
    }

    return -1;
}

/**
 * @brief  数据包状态机解析（从机端使用）
 * @param  byte: 接收到的单字节数据
 * @param  packet: 解析成功后存储的数据包指针
 * @retval BT_RxState_t: 当前接收状态
 */
BT_RxState_t BT_ParseByte(uint8_t byte, BT_ControlPacket_t *packet)
{
    switch (rx_state)
    {
        case BT_RX_STATE_IDLE:
            // 等待帧头
            if (byte == BT_FRAME_HEADER) {
                rx_buffer[0] = byte;
                rx_index = 1;
                rx_state = BT_RX_STATE_RECEIVING;
            }
            break;

        case BT_RX_STATE_RECEIVING:
            // 接收数据
            rx_buffer[rx_index++] = byte;

            // 接收完整
            if (rx_index >= BT_PACKET_SIZE) {
                // 验证帧尾
                if (rx_buffer[BT_INDEX_TAIL] != BT_FRAME_TAIL) {
                    rx_state = BT_RX_STATE_ERROR;
                    return rx_state;
                }

                // 提取数据
                packet->mode = rx_buffer[BT_INDEX_MODE];
                packet->yaw = rx_buffer[BT_INDEX_YAW];
                packet->pitch = rx_buffer[BT_INDEX_PITCH];
                packet->checksum = rx_buffer[BT_INDEX_CHECKSUM];

                // 验证校验和
                if (BT_VerifyChecksum(packet) != 0) {
                    rx_state = BT_RX_STATE_ERROR;
                    return rx_state;
                }

                // 接收完成
                rx_state = BT_RX_STATE_COMPLETE;
                return rx_state;
            }
            break;

        case BT_RX_STATE_COMPLETE:
        case BT_RX_STATE_ERROR:
            // 这些状态下不应该接收数据，重置状态机
            BT_ResetRxState();
            break;
    }

    return rx_state;
}

/**
 * @brief  重置接收状态机
 * @param  None
 * @retval None
 */
void BT_ResetRxState(void)
{
    rx_index = 0;
    rx_state = BT_RX_STATE_IDLE;
    memset(rx_buffer, 0, BT_PACKET_SIZE);
}

/**
 * @brief  计算校验和
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验和（8位）
 */
uint8_t BT_CalculateChecksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

/**
 * @brief  验证数据包校验和
 * @param  packet: 数据包指针
 * @retval 0: 校验成功, -1: 校验失败
 */
int8_t BT_VerifyChecksum(BT_ControlPacket_t *packet)
{
    uint8_t data[3] = {packet->mode, packet->yaw, packet->pitch};
    uint8_t calc_checksum = BT_CalculateChecksum(data, 3);

    return (calc_checksum == packet->checksum) ? 0 : -1;
}
