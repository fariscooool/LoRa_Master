#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 初始化
 *============================================================================*/

/**
 * @brief 初始化 LoRa 串口:启动 LPUART1 DMA 接收(收满或空闲触发回调)
 * @note  需要在 MX_LPUART1_UART_Init() / MX_DMA_Init() 之后调用
 */
void lora_uart_init(void);

/*==============================================================================
 * 发送(DMA,阻塞语义)
 *============================================================================*/

/**
 * @brief 把一帧数据原样发送给 LoRa 模块(DMA;返回前已发送完成)
 * @param data  待发送数据
 * @param len   字节数
 */
void lora_send(const uint8_t *data, uint16_t len);

/*==============================================================================
 * 接收(DMA+空闲检测,数据被搬入内部 FIFO,接口保持简单)
 *============================================================================*/

/**
 * @brief 当前接收 FIFO 中可读字节数
 */
uint32_t lora_rx_available(void);

/**
 * @brief 从接收 FIFO 取一个字节
 * @retval 1=成功  0=空
 */
uint8_t lora_rx_get(uint8_t *byte);

/**
 * @brief 从接收 FIFO 查看一个字节(不移动读指针)
 * @retval 1=成功  0=空
 */
uint8_t lora_rx_peek(uint8_t *byte);

/**
 * @brief 从接收 FIFO 取多字节
 * @retval 1=成功  0=失败(空或可读字节不足)
 */
uint8_t lora_rx_get_bytes(uint8_t *buf, uint16_t len);

/**
 * @brief DMA 收到一段完整数据后回调(由 bsp_dbg_uart 的 HAL 回调分发过来)
 * @note  仅中断上下文调用;内部会搬数据并重新启动接收
 */
void lora_uart_rx_event(uint16_t size);

/**
 * @brief 接收出错后重新启动接收(由 HAL_UART_ErrorCallback 分发过来)
 * @note  仅中断上下文调用
 */
void lora_uart_rx_restart(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LORA_UART_H */

