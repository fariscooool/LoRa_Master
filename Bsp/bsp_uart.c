/**
 * @file bsp_lora_uart.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  LoRa 数据串口(LPUART1)驱动实现(DMA 收发)
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   - 发送用 DMA,但保持“阻塞”语义(等发完才返回)。
 *         - 接收用 DMA+空闲检测(HAL_UARTEx_ReceiveToIdle_DMA):
 *             DMA 为 Normal 模式,收满一缓冲或总线空闲即停止并回调;
 *             回调里把数据搬进 FIFO 后重新启动接收。
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

#include "main.h"
#include "usart.h"
#include "stm32f4xx_hal_uart.h"

/*==============================================================================
 * 局部定义
 *============================================================================*/

#define LORA_RX_DMA_SIZE   64U    /* DMA 接收缓冲(单次最多缓存字节) */
#define LORA_RX_FIFO_SIZE  128U   /* 内部 FIFO(供应用层读取) */

/*==============================================================================
 * 局部变量
 *============================================================================*/

static uint8_t   s_rx_dma[LORA_RX_DMA_SIZE];     /* DMA 接收缓冲 */
static uint8_t   s_rx_fifo[LORA_RX_FIFO_SIZE];   /* 内部 FIFO 存储区 */
static volatile uint16_t s_rx_head = 0;          /* FIFO 写指针(中断写) */
static volatile uint16_t s_rx_tail = 0;          /* FIFO 读指针         */

/*==============================================================================
 * 内部函数
 *============================================================================*/

static void lora_fifo_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % LORA_RX_FIFO_SIZE);
    if (next != s_rx_tail)           /* 未满才写入 */
    {
        s_rx_fifo[s_rx_head] = byte;
        s_rx_head = next;
    }
}

/* 启动一次 DMA“收到空闲/收满”接收 */
static void lora_rx_start(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_rx_dma, LORA_RX_DMA_SIZE);
}

/* 等待 DMA 发送真正结束(gState=READY 且 UART 移位寄存器发完 TC) */
static void lora_wait_tx_done(UART_HandleTypeDef *huart)
{
    uint32_t t0 = HAL_GetTick();
    while ((huart->gState != HAL_UART_STATE_READY) ||
           (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET))
    {
        if ((HAL_GetTick() - t0) > 200U)
        {
            break;
        }
    }
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

void lora_uart_init(void)
{
    /* LPUART1 与 DMA 中断已由 CubeMX(MX_DMA_Init / HAL_UART_MspInit)使能 */
    lora_rx_start();
}

void lora_send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    /* 先清掉上一次遗留的 TC */
    __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);

    /* 本工程 usart.c 只做了 __HAL_LINKDMA(..., hdmarx, ...),没有 TX DMA,
     * 所以 huart6.hdmatx == NULL。而 HAL_UART_Transmit_DMA() 进去第一件事就是
     * huart->hdmatx->XferCpltCallback = ...(stm32f4xx_hal_uart.c:1399,前面不判空)
     * → 空指针写入 → HardFault。这里必须判空后才能走 DMA 发送。 */
    if ((huart6.hdmatx != NULL) &&
        (HAL_UART_Transmit_DMA(&huart6, data, len) == HAL_OK))
    {
        lora_wait_tx_done(&huart6);  /* 阻塞等待发完(进低功耗前总线空闲) */
        return;
    }

    /* 没有 TX DMA 通道 / DMA 启动失败:退化为阻塞发送,保证这一帧能发出去 */
    (void)HAL_UART_Transmit(&huart6, data, len, 200U);
}

uint32_t lora_rx_available(void)
{
    return (uint32_t)((s_rx_head - s_rx_tail) % LORA_RX_FIFO_SIZE);
}

uint8_t lora_rx_get(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head)
    {
        return 0;                    /* 空 */
    }
    *byte = s_rx_fifo[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % LORA_RX_FIFO_SIZE);
    return 1;
}

uint8_t lora_rx_peek(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head)
    {
        return 0;                    /* 空 */
    }
    *byte = s_rx_fifo[s_rx_tail];
    return 1;
}

uint8_t lora_rx_get_bytes(uint8_t *buf, uint16_t len)
{
    if(buf == NULL){
        return 0;
    }
    
    if(lora_rx_available() < len)
    {
        return 0;
    }

    for(uint16_t i = 0; i < len; i++)
    {
        if(!lora_rx_get(&buf[i]))
        {
            return 0;
        }
    }
    return 1;
}

/*==============================================================================
 * 中断上下文处理(由 bsp_dbg_uart 的 HAL 回调分发调用)
 *============================================================================*/

/* DMA 收到一段完整数据(空闲/收满)后:搬进 FIFO 并重新启动接收 */
void lora_uart_rx_event(uint16_t size)
{
    uint16_t i;

    if (size > LORA_RX_DMA_SIZE)
    {
        size = LORA_RX_DMA_SIZE;
    }
    for (i = 0U; i < size; i++)

    {
        lora_fifo_push(s_rx_dma[i]);
    }
    lora_rx_start();
}

/* 接收出错后重新启动接收 */
void lora_uart_rx_restart(void)
{
    lora_rx_start();
}

/*==============================================================================
 * HAL 回调:把 UART 事件分发给本驱动
 *
 * ⚠ 之前只提供了 lora_uart_rx_event()/lora_uart_rx_restart(),但没有实现下面
 *   这两个回调,也没有 bsp_dbg_uart 之类的分发层(本工程里根本不存在),
 *   所以 HAL 走的是 __weak 空实现 → FIFO 永远是空的、一帧也收不到。
 *============================================================================*/

/* ReceiveToIdle_DMA 的事件回调:总线空闲(IDLE)或 DMA 收满时都会进来,
 * Size = 本轮收到的字节数(Normal 模式语义)。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        lora_uart_rx_event(Size);
    }
}

/* 出错(过载/帧错/噪声/校验)时 HAL 会中止本轮接收,必须重新启动 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        lora_uart_rx_restart();
    }
}

