/**
 * @file app_main.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "app_main.h"
#include "app_lora_procotol.h"

#include "app_lora.h"
#include "bsp_uart.h"
#include "LOG.h"


/*==============================================================================
 * 调试观测变量(在调试器的 Watch / Live Watch 窗口直接查看)
 *============================================================================*/
volatile uint8_t  g_cfg_ok;         /* LoRa 配置"写入+读回校验"结果:1=成功 0=失败 */
volatile uint16_t g_test_tx_cnt;    /* 测试:已发出的帧数 */
volatile uint16_t g_test_rx_cnt;    /* 测试:已收到的字节数 */
volatile uint8_t  g_test_rx_buf[32];/* 测试:收到的前 32 个字节 */
static lora_reg_parm_cfg_t lora_cfg_t;

/*==============================================================================
 * 对外接口(供 main.c 调用)
 *============================================================================*/

void app_init(void)
{

    lora_uart_init();        /* 启动 UART6 中断接收(必须在 MX_USART6_UART_Init 之后) */

    app_lora_init();        /* LoRa 应用层:模式初始化 */

    LOG_INIT();              /* 初始化日志模块 */

    // LORA 参数配置
    lora_reg_parm_cfg_t lora_cfg = {
        .detail.addr_high = 0xFF,
        .detail.addr_low = 0xFF,
        .detail.sped.bits.air_rate = AIR_2_4K,
        .detail.sped.bits.ttl_rate = BAUD_9600,
        .detail.sped.bits.parity = PARITY_8N1,
        .detail.channel = 0x00,   /* 根据实际情况初始化 */
        .detail.option.bits.power = TX_POWER_20DBM,
        .detail.option.bits.fec = FEC_DISABLE,
        .detail.option.bits.wakeup_time = WAKEUP_250MS,
        .detail.option.bits.io_drv_mode = IO_DRV_MODE_PUSH_PULL,
        .detail.option.bits.fixed_point_trans = FIXED_POINT_TRANS_DISABLE,
    };

    lora_cfg_t = lora_cfg;

    g_cfg_ok = app_lora_cfg_reg_verify(lora_cfg_t, sizeof(lora_cfg_t.data));

    log_i("LoRa configuration %s", g_cfg_ok ? "successful" : "failed");
}

void app_deinit(void)
{
    /* 暂无需要释放的资源 */
}

/*==============================================================================
 * 临时:收发验证(验证完成后把 LORA_TX_RX_TEST 改成 0 即可恢复协议处理)
 *
 * 用法A - 单板自环,验证 MCU 侧 UART 收发通路(不需要第二个模块):
 *    把 PC6(USART6_TX) 与 PC7(USART6_RX) 直接短接(或拔掉模块后短接),重新上电
 *    预期:g_test_tx_cnt 与 g_test_rx_cnt 同步递增,最终两者近似相等
 *
 * 用法B - 双板 RF,验证模块收发:
 *    两块板都烧同一固件(地址/信道完全一样,透传模式可以互通)
 *    预期:g_cfg_ok == 1,且能收到对端周期性发来的 5A 04 00 F7,g_test_rx_cnt 持续递增
 *============================================================================*/
#define LORA_TX_RX_TEST   0U

#if LORA_TX_RX_TEST
#define LORA_TEST_TX_PERIOD_MS  500U

static void lora_tx_rx_test(void)
{
   


}
#endif


// uint8_t debug_gpio_signal[BSP_SIG_CH_SIGNAL_MAX] = {0U};
void app_task(void)
{
#if LORA_TX_RX_TEST
    lora_tx_rx_test();              /* 收发验证:绕开协议状态机,直接收发原始字节 */
#else
    app_lora_process();
#endif

    HAL_Delay(50U);                 /* 调试:不进低功耗,一直运行 */

    /* 唤醒后回到循环顶部:喂狗、处理事件、再休眠 */
}



