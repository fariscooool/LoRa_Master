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
#include "app_lora_config.h"

#include "app_lora.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "LOG.h"


/*==============================================================================
 * 调试观测变量(在调试器的 Watch / Live Watch 窗口直接查看)
 *============================================================================*/
volatile uint8_t  g_cfg_ok;         /* LoRa 配置"写入+读回校验"结果:1=成功 0=失败 */
static lora_reg_parm_cfg_t lora_cfg_t;

/*==============================================================================
 * 对外接口(供 main.c 调用)
 *============================================================================*/

void app_init(void)
{
    /* M0/M1/AUX 必须由 bsp_gpio_init() 配置成输出/输入:
     * 漏掉这一步 M0/M1 处于复位后的浮空输入态,写 BSRR 不会驱动引脚,
     * 模块模式不可控(表现为能进配置模式就再也回不到透传模式)。 */
    bsp_gpio_init();

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

    g_cfg_ok = app_lora_cfg_reg_verify(lora_cfg_t);

    log_i("LoRa configuration %s", g_cfg_ok ? "successful" : "failed");
}

void app_deinit(void)
{
    /* 暂无需要释放的资源 */
}

// uint8_t debug_gpio_signal[BSP_SIG_CH_SIGNAL_MAX] = {0U};
void app_task(void)
{
    lora_uart_rx_poll();
    
    app_lora_process();

    HAL_Delay(50U);                 /* 调试:不进低功耗,一直运行 */

    /* 唤醒后回到循环顶部:喂狗、处理事件、再休眠 */
}



