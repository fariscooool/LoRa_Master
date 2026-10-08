/**
 * @file app_lora_config.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "app_lora.h"
#include "app_lora_config.h"
#include "bsp_gpio.h"
#include "main.h"
#include "LOG.h"



/* 单帧接收缓冲:按协议最大帧长取 */
#define LORA_FRAME_BUF_MAX  LORA_FRAME_MAX_LEN

#define LORA_CFG_CMD_WRITE  0xC0U   /* 写寄存器(掉电保存);临时写可改 0xC2 */
#define LORA_CFG_CMD_READ   0xC1U   /* 读参数 */

#define LORA_CFG_SWITCH_MS  20U     /* 模式切换后等待模块稳定(ms) */
#define LORA_CFG_READY_MS   100U    /* 等 AUX 就绪的超时(ms) */
#define LORA_CFG_REPLY_MS   300U    /* 等配置回包的超时(ms) */

/* 单次配置读/写的最大数据字节数 */
#define APP_LORA_CFG_DATA_MAX   5U

/* 厂商回包长度上限:C1C1C1 回显(最多 3 字节) + 5 字节参数 */
#define LORA_CFG_REPLY_MAX      (1U + APP_LORA_CFG_DATA_MAX)

/* 收到第一个字节后,总线静默超过这个时间就认为回包收完
 * (各版本固件回包长度不一:有的回 C1+5 字节,有的回 C1C1C1+5 字节) */
#define LORA_CFG_REPLY_GAP_MS   20U

 /*********************************
 * 一、配置包打包以及接收
 * 亿佰特 E22/E220 系列芯片配置参数需要在休眠模式下:
 *   写: C0 + 5字节工作参数，共6字节（掉电保存）
 *   读: 主机发：C1C1C1 模块回：已配置的参数.
 *       主机发：C3C3C3 模块回：版本信息
 *       主机发：C4C4C4 模块复位
 *********************************/

/* 清空接收 FIFO 残留(交易前后用) */
static void lora_cfg_flush_rx(void)
{
    uint8_t b;

    while (app_lora_read_byte(&b) != 0U)
    {
        /* 丢弃 */
    }
}

/* 进入配置模式:切 M0/M1 + 等模块稳定 + 等 AUX 就绪 */
static void lora_cfg_mode_enter(void)
{
    uint32_t t0;

    app_lora_set_mode(LORA_MODE_SLEEP);         /* M1=1,M0=1:配置模式 */
    HAL_Delay(LORA_CFG_SWITCH_MS);

    /* 空闲时 AUX 为高;超时也继续,按最快路径尝试 */
    t0 = HAL_GetTick();
    while ((bsp_gpio_read(BSP_LORA_AUX) == BSP_GPIO_LOW) &&
           ((HAL_GetTick() - t0) < LORA_CFG_READY_MS))
    {
        HAL_Delay(1U);
    }
}

/* 退出配置模式:切回正常传输模式
 * ⚠ 这一句不能省:休眠(配置)模式下模块射频是关的,收不到任何空中数据,
 *   串口也不会有输出(表现为 USART6/DMA 中断永远不进)。 */
static void lora_cfg_mode_exit(void)
{
    app_lora_set_mode(LORA_MODE_NORMAL);
    HAL_Delay(LORA_CFG_SWITCH_MS);
}

/* 发一帧厂商帧并尽量把回包收下来。
 * 厂商回包长度随固件版本而异(6 字节 C1+5参数,或 8 字节 C1C1C1+5参数),
 * 所以不按固定长度等,而是“收到第一个字节后等总线静默”来判帧尾。
 * @retval 实际收到的字节数(0 = 一个字节都没收到) */
static uint8_t lora_cfg_transaction(uint8_t *tx, uint8_t tx_len,
                                    uint8_t *rx, uint8_t rx_cap)
{
    uint32_t t0;
    uint32_t tlast;
    uint8_t  got = 0U;

    lora_cfg_flush_rx();
    app_lora_send_bytes(tx, tx_len);

    if ((rx == NULL) || (rx_cap == 0U))
    {
        return 0U;
    }

    t0 = tlast = HAL_GetTick();
    while ((HAL_GetTick() - t0) < LORA_CFG_REPLY_MS)
    {
        if ((got < rx_cap) && (app_lora_read_byte(&rx[got]) != 0U))
        {
            got++;
            tlast = HAL_GetTick();   /* 每收到一个字节续期,容忍慢回包 */
            continue;
        }

        if ((got > 0U) && ((HAL_GetTick() - tlast) >= LORA_CFG_REPLY_GAP_MS))
        {
            break;                   /* 有回包且已静默 -> 一帧收完 */
        }
    }

    if (got == 0U)
    {
        log_e("cfg: no reply within %u ms", (unsigned)LORA_CFG_REPLY_MS);
    }

    return got;
}

uint8_t app_lora_cfg_write(lora_reg_parm_cfg_t cfg)
{
    uint8_t tx[1U + APP_LORA_CFG_DATA_MAX];
    uint8_t ok = 0U;
    uint8_t i;

    tx[0] = LORA_CFG_CMD_WRITE;

    for (i = 0U; i < APP_LORA_CFG_DATA_MAX; i++)
    {
        tx[1U + i] = cfg.data[i];
    }

    lora_cfg_mode_enter();
    /* 写指令:有的固件会回显(C0 + 参数),有的不回;不管哪种都读进来打印,
     * 方便对照。写是否真正生效由 app_lora_cfg_reg_verify() 的读回比对判定。 */
    {
        (void)lora_cfg_transaction(tx, (uint8_t)(1U + APP_LORA_CFG_DATA_MAX), NULL, 0);
    }
    ok = 1U;
    lora_cfg_mode_exit();

    return ok;
}


uint8_t app_lora_cfg_read(lora_reg_parm_cfg_t *buf)
{
    uint8_t tx[3U];
    /* 回包缓冲区：C0 + 参数 */
    uint8_t rx[LORA_CFG_REPLY_MAX + 1];
    uint8_t ok = 0U;
    uint8_t i;

    if (buf == NULL)
    {
        return 0U;
    }

    tx[0] = LORA_CFG_CMD_READ;
    tx[1] = LORA_CFG_CMD_READ;
    tx[2] = LORA_CFG_CMD_READ;

    lora_cfg_mode_enter();

    uint8_t got = lora_cfg_transaction(tx, 3U, rx, (uint8_t)sizeof(rx));

    if (got == 0U)
    {
        log_e("cfg read: no reply");
        lora_cfg_mode_exit();
        return 0U;
    }

    if (got >= (1U + APP_LORA_CFG_DATA_MAX))
    {
        for (i = 0U; i < APP_LORA_CFG_DATA_MAX; i++)
        {
            buf->data[i] = rx[1U + i];
        }
        ok = 1U;
    }
    else
    {
        log_e("cfg read: bad reply.");
    }

    lora_cfg_mode_exit();

    return ok;
}

/**
 * @brief 一次配置模式下完成 "写 + 读回" 并比对
 * @retval 1=写回显与读回都一致 0=失败
 */
static uint8_t lora_cfg_write_read_verify(const lora_reg_parm_cfg_t val)
{
    uint8_t tx[1U + APP_LORA_CFG_DATA_MAX];
    lora_reg_parm_cfg_t rd;
    uint8_t ok = 0U;
    uint8_t i;

    lora_cfg_mode_enter();

    /* 1) 写:模块应原样回显 */
    tx[0] = LORA_CFG_CMD_WRITE;
    for (i = 0U; i < APP_LORA_CFG_DATA_MAX; i++)
    {
        tx[1U + i] = val.data[i];
    }
    if (lora_cfg_transaction(tx, (uint8_t)(1U + APP_LORA_CFG_DATA_MAX), NULL, 0) != 0U)
    {
        ok = 1U;
    }

    /* 2) 读回数据,解析后与期望值逐个比对 */
    while(!app_lora_rx_available())
    {
        /* 等待接收缓冲区有数据 */
        ;
    }
    if(app_lora_cfg_read(&rd) != 0U)
    {
        for (i = 0U; i < APP_LORA_CFG_DATA_MAX; i++)
        {
            if (rd.data[i] != val.data[i])
            {
                ok = 0U;
                log_e("LoRa configuration verification failed at byte %d", i);
                break;
            }
            log_i("LoRa configuration verification successful at byte %d", i);
            ok = 1U;
        }
    }
    else {
        ok = 0;
    }

    lora_cfg_mode_exit();

    return ok;
}

uint8_t app_lora_cfg_reg_verify(lora_reg_parm_cfg_t val)
{
    return lora_cfg_write_read_verify(val);
}
