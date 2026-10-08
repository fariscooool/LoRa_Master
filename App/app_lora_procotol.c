/**
 * @file app_lora_procotol.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief LoRa 协议处理:数据拆包解析 / 装包发送(待自己实现)
 * @version 0.1
 * @date 2026-09-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "app_lora.h"
#include "app_lora_procotol.h"
#include "bsp_gpio.h"
#include "main.h"
#include "LOG.h"

/* 自定义帧格式(收发一致,与现有发送函数保持一致):
 *   [0]      0x5A
 *   [1]      LEN  = 整帧字节数(含帧头与 CRC)
 *   [2]      FUN  功能码
 *   [3..]    DATA 数据域,长度 = LEN - 4
 *   [LEN-1]  CRC8 (对 [0..LEN-2] 计算,多项式 0x07)
 * 举例:心跳帧 0x5A 0x04 0x00 0xF7
 */
#define LORA_FRAME_HEAD     0x5A
#define LORA_FRAME_MIN_LEN  4U      /* 头+长度+FUN+CRC(数据域为空) */
#define LORA_FRAME_DATA_MAX 12U     /* 数据域上限 */
#define LORA_FRAME_BUF_MAX  (LORA_FRAME_MIN_LEN + LORA_FRAME_DATA_MAX)
#define LORA_FRAME_GAP_MS   50U     /* 字节间超时:超时则丢弃半帧 */

/* 上行(设备 -> 主机)功能码 */
#define LORA_FUN_HEARTBEAT 0x00     // 心跳帧
#define LORA_FUN_SIGNAL 0x01        // 信号帧，主要是ABC相开关状态
#define LORA_FUN_POWER 0x02         // 电池电量帧

/* 下行(主机 -> 设备)功能码:控制参数 */
#define LORA_FUN_GET_POWER   0x10U  /* 数据 0B:获取电池电量*/
#define LORA_FUN_GET_SIGNAL   0x11U  /* 数据 1B:主动获取ABC相开关状态*/
// #define LORA_FUN_SET_ADDR     0x12U  /* 数据 2B:模块地址(小端,掉电保存)  */
// #define LORA_FUN_SET_CHANNEL  0x13U  /* 数据 1B:模块信道(掉电保存)       */
// #define LORA_FUN_QUERY_CFG    0x14U  /* 数据 0B:请求回读当前配置         */

#define LORA_CFG_CMD_WRITE  0xC0U   /* 写寄存器(掉电保存);临时写可改 0xC2 */
#define LORA_CFG_CMD_READ   0xC1U   /* 读参数 */

#define LORA_CFG_SWITCH_MS  20U     /* 模式切换后等待模块稳定(ms) */
#define LORA_CFG_READY_MS   100U    /* 等 AUX 就绪的超时(ms) */
#define LORA_CFG_REPLY_MS   300U    /* 等配置回包的超时(ms) */

/* 单次配置读/写的最大数据字节数 */
#define APP_LORA_CFG_DATA_MAX   5U

/* 厂商回包长度上限:C1C1C1 回显(最多 3 字节) + 5 字节参数 */
#define LORA_CFG_REPLY_MAX      (3U + APP_LORA_CFG_DATA_MAX)
/* 收到第一个字节后,总线静默超过这个时间就认为回包收完
 * (各版本固件回包长度不一:有的回 C1+5 字节,有的回 C1C1C1+5 字节) */
#define LORA_CFG_REPLY_GAP_MS   20U


/*********************************
 * 静态函数
 *********************************/
// CRC 8-CCITT (多项式 0x07)
static uint8_t lora_crc8_cal(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07; /* CRC-8-CCITT 多项式 */
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* 清空接收 FIFO 残留(交易前后用) */
static void lora_cfg_flush_rx(void)
{
    uint8_t b;

    while (app_lora_read_byte(&b) != 0U)
    {
        /* 丢弃 */
    }
}

/*********************************
 * 一、配置包打包以及接收
 * 亿佰特 E22/E220 系列芯片配置参数需要在休眠模式下:
 *   写: C0 + 5字节工作参数，共6字节（掉电保存）
 *   读: 主机发：C1C1C1 模块回：已配置的参数.
 *       主机发：C3C3C3 模块回：版本信息
 *       主机发：C4C4C4 模块复位
 *********************************/
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

uint8_t app_lora_cfg_write(lora_reg_parm_cfg_t cfg, uint8_t len)
{
    uint8_t tx[1U + APP_LORA_CFG_DATA_MAX];
    uint8_t ok = 0U;
    uint8_t i;

    if ((len == 0U) || (len > APP_LORA_CFG_DATA_MAX))
    {
        return 0U;
    }

    tx[0] = LORA_CFG_CMD_WRITE;

    for (i = 0U; i < len; i++)
    {
        tx[1U + i] = cfg.data[i];
    }

    lora_cfg_mode_enter();
    /* 写指令:有的固件会回显(C0 + 参数),有的不回;不管哪种都读进来打印,
     * 方便对照。写是否真正生效由 app_lora_cfg_reg_verify() 的读回比对判定。 */
    {
        uint8_t echo[LORA_CFG_REPLY_MAX];

        (void)lora_cfg_transaction(tx, (uint8_t)(1U + len), echo, (uint8_t)sizeof(echo));
    }
    ok = 1U;
    lora_cfg_mode_exit();

    return ok;
}


uint8_t app_lora_cfg_read(lora_reg_parm_cfg_t *buf, uint8_t len)
{
    uint8_t tx[3U];
    /* 回包:开头是命令回显(1~3 个 0xC1),后面才是 5 字节参数 */
    uint8_t rx[LORA_CFG_REPLY_MAX];
    uint8_t got = 0U;
    uint8_t n_cmd = 0U;
    uint8_t ok = 0U;
    uint8_t i;

    if ((buf == NULL) || (len == 0U) || (len > APP_LORA_CFG_DATA_MAX))
    {
        return 0U;
    }

    tx[0] = LORA_CFG_CMD_READ;
    tx[1] = LORA_CFG_CMD_READ;
    tx[2] = LORA_CFG_CMD_READ;

    lora_cfg_mode_enter();

    got = lora_cfg_transaction(tx, 3U, rx, (uint8_t)sizeof(rx));

    /* 跳过开头的命令回显字节(0xC1),剩下的就是参数 */
    while ((n_cmd < got) && (rx[n_cmd] == LORA_CFG_CMD_READ))
    {
        n_cmd++;
    }

    if ((uint8_t)(got - n_cmd) >= len)
    {
        for (i = 0U; i < len; i++)
        {
            buf->data[i] = rx[n_cmd + i];
        }
        ok = 1U;
    }
    else
    {
        log_e("cfg read: bad reply (got %d bytes, %d cmd echo)", got, n_cmd);
    }

    lora_cfg_mode_exit();

    return ok;
}

/**
 * @brief 一次配置模式下完成 "写 + 读回" 并比对
 * @retval 1=写回显与读回都一致 0=失败
 */
static uint8_t lora_cfg_write_read_verify(const lora_reg_parm_cfg_t val, uint8_t len)
{
    uint8_t tx[1U + APP_LORA_CFG_DATA_MAX];
    lora_reg_parm_cfg_t rd;
    uint8_t ok = 0U;
    uint8_t i;

    if (len != 5U)
    {
        return 0U;
    }

    lora_cfg_mode_enter();

    /* 1) 写:模块应原样回显 */
    tx[0] = LORA_CFG_CMD_WRITE;
    for (i = 0U; i < len; i++)
    {
        tx[1U + i] = val.data[i];
    }
    if (lora_cfg_transaction(tx, (uint8_t)(1U + len), NULL, 0) != 0U)
    {
        ok = 1U;
    }

    /* 2) 读回数据,解析后与期望值逐个比对 */
    if(app_lora_cfg_read(&rd, (uint8_t)sizeof(rd.data)) != 0U)
    {
        for (i = 0U; i < len; i++)
        {
            if (rd.data[i] != val.data[i])
            {
                ok = 0U;
                log_e("LoRa configuration verification failed at byte %d", i);
                break;
            }
            log_i("LoRa configuration verification successful at byte %d", i);
        }
    }
    else {
        ok = 0;
    }

    lora_cfg_mode_exit();

    return ok;
}

uint8_t app_lora_cfg_reg_verify(lora_reg_parm_cfg_t val, uint8_t len)
{
    return lora_cfg_write_read_verify(val, len);
}


/* lora 接收消息解析 */
/* ---- 整帧校验通过后:功能码 switch 直接落地,长度不符/未知功能码一律丢弃 ---- */
static void lora_rx_parse(const uint8_t *f)
{
    uint8_t fun = f[2];
    switch (fun)
    {
        case LORA_FUN_SIGNAL:
            /* 处理信号功能码的逻辑 */
            log_i("Signal received, %d", f[3]);
            break;
        case LORA_FUN_POWER:
            /* 处理电源功能码的逻辑 */
            log_i("Power received, %d", f[3]);

            break;
        default:
            break;
    }
    (void)f;
}

/* ---- 接收:找帧头 -> 按 LEN 收满 -> CRC 校验 -> 解析 ---- */
static uint8_t  s_rx_buf[LORA_FRAME_BUF_MAX];
static uint8_t  s_rx_idx;       /* 0 = 正在找帧头,其余为已收字节数 */
static uint32_t s_rx_tick;      /* 最近一个字节的时刻,用于字节间超时 */

/* 喂一个字节:无阻塞、无忙等,有多少吃多少 */
static void lora_rx_byte(uint8_t b)
{
    s_rx_tick = HAL_GetTick();

    if (s_rx_idx == 0U)                     /* 找帧头:非帧头字节直接丢 */
    {
        if (b == LORA_FRAME_HEAD)
        {
            s_rx_buf[0] = b;
            s_rx_idx    = 1U;
        }
        return;
    }

    s_rx_buf[s_rx_idx] = b;
    s_rx_idx++;

    if (s_rx_idx == 2U)                     /* 刚收完长度字节:先做范围检查 */
    {
        uint8_t len = s_rx_buf[1];

        if ((len < LORA_FRAME_MIN_LEN) || (len > (uint8_t)sizeof(s_rx_buf)))
        {
            s_rx_idx = 0U;                  /* 长度非法:丢帧重找帧头 */
        }
        return;
    }

    if (s_rx_idx == s_rx_buf[1])            /* 整帧收满:CRC 对才解析 */
    {
        uint8_t len = s_rx_buf[1];

        if (lora_crc8_cal(s_rx_buf, (uint16_t)(len - 1U)) == s_rx_buf[len - 1U])
        {
            lora_rx_parse(s_rx_buf);
        }
        s_rx_idx = 0U;
    }
}

void app_lora_process(void)
{
    uint8_t b;

    /* 无唤醒事件 / 正在发送:本轮不拆包 */
    if (app_lora_get_status() != LORA_IDLE)
    {
        return;
    }

    /* 1) 收:把接收 FIFO 里的字节全部喂给状态机(非阻塞,有多少吃多少) */
    while (app_lora_read_byte(&b) != 0U)
    {
        lora_rx_byte(b);
    }

    /* 2) 半帧超时保护:字节间超过 LORA_FRAME_GAP_MS 就丢弃重找帧头 */
    if ((s_rx_idx != 0U) && ((HAL_GetTick() - s_rx_tick) > LORA_FRAME_GAP_MS))
    {
        s_rx_idx = 0U;
    }
}
