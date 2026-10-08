/**
 * @file app_lora_procotol.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief LoRa 协议 v1.5 主机侧实现:上行帧解析、按设备建表、ACK 应答、点名命令
 *        (样机阶段 ACK 由 APP_LORA_ACK_ENABLE 关闭,只做接收)
 * @version 0.2
 * @date 2026-10-09
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

/* 帧格式与协议常量定义见 app_lora_procotol.h(与 LoRa_Protocol_v1.5.md 一致):
 *   [0] HEAD=0x5A  [1] ADDR  [2] LEN  [3] FUN  [4..LEN-2] DATA  [LEN-1] CRC8
 */

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

/*==============================================================================
 * 主机侧:设备状态表 + ACK 应答 + 点名命令(§5 / §8)
 *============================================================================*/

static app_lora_dev_t s_dev[APP_LORA_DEV_MAX];

/* 查找设备条目;不存在则在空槽新建(表满返回 NULL) */
static app_lora_dev_t *lora_dev_get(uint8_t addr)
{
    uint8_t i;
    app_lora_dev_t *slot = NULL;

    for (i = 0U; i < APP_LORA_DEV_MAX; i++)
    {
        if (s_dev[i].used != 0U)
        {
            if (s_dev[i].addr == addr)
            {
                return &s_dev[i];
            }
        }
        else if (slot == NULL)
        {
            slot = &s_dev[i];
        }
    }

    if (slot != NULL)
    {
        memset(slot, 0, sizeof(*slot));
        slot->used = 1U;
        slot->addr = addr;
    }

    return slot;
}

const app_lora_dev_t *app_lora_dev_find(uint8_t addr)
{
    uint8_t i;

    for (i = 0U; i < APP_LORA_DEV_MAX; i++)
    {
        if ((s_dev[i].used != 0U) && (s_dev[i].addr == addr))
        {
            return &s_dev[i];
        }
    }
    return NULL;
}

const app_lora_dev_t *app_lora_dev_table(void)
{
    return s_dev;
}

/* ACK 帧(§5.1):5A DST 07 (FUN|0x80) CORR CODE CRC */
void app_lora_send_ack(uint8_t dst, uint8_t up_fun, uint8_t corr, uint8_t code)
{
    uint8_t f[LORA_FRAME_LEN_DATA];

    f[LORA_OFS_HEAD] = LORA_FRAME_HEAD;
    f[LORA_OFS_ADDR] = dst;
    f[LORA_OFS_LEN]  = LORA_FRAME_LEN_DATA;
    f[LORA_OFS_FUN]  = (uint8_t)(up_fun | LORA_FUN_ACK_MASK);
    f[4]             = corr;
    f[5]             = code;
    f[6]             = lora_crc8_cal(f, (uint16_t)(sizeof(f) - 1U));

    app_lora_send_bytes(f, (uint16_t)sizeof(f));
}

/* 点名命令(§5.2):5A DST 05 FUN CRC */
void app_lora_query(uint8_t dst, uint8_t fun)
{
    uint8_t f[5U];

    f[LORA_OFS_HEAD] = LORA_FRAME_HEAD;
    f[LORA_OFS_ADDR] = dst;
    f[LORA_OFS_LEN]  = 5U;
    f[LORA_OFS_FUN]  = fun;
    f[4]             = lora_crc8_cal(f, 4U);

    app_lora_send_bytes(f, (uint16_t)sizeof(f));
}

/* ---- 整帧校验通过后:按功能码处理上行帧;ACK 由 APP_LORA_ACK_ENABLE 控制 ---- */
static void lora_rx_parse(const uint8_t *f)
{
    uint8_t len = f[LORA_OFS_LEN];
    uint8_t src = f[LORA_OFS_ADDR];
    uint8_t fun = f[LORA_OFS_FUN];
    app_lora_dev_t *dev;

    /* 上行源地址不可能是广播地址:此时 ACK 无法精确指向设备,只能丢弃 */
    if (src == LORA_ADDR_BROADCAST)
    {
        log_e("uplink: broadcast src addr, dropped (fun=0x%02X)", fun);
        return;
    }

    if ((fun != LORA_FUN_SIGNAL) && (fun != LORA_FUN_POWER))
    {
        log_w("uplink: unsupported fun=0x%02X from 0x%02X", fun, src);
        return;
    }

    if (len != LORA_FRAME_LEN_DATA)
    {
#if APP_LORA_ACK_ENABLE
        /* 长度不符:能取到 SEQ 就回非 0 码,让设备不必重发 */
        if (len >= (LORA_OFS_DATA + 1U))
        {
            app_lora_send_ack(src, fun, f[LORA_OFS_DATA], LORA_ACK_CODE_BADLEN);
        }
#endif
        log_e("uplink: bad len=%u (fun=0x%02X addr=0x%02X)", len, fun, src);
        return;
    }

    dev = lora_dev_get(src);
    if (dev == NULL)
    {
        log_e("uplink: dev table full, drop addr=0x%02X", src);
        return;
    }

    if (fun == LORA_FUN_SIGNAL)
    {
        dev->sig       = f[LORA_OFS_DATA + 1U];
        dev->sig_seq   = f[LORA_OFS_DATA];
        dev->sig_valid = 1U;
        dev->sig_tick  = HAL_GetTick();
        log_i("SIG  dev=0x%02X seq=%u sig=0x%02X (%s)", src, dev->sig_seq, dev->sig,
              (dev->sig == LORA_SIG_DONE) ? "done" : "undone");
    }
    else /* LORA_FUN_POWER */
    {
        dev->pwr       = f[LORA_OFS_DATA + 1U];
        dev->pwr_seq   = f[LORA_OFS_DATA];
        dev->pwr_valid = (dev->pwr != LORA_PWR_INVALID) ? 1U : 0U;
        dev->pwr_tick  = HAL_GetTick();
        if (dev->pwr_valid != 0U)
        {
            log_i("PWR  dev=0x%02X seq=%u pwr=%u%%", src, dev->pwr_seq, dev->pwr);
        }
        else
        {
            log_w("PWR  dev=0x%02X seq=%u invalid(0xFF)", src, dev->pwr_seq);
        }
    }

    dev->rx_ok++;

#if APP_LORA_ACK_ENABLE
    /* §8.1:收到上行帧后在 100ms 内回 ACK(CODE=0 表示已正确接收) */
    app_lora_send_ack(src, fun, f[LORA_OFS_DATA], LORA_ACK_CODE_OK);
#endif
}

/* ---- 接收:找帧头 -> 收够 LEN 字节 -> CRC 校验 -> 解析 ---- */
static uint8_t  s_rx_buf[LORA_FRAME_MAX_LEN];
static uint8_t  s_rx_idx;       /* 0 = 正在找帧头,其余为已收字节数 */
static uint32_t s_rx_tick;      /* 最近一个字节的时刻,用于字节间超时 */
static uint32_t s_crc_log_tick; /* CRC 错误日志上次打印时刻(限流用) */
static uint8_t  s_crc_logged;   /* 是否已打印过 CRC 错误(限流用)     */

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

    /* LEN 字节(偏移 LORA_OFS_LEN)还没收齐前不能做长度判断:
     * 否则会读到缓冲区里上一帧残留的陈旧 LEN,把帧头+地址误判成整帧 */
    if (s_rx_idx <= LORA_OFS_LEN)
    {
        return;
    }

    if (s_rx_idx == (LORA_OFS_LEN + 1U))    /* 刚收完长度字节:先做范围检查 */
    {
        uint8_t len = s_rx_buf[LORA_OFS_LEN];

        if ((len < LORA_FRAME_MIN_LEN) || (len > LORA_FRAME_MAX_LEN))
        {
            s_rx_idx = 0U;                  /* 长度非法:丢帧重找帧头 */
        }
        return;
    }

    if (s_rx_idx == s_rx_buf[LORA_OFS_LEN]) /* 整帧收满:CRC 对才解析 */
    {
        uint8_t len = s_rx_buf[LORA_OFS_LEN];

        if (lora_crc8_cal(s_rx_buf, (uint16_t)(len - 1U)) == s_rx_buf[len - 1U])
        {
            lora_rx_parse(s_rx_buf);
        }
        else
        {
            /* CRC 失败:帧内容不可信,不新建条目,仅对已知设备做尽力归属 */
            uint8_t a = s_rx_buf[LORA_OFS_ADDR];
            uint8_t i;

            for (i = 0U; i < APP_LORA_DEV_MAX; i++)
            {
                if ((s_dev[i].used != 0U) && (s_dev[i].addr == a))
                {
                    s_dev[i].crc_err++;
                    break;
                }
            }

            /* 限流打印:空口噪声可能连续触发,避免把 RTT 刷爆(计数不受影响) */
            if ((s_crc_logged == 0U) ||
                ((HAL_GetTick() - s_crc_log_tick) >= LORA_CRC_LOG_MS))
            {
                s_crc_logged   = 1U;
                s_crc_log_tick = HAL_GetTick();
                log_e("rx: crc error (len=%u, addr~0x%02X)", len, a);
            }
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
