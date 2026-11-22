/**
 * @file app_lora_procotol.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief LoRa 协议 v1.6 主机侧实现:上行快照帧解析、UID32 白名单过滤、
 *        按设备建表、ACK 应答、点名命令
 * @version 0.3
 * @date 2026-10-10
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

/* 帧格式与协议常量定义见 app_lora_procotol.h(与 LoRa_Protocol_v1.6.md 一致):
 *   [0] HEAD=0x5A  [1] ADDR  [2] LEN  [3] FUN  [4..7] UID32  [8..LEN-2] DATA  [LEN-1] CRC8
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
 * 主机侧:设备状态表(白名单) + ACK 应答 + 点名命令(§5 / §8)
 *============================================================================*/

static app_lora_dev_t s_dev[APP_LORA_DEV_MAX];

/* UID32 小端解析/打包(§2) */
static uint32_t lora_uid32_get(const uint8_t *f)
{
    return ((uint32_t)f[LORA_OFS_UID32]) |
           ((uint32_t)f[LORA_OFS_UID32 + 1U] << 8) |
           ((uint32_t)f[LORA_OFS_UID32 + 2U] << 16) |
           ((uint32_t)f[LORA_OFS_UID32 + 3U] << 24);
}

static void lora_uid32_put(uint8_t *f, uint32_t uid32)
{
    f[LORA_OFS_UID32]      = (uint8_t)(uid32 & 0xFFU);
    f[LORA_OFS_UID32 + 1U] = (uint8_t)((uid32 >> 8) & 0xFFU);
    f[LORA_OFS_UID32 + 2U] = (uint8_t)((uid32 >> 16) & 0xFFU);
    f[LORA_OFS_UID32 + 3U] = (uint8_t)((uid32 >> 24) & 0xFFU);
}

/* 仅按地址查已登记设备,不新建 */
static app_lora_dev_t *lora_dev_lookup(uint8_t addr)
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

/* 新建条目并绑定 (addr, uid32);表满返回 NULL */
static app_lora_dev_t *lora_dev_alloc(uint8_t addr, uint32_t uid32)
{
    uint8_t i;

    for (i = 0U; i < APP_LORA_DEV_MAX; i++)
    {
        if (s_dev[i].used == 0U)
        {
            memset(&s_dev[i], 0, sizeof(s_dev[i]));
            s_dev[i].used  = 1U;
            s_dev[i].addr  = addr;
            s_dev[i].uid32 = uid32;
            return &s_dev[i];
        }
    }
    return NULL;
}

/* 预先登记白名单条目(§8.1);同 addr 重复登记则覆盖 uid32 */
void app_lora_dev_bind(uint8_t addr, uint32_t uid32)
{
    app_lora_dev_t *dev = lora_dev_lookup(addr);

    if (dev == NULL)
    {
        dev = lora_dev_alloc(addr, uid32);
    }
    else
    {
        dev->uid32 = uid32;
    }

    if (dev == NULL)
    {
        log_e("dev bind: table full, addr=0x%02X dropped", addr);
        return;
    }

    log_i("dev bound: addr=0x%02X uid32=0x%08X", addr, (unsigned)uid32);
}

const app_lora_dev_t *app_lora_dev_find(uint8_t addr)
{
    return (const app_lora_dev_t *)lora_dev_lookup(addr);
}

const app_lora_dev_t *app_lora_dev_table(void)
{
    return s_dev;
}

/* ACK 帧(§5.1):5A DST 0B 83 UID32(4) CORR CODE CRC
 * DST/UID32 必须是目标设备自身的值:填广播值(0xFF/0x00000000)设备一律忽略 */
void app_lora_send_ack(uint8_t dst, uint32_t uid32, uint8_t corr, uint8_t code)
{
    uint8_t f[LORA_FRAME_LEN_ACK];

    f[LORA_OFS_HEAD] = LORA_FRAME_HEAD;
    f[LORA_OFS_ADDR] = dst;
    f[LORA_OFS_LEN]  = LORA_FRAME_LEN_ACK;
    f[LORA_OFS_FUN]  = LORA_FUN_ACK;
    lora_uid32_put(f, uid32);
    f[LORA_OFS_ACK_CORR] = corr;
    f[LORA_OFS_ACK_CODE] = code;
    f[LORA_FRAME_LEN_ACK - 1U] =
        lora_crc8_cal(f, (uint16_t)(LORA_FRAME_LEN_ACK - 1U));

    app_lora_send_bytes(f, (uint16_t)sizeof(f));
}

/* 点名命令(§5.2):5A DST 09 FUN UID32(4) CRC
 * 定向:填目标设备 addr + uid32;广播:0xFF + 0x00000000 */
void app_lora_query(uint8_t dst, uint32_t uid32, uint8_t fun)
{
    uint8_t f[LORA_FRAME_LEN_CMD];

    f[LORA_OFS_HEAD] = LORA_FRAME_HEAD;
    f[LORA_OFS_ADDR] = dst;
    f[LORA_OFS_LEN]  = LORA_FRAME_LEN_CMD;
    f[LORA_OFS_FUN]  = fun;
    lora_uid32_put(f, uid32);
    f[LORA_FRAME_LEN_CMD - 1U] =
        lora_crc8_cal(f, (uint16_t)(LORA_FRAME_LEN_CMD - 1U));

    app_lora_send_bytes(f, (uint16_t)sizeof(f));
}

/* ---- 快照帧(0x03):CRC 已通过,白名单校验 -> 更新设备表 -> 回 ACK(§8) ---- */
static void lora_rx_snapshot(const uint8_t *f)
{
    uint8_t  len = f[LORA_OFS_LEN];
    uint8_t  src = f[LORA_OFS_ADDR];
    uint8_t  seq = f[LORA_OFS_SEQ];
    uint8_t  sig = f[LORA_OFS_SIG];
    uint8_t  pwr = f[LORA_OFS_PWR];
    uint32_t uid = lora_uid32_get(f);
    app_lora_dev_t *dev;

    /* 上行源地址/UID32 不可能是广播值:此时 ACK 无法精确指向设备,只能丢弃 */
    if ((src == LORA_ADDR_BROADCAST) || (uid == LORA_UID32_BROADCAST))
    {
        log_e("uplink: illegal identity src=0x%02X uid32=0x%08X, dropped", src, (unsigned)uid);
        return;
    }

    if (len != LORA_FRAME_LEN_SNAPSHOT)
    {

        /* 长度不符:SEQ/UID32 仍可取出,回非 0 码让设备不必重发 */
        app_lora_send_ack(src, uid, seq, LORA_ACK_CODE_BADLEN);

        log_e("uplink: bad len=%u (fun=0x%02X addr=0x%02X)", len, f[LORA_OFS_FUN], src);
        return;
    }

    dev = lora_dev_lookup(src);
    if (dev != NULL)
    {
        /* §8.1 白名单:addr 已登记但 UID32 不符 -> 不是本套设备,过滤丢弃 */
        if (dev->uid32 != uid)
        {
            dev->filtered++;
            log_w("uplink: uid32 mismatch, filtered (addr=0x%02X got=0x%08X want=0x%08X)",
                  src, (unsigned)uid, (unsigned)dev->uid32);
            return;
        }

        /* 已登记的不用执行操作，后续会确认内容后发送ACK */
    }
    else
    {
        /* 首次上到:用帧内的 (addr, uid32) 绑定白名单 */
        dev = lora_dev_alloc(src, uid);
        if (dev == NULL)
        {
            log_e("uplink: dev table full, drop addr=0x%02X", src);
            return;
        }
        log_i("dev bound: addr=0x%02X uid32=0x%08X", src, (unsigned)uid);
    }

    /* §8.4 重发容错:同一数据可能带不同 SEQ 到达多次,按"最后一次为准"覆盖 */
    dev->sig       = sig;
    dev->sig_valid = 1U;
    dev->pwr       = pwr;
    dev->pwr_valid = (pwr != LORA_PWR_INVALID) ? 1U : 0U;
    dev->last_seq  = seq;
    dev->last_tick = HAL_GetTick();
    dev->rx_ok++;

    if (dev->pwr_valid != 0U)
    {
        log_i("SNAP dev=0x%02X seq=%u sig=0x%02X (%s) pwr=%u%%", src, seq, sig,
              (sig == LORA_SIG_DONE) ? "done" : "undone", pwr);
    }
    else
    {
        log_w("SNAP dev=0x%02X seq=%u sig=0x%02X (%s) pwr=invalid(0xFF)", src, seq, sig,
              (sig == LORA_SIG_DONE) ? "done" : "undone");
    }

    /* §8.2:收到快照帧后 ≤100ms 内回 ACK,CODE=0;CORR 原样回填收到的 SEQ */
    app_lora_send_ack(src, uid, seq, LORA_ACK_CODE_OK);

}

/* ---- 整帧 CRC 通过后:按功能码分发 ---- */
static void lora_rx_parse(const uint8_t *f)
{
    uint8_t fun = f[LORA_OFS_FUN];

    if (fun == LORA_FUN_MATCH)
    {
        /* 配对方式需要另行商讨，配对做完后可以提测。 */
    }

    if (fun == LORA_FUN_SNAPSHOT)
    {
        lora_rx_snapshot(f);
        return;
    }

    /* ACK/点名命令都是主机发出的下行帧,主机侧收到说明是别的套在通信,忽略 */
    if (fun == LORA_FUN_ACK)
    {
        log_w("rx: downlink fun=0x%02X ignored", fun);
        return;
    }

    log_w("rx: unsupported fun=0x%02X from 0x%02X", fun, f[LORA_OFS_ADDR]);
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

    // 1、找帧头
    if(s_rx_idx == 0)
    {
        if(b == LORA_FRAME_HEAD)
        {
            s_rx_buf[0] = b;
            s_rx_idx = 1U;
        }
        return;
    }

    //读取后续数据
    s_rx_buf[s_rx_idx] = b;
    s_rx_idx++;

    /* LEN 字节被找到之前不会执行后续操作 */
    if (s_rx_idx <= LORA_OFS_LEN)
    {
        return;
    }

    /* 刚收完长度字节:要对数据长度做范围检查 */
    if (s_rx_idx == (LORA_OFS_LEN + 1U))    
    {
        uint8_t len = s_rx_buf[LORA_OFS_LEN];

        if ((len < LORA_FRAME_MIN_LEN) || (len > LORA_FRAME_MAX_LEN))
        {
            s_rx_idx = 0U;                  /* 长度非法:丢帧重找帧头 */
        }
        return;
    }

    /* 整帧收满后，先做CRC，校验完成后再做解析 */
    if (s_rx_idx == s_rx_buf[LORA_OFS_LEN]) 
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
            app_lora_dev_t *d = lora_dev_lookup(a);

            if (d != NULL)
            {
                d->crc_err++;
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
