#ifndef APP_LORA_PROCOTOL_H
#define APP_LORA_PROCOTOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_lora.h"


/**
 * @brief 主机下发的控制参数(解析后的落地值)
 * @note  由 app_lora_process() 中的接收状态机解析后更新
 */
typedef struct
{
    uint8_t  report_enable;      /* 0=停止上报 1=允许上报 */
    uint16_t report_period_s;    /* 周期上报间隔(秒)      */
    uint16_t device_addr;        /* 模块地址              */
    uint8_t  channel;            /* 模块信道              */
} app_lora_ctrl_t;

/*==============================================================================
 * LoRa 协议 v1.6 帧结构(设备 <-> 主机),详见 LoRa_Protocol_v1.6.md
 *
 *   [0]        HEAD = 0x5A
 *   [1]        ADDR   上行 = 源设备地址;下行 = 目标设备地址(广播 = 0xFF)
 *   [2]        LEN    整帧字节数(含帧头与 CRC),[LORA_FRAME_MIN_LEN, LORA_FRAME_MAX_LEN]
 *   [3]        FUN    功能码
 *   [4..7]     UID32  设备短标识(小端,非广播帧必填;广播帧填 0)
 *   [8..LEN-2] DATA   数据域(内容随帧型变化)
 *   [LEN-1]    CRC8   多项式 0x07,初值 0x00,覆盖 [0..LEN-2]
 *
 * 上行快照帧(0x03,12 字节):
 *   5A SRC 0C 03 UID32(4) SEQ SIG PWR CRC
 * 下行 ACK 帧(0x83,11 字节):
 *   5A DST 0B 83 UID32(4) CORR CODE CRC
 * 下行点名命令(0x10/0x11,9 字节):
 *   5A DST 09 FUN UID32(4) CRC
 *============================================================================*/
#define LORA_FRAME_HEAD       0x5AU  /* 帧头                  */
#define LORA_FRAME_MIN_LEN    9U     /* 最小合法帧长          */
#define LORA_FRAME_MAX_LEN    24U    /* 最大合法帧长          */
#define LORA_FRAME_GAP_MS     50U    /* 字节间超时(半帧丢弃) */

/* 帧内字段偏移 */
#define LORA_OFS_HEAD         0U
#define LORA_OFS_ADDR         1U
#define LORA_OFS_LEN          2U
#define LORA_OFS_FUN          3U
#define LORA_OFS_UID32        4U     /* 4 字节,低字节在前(偏移 4..7) */

/* 上行快照帧(0x03)数据域偏移 */
#define LORA_OFS_SEQ          8U     /* 发送序号(重发换新号)         */
#define LORA_OFS_SIG          9U     /* 挂接状态                     */
#define LORA_OFS_PWR          10U    /* 电量百分比                   */

/* 下行 ACK 帧(0x83)数据域偏移 */
#define LORA_OFS_ACK_CORR     8U     /* 被确认帧的 SEQ(原样回填)     */
#define LORA_OFS_ACK_CODE     9U     /* 结果码                       */

/* 广播地址与广播 UID32:保留,不可用作设备地址/设备 UID32 */
#define LORA_ADDR_BROADCAST   0xFFU
#define LORA_UID32_BROADCAST  0x00000000UL

/* 功能码 */
#define LORA_FUN_SNAPSHOT     0x03U  /* 上行状态快照:UID32 + SEQ + SIG + PWR  */
#define LORA_FUN_ACK          0x83U  /* 下行 ACK:UID32 + CORR + CODE          */
#define LORA_FUN_GET_POWER    0x10U  /* 下行点名取电量                        */
#define LORA_FUN_GET_SIGNAL   0x11U  /* 下行点名取信号                        */

/* ACK 结果码(CODE) */
#define LORA_ACK_CODE_OK      0x00U  /* 主机已正确接收            */
#define LORA_ACK_CODE_BADLEN  0x01U  /* 帧长/格式不符,拒收        */
#define LORA_ACK_CODE_REJECT  0x02U  /* 数据无效,拒收(设备记拒绝) */

/* 快照帧 SIG 取值 */
#define LORA_SIG_DONE         0xAAU  /* 挂接完成(三相均为低电平) */
#define LORA_SIG_UNDONE       0x55U  /* 未完成                   */

/* 快照帧 PWR 无效值 */
#define LORA_PWR_INVALID      0xFFU

/* 各帧型固定帧长 */
#define LORA_FRAME_LEN_SNAPSHOT  12U /* 快照帧 0x03  */
#define LORA_FRAME_LEN_ACK       11U /* ACK 帧 0x83  */
#define LORA_FRAME_LEN_CMD       9U  /* 点名命令 0x10/0x11 */

/* CRC 错误日志限流间隔(仅影响打印,统计计数不受影响) */
#define LORA_CRC_LOG_MS       1000U

/* 主机侧设备表容量(按实际节点数调整) */
#define APP_LORA_DEV_MAX      16U

/**
 * @brief 主机侧按设备维护的状态表条目
 * @note  重发会换新 SEQ,同一数据可能多次到达,这里按"最后一次为准"覆盖
 */
typedef struct
{
    uint8_t  used;        /* 1=槽位已使用(已登记/已绑定)      */
    uint8_t  addr;        /* 设备地址(上行帧的 SRC)           */
    uint32_t uid32;       /* 设备 UID32(白名单比对依据)       */

    uint8_t  sig_valid;   /* 是否收到过快照帧                 */
    uint8_t  sig;         /* 最近一次 SIG                     */
    uint8_t  pwr_valid;   /* 电量值是否有效(非 0xFF)          */
    uint8_t  pwr;         /* 最近一次 PWR                     */
    uint8_t  last_seq;    /* 最近一次快照帧的 SEQ             */
    uint32_t last_tick;   /* 最近一次快照帧到达时刻(ms)       */

    uint32_t rx_ok;       /* 已正确接收并回 ACK 的快照帧数     */
    uint32_t crc_err;     /* 该设备 CRC 校验失败帧数           */
    uint32_t filtered;    /* 该地址上 UID32 不匹配被丢弃的帧数 */
} app_lora_dev_t;

// LoRa 协议处理接口================================
/**
 * @brief LoRa 模块周期处理函数
 * @note  在主循环里周期性调用:
 *        唤醒事件 -> 把 FIFO 字节喂给接收状态机 -> 整帧收齐则校验并分发
 *        (非阻塞:来多少字节吃多少,不做忙等)
 */
void app_lora_process(void);

// LoRa 协议 v1.6 主机侧接口================================
/*==============================================================================
 * 编译开关
 *============================================================================*/
/**
 * @brief ACK 应答总开关(§8.2)
 *   1 = 收到快照帧后立即回 ACK(协议要求 ≤100ms 内),设备据此结束本轮
 *   0 = 只收不应答:解析上行帧、更新设备表,但不回 ACK(样机排查用)
 * @note  关闭的只是"自动应答",app_lora_send_ack() 本身仍保留,便于随时打开
 */
#define APP_LORA_ACK_ENABLE   1

/**
 * @brief 发送 ACK 帧(§5.1,11 字节)
 * @param  dst   目标设备地址(必须精确到设备,不能用广播,否则设备忽略)
 * @param  uid32 目标设备 UID32(同上,广播 0 会被设备忽略)
 * @param  corr  被确认帧的 SEQ(原样回填)
 * @param  code  结果码:0=已正确接收,非 0=拒收(设备不再重发)
 */
void app_lora_send_ack(uint8_t dst, uint32_t uid32, uint8_t corr, uint8_t code);

/**
 * @brief 发送点名命令(§5.2,9 字节)
 * @param  dst   目标设备地址(广播填 LORA_ADDR_BROADCAST,注意多设备同时应答会碰撞)
 * @param  uid32 目标设备 UID32(广播填 LORA_UID32_BROADCAST)
 * @param  fun   LORA_FUN_GET_POWER 取电量 / LORA_FUN_GET_SIGNAL 取信号
 * @note  设备收到后立即回一帧 0x03 快照帧,不等 ACK;主机按请求自行关联
 */
void app_lora_query(uint8_t dst, uint32_t uid32, uint8_t fun);

/**
 * @brief 预先登记设备白名单(§8.1,可选)
 * @param  addr  设备地址
 * @param  uid32 设备 UID32(产线台账/UART 日志获取)
 * @note  已登记的设备只接受同 (addr, uid32) 的帧;未登记的设备在首次上报时
 *        自动绑定。同 addr 重复登记会覆盖原 uid32。
 */
void app_lora_dev_bind(uint8_t addr, uint32_t uid32);

/**
 * @brief 查找某台设备在主机的状态表条目
 * @param  addr 设备地址
 * @retval 指向设备条目的指针,未找到返回 NULL
 */
const app_lora_dev_t *app_lora_dev_find(uint8_t addr);

/**
 * @brief 获取设备状态表首地址(遍历用,有效长度 APP_LORA_DEV_MAX)
 * @retval 设备表指针
 */
const app_lora_dev_t *app_lora_dev_table(void);


#ifdef __cplusplus
}
#endif

#endif // APP_LORA_PROCOTOL_H