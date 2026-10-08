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
 * LoRa 协议 v1.5 帧结构(设备 <-> 主机),详见 LoRa_Protocol_v1.5.md
 *
 *   [0]        HEAD = 0x5A
 *   [1]        ADDR   上行 = 源设备地址;下行 = 目标设备地址
 *   [2]        LEN    整帧字节数(含帧头与 CRC),[LORA_FRAME_MIN_LEN, LORA_FRAME_MAX_LEN]
 *   [3]        FUN    功能码
 *   [4..LEN-2] DATA   数据域
 *   [LEN-1]    CRC8   多项式 0x07,初值 0x00,覆盖 [0..LEN-2]
 *============================================================================*/
#define LORA_FRAME_HEAD       0x5AU  /* 帧头                  */
#define LORA_FRAME_MIN_LEN    5U     /* 最小合法帧长          */
#define LORA_FRAME_MAX_LEN    17U    /* 最大合法帧长          */
#define LORA_FRAME_GAP_MS     50U    /* 字节间超时(半帧丢弃) */

/* 帧内字段偏移 */
#define LORA_OFS_HEAD         0U
#define LORA_OFS_ADDR         1U
#define LORA_OFS_LEN          2U
#define LORA_OFS_FUN          3U
#define LORA_OFS_DATA         4U

/* 广播地址:保留,不可用作设备地址 */
#define LORA_ADDR_BROADCAST   0xFFU

/* 上行功能码(设备 -> 主机) */
#define LORA_FUN_HEARTBEAT    0x00U  /* 心跳帧(保留)                              */
#define LORA_FUN_SIGNAL       0x01U  /* 信号帧:DATA = SEQ + SIG                    */
#define LORA_FUN_POWER        0x02U  /* 电量帧:DATA = SEQ + PWR                    */

/* ACK 功能码 = 对应上行功能码 | LORA_FUN_ACK_MASK (0x01->0x81, 0x02->0x82) */
#define LORA_FUN_ACK_MASK     0x80U

/* ACK 结果码(CODE) */
#define LORA_ACK_CODE_OK      0x00U  /* 主机已正确接收          */
#define LORA_ACK_CODE_BADLEN  0x01U  /* 帧长/格式不符,拒收      */
#define LORA_ACK_CODE_REJECT  0x02U  /* 数据无效,拒收(设备记失败) */

/* 下行点名命令功能码(5 字节,无数据域) */
#define LORA_FUN_GET_POWER    0x10U  /* 取电量 */
#define LORA_FUN_GET_SIGNAL   0x11U  /* 取信号 */

/* 信号帧 SIG 取值 */
#define LORA_SIG_DONE         0xAAU  /* 挂接完成(三相均为低电平) */
#define LORA_SIG_UNDONE       0x55U  /* 未完成                   */

/* 电量帧 PWR 无效值 */
#define LORA_PWR_INVALID      0xFFU

/* 信号/电量帧的固定帧长 */
#define LORA_FRAME_LEN_DATA   7U

/* CRC 错误日志限流间隔(仅影响打印,统计计数不受影响) */
#define LORA_CRC_LOG_MS       1000U

/* 主机侧设备表容量(按实际节点数调整) */
#define APP_LORA_DEV_MAX      16U

/**
 * @brief 主机侧按设备维护的状态表条目
 * @note  重发会带新 SEQ,同一数据可能多次到达,这里按"最后一次为准"覆盖
 */
typedef struct
{
    uint8_t  used;        /* 1=槽位已使用                   */
    uint8_t  addr;        /* 设备地址(上行帧的 SRC)         */

    uint8_t  sig_valid;   /* 是否收到过信号帧               */
    uint8_t  sig;         /* 最近一次 SIG                   */
    uint8_t  sig_seq;     /* 最近一次信号帧的 SEQ           */
    uint32_t sig_tick;    /* 最近一次信号帧到达时刻(ms)     */

    uint8_t  pwr_valid;   /* 电量值是否有效(非 0xFF)        */
    uint8_t  pwr;         /* 最近一次 PWR                   */
    uint8_t  pwr_seq;     /* 最近一次电量帧的 SEQ           */
    uint32_t pwr_tick;    /* 最近一次电量帧到达时刻(ms)     */

    uint32_t rx_ok;       /* 已正确接收并回 ACK 的上行帧数   */
    uint32_t crc_err;     /* 该设备 CRC 校验失败帧数         */
} app_lora_dev_t;

// LoRa 协议处理接口================================
/**
 * @brief LoRa 协议处理接口
 * @note 包含心跳、信号、功率帧的发送以及模块周期处理函数
 */
void app_lora_heartbeat(void);

/**
 * @brief 发送信号帧，lora 自定义协议比较简单，只有触点信号帧和电量帧
 * 
 * @param sig 
 */
void app_lora_signal(uint8_t sig);

/**
 * @brief 发送功率帧
 * 
 * @param power 0~100 表示电池电量百分比
 */
void app_lora_power(uint8_t power);

/**
 * @brief 设置外部IO状态(信号)
 * @param signal 信号状态指针
 */
void app_lora_set_signal(uint8_t *signal);

/**
 * @brief 设置电池电量状态指针
 * @param power 电池电量状态指针
 */
void app_lora_set_power(uint8_t *power);

/**
 * @brief LoRa 模块周期处理函数
 * @note  在主循环里周期性调用:
 *        唤醒事件 -> 把 FIFO 字节喂给接收状态机 -> 整帧收齐则校验并分发
 *        (非阻塞:来多少字节吃多少,不做忙等)
 */
void app_lora_process(void);

// LoRa 协议 v1.5 主机侧接口================================
/*==============================================================================
 * 编译开关
 *============================================================================*/
/**
 * @brief ACK 应答总开关
 *   0 = 样机阶段只保留"收":解析上行帧、更新设备表,但不回 ACK
 *       (从机侧 ACK 逻辑待排查,主机先不应答,避免两边互相干扰)
 *   1 = 恢复应答:收到上行帧后 ≤100ms 回 ACK(§8.1)
 * @note  屏蔽的只是"自动应答",app_lora_send_ack() 本身仍保留,便于随时打开
 */
#define APP_LORA_ACK_ENABLE   0

/**
 * @brief 发送 ACK 帧(§5.1,7 字节)
 * @param  dst    目标设备地址(必须精确到设备,不能用广播,否则设备忽略)
 * @param  up_fun 被确认的上行功能码(0x01/0x02),内部自动 |0x80
 * @param  corr   被确认帧的 SEQ(原样回填)
 * @param  code   结果码:0=已正确接收,非 0=拒收
 */
void app_lora_send_ack(uint8_t dst, uint8_t up_fun, uint8_t corr, uint8_t code);

/**
 * @brief 发送点名命令(§5.2,5 字节)
 * @param  dst 目标设备地址(可 0xFF 广播,注意多设备同时应答会碰撞)
 * @param  fun LORA_FUN_GET_POWER 取电量 / LORA_FUN_GET_SIGNAL 取信号
 */
void app_lora_query(uint8_t dst, uint8_t fun);

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