/**
 * @file bsp_gpio.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief GPIO 驱动实现文件
 * @version 0.1
 * @date 2026-10-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_gpio.h"
#include "stm32f446xx.h"

#define EXTI_NULL 0xFFFFFFFF

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      mode;
    uint32_t      pull;
    uint32_t      speed;
    uint32_t      alternate;
    uint32_t      exti_irq;    /* 该引脚所属的 EXTI 中断号 */
    uint32_t      pre_priority;
    uint32_t      sub_priority;
} bsp_sig_map_t;


static const bsp_sig_map_t bsp_sig_map[BSP_SIG_CH_MAX] =
{
    
    [BSP_LORA_AUX] = { GPIOC, GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0, EXTI_NULL, 0, 0 },
    [BSP_LORA_M0] = { GPIOB, GPIO_PIN_15, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0, EXTI_NULL, 0, 0 },
    [BSP_LORA_M1] = { GPIOB, GPIO_PIN_14, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0, EXTI_NULL, 0, 0 },

};

void bsp_gpio_init(void)
{


    // 使能对应 GPIO 端口的时钟
    for(int i = 0; i < BSP_SIG_CH_MAX; i++)
    {
        if(bsp_sig_map[i].port == GPIOA)
        {
            /* 对 GPIOA 的引脚进行初始化 */
            __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        if(bsp_sig_map[i].port == GPIOB)
        {
            /* 对 GPIOB 的引脚进行初始化 */
            __HAL_RCC_GPIOB_CLK_ENABLE();
        }
        if(bsp_sig_map[i].port == GPIOC)
        {
            /* 对 GPIOC 的引脚进行初始化 */
            __HAL_RCC_GPIOC_CLK_ENABLE();
        }
    }

    for(int i = 0; i < BSP_SIG_CH_MAX; i++)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = bsp_sig_map[i].pin;
        GPIO_InitStruct.Mode = bsp_sig_map[i].mode;
        GPIO_InitStruct.Pull = bsp_sig_map[i].pull;
        GPIO_InitStruct.Speed = bsp_sig_map[i].speed;
        GPIO_InitStruct.Alternate = bsp_sig_map[i].alternate;
        HAL_GPIO_Init(bsp_sig_map[i].port, &GPIO_InitStruct);
    }

    for(int i = 0; i < BSP_SIG_CH_MAX; i++)
    {
        if(bsp_sig_map[i].exti_irq != EXTI_NULL)
        {
            HAL_NVIC_SetPriority(bsp_sig_map[i].exti_irq, bsp_sig_map[i].pre_priority, bsp_sig_map[i].sub_priority);
            HAL_NVIC_EnableIRQ(bsp_sig_map[i].exti_irq);
        }
    }
}

void bsp_gpio_write(bsp_sig_ch_t ch, bsp_gpio_level_t level)
{
    if (ch >= BSP_SIG_CH_MAX)
        return;

    GPIO_TypeDef *port = bsp_sig_map[ch].port;
    uint16_t pin = bsp_sig_map[ch].pin;

    HAL_GPIO_WritePin(port, pin, (level == BSP_GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bsp_gpio_level_t bsp_gpio_read(bsp_sig_ch_t ch)
{
    if (ch >= BSP_SIG_CH_MAX)
        return BSP_GPIO_LOW;

    GPIO_TypeDef *port = bsp_sig_map[ch].port;
    uint16_t pin = bsp_sig_map[ch].pin;

    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? BSP_GPIO_HIGH : BSP_GPIO_LOW;
}
