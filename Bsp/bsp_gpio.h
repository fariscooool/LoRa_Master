#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include <stdbool.h>

#include "gpio.h"


typedef enum
{
    BSP_LORA_AUX,       /* PC0 */
    BSP_LORA_M0,        /* PB15 */
    BSP_LORA_M1,        /* PB14 */
    BSP_SIG_CH_MAX
} bsp_sig_ch_t;

typedef enum
{
    BSP_GPIO_LOW  = 0,
    BSP_GPIO_HIGH
} bsp_gpio_level_t;

void bsp_gpio_init(void);
void bsp_gpio_write(bsp_sig_ch_t ch, bsp_gpio_level_t level);
bsp_gpio_level_t bsp_gpio_read(bsp_sig_ch_t ch);


#endif /* BSP_GPIO_H */
