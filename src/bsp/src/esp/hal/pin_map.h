#ifndef __PIN_MAP_H__
#define __PIN_MAP_H__

#include "driver/spi_common.h"
#include "hal/gpio_types.h"

// SPI Link to STM
#define STM_SPI VSPI_HOST
#define STM_MOSI_D GPIO_NUM_23
#define STM_MISO_Q GPIO_NUM_19
#define STM_CLK GPIO_NUM_18
#define STM_CS GPIO_NUM_5

// DMA_CHANNELS
#define DMA_CHANNEL_SPI 1

// User GPIOs
#define STM_USER_0 GPIO_NUM_17
#define STM_USER_0_PIN_MASK GPIO_SEL_17
#define STM_USER_1 GPIO_NUM_27
#define STM_USER_1_PIN_MASK GPIO_SEL_27

// Output LED
#define USER_LED GPIO_NUM_16

#endif // __PIN_MAP_H__
