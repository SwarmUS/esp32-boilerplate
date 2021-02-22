#ifndef __PIN_MAP_H__
#define __PIN_MAP_H__

#include "driver/spi_common.h"
#include "hal/gpio_types.h"

/** IMPORTANT:
 * There was an error during the rooting of the HiveSight. GPIO 34 and GPIO 35 were selected as
 * the user GPIOs on the WROOM module, but they are only workin in input mode. To fix this, the chip
 * select was changed with the user 0 for both the WROOM and the SOC modules. This was done so that
 * only the code on the ESP needed to be recompiled, since it is easier. The flag below can be
 * changed to set the proper configuration. Make sure not to commit any change to it: by default,
 * it should always be set to SPI_CONFIG_WROOM.
 */

// Change to SPI_CONFIG_SOC to test communication with SOC
#define SPI_CONFIG_WROOM

// SPI Link to STM
#define STM_SPI VSPI_HOST
#define STM_MOSI_D GPIO_NUM_23
#define STM_MISO_Q GPIO_NUM_19
#define STM_CLK GPIO_NUM_18
// Different CS for WROOM and SOC
#ifdef SPI_CONFIG_WROOM
#define STM_CS GPIO_NUM_34
#elif SPI_CONFIG_SOC
#define STM_CS GPIO_NUM_17
#endif


// DMA_CHANNELS
#define DMA_CHANNEL_SPI 1 // No dma

// User GPIOs
#define STM_USER_0 GPIO_NUM_5
#define STM_USER_0_PIN_MASK GPIO_SEL_5
// Different user pins for WROOM and SOC
#ifdef SPI_CONFIG_WROOM
#define STM_USER_1 GPIO_NUM_35
#define STM_USER_1_PIN_MASK GPIO_SEL_35
#elif SPI_CONFIG_SOC
#define STM_USER_1 GPIO_NUM_27
#define STM_USER_1_PIN_MASK GPIO_SEL_27
#endif

// Output LED
#define USER_LED GPIO_NUM_16

#endif // __PIN_MAP_H__
