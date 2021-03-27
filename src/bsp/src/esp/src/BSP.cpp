#include "BSP.h"
#include "NetworkContainer.h"
#include "driver/spi_slave.h"
#include "esp_event.h"
#include "esp_system.h"
#include "hal/gpio_types.h"
#include "hal/pin_map.h"
#include "hal/spi_callbacks.h"
#include "nvs_flash.h"
#include <driver/gpio.h>

BSP::BSP() { m_UUID = 0; }

void BSP::initChip() {
    // Init event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Init non volatile flash storage
    ESP_ERROR_CHECK(nvs_flash_init());
    // Init spi slave
    initSPI();
}

ChipInfo BSP::getChipInfo() {
    esp_chip_info_t chipInfo;
    esp_chip_info(&chipInfo);

    return (ChipInfo){.m_cores = chipInfo.cores, .m_osType = ChipInfo::ESP};
}

void BSP::initSPI() {

    // Configuration for the spi bus
    spi_bus_config_t busConfig = {
        .mosi_io_num = STM_MOSI_D,
        .miso_io_num = STM_MISO_Q,
        .sclk_io_num = STM_CLK,
        // -1 for unused quad spi
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slaveConfig = {.spics_io_num = STM_CS,
                                                .flags = 0,
                                                .queue_size =
                                                    1, // Should have at most 2 (1 input, 1 output)
                                                .mode = 3,
                                                .post_setup_cb = NULL,
                                                .post_trans_cb = transactionCompleteCallback};

    assert(spi_slave_initialize(STM_SPI, &busConfig, &slaveConfig, DMA_CHANNEL_SPI) == ESP_OK);

    // Configuration for the output line
    gpio_config_t gpioConfig = {.pin_bit_mask = STM_USER_0_PIN_MASK,
                                .mode = GPIO_MODE_OUTPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = GPIO_INTR_DISABLE};

    assert(gpio_config(&gpioConfig) == ESP_OK);
    // Default to 0
    gpio_reset_pin(STM_USER_0);

    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(STM_MOSI_D, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(STM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(STM_CS, GPIO_PULLUP_ONLY);
}

uint16_t BSP::getHiveMindUUID() { return m_UUID; }

void BSP::setHiveMindUUID(uint16_t uuid) { m_UUID = uuid; }
