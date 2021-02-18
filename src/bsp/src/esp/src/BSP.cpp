#include "BSP.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include "freertos/task.h"

BSP::BSP() = default;
BSP::~BSP() = default;

void flashLEDTask(void* param) {
    (void) param;
    static bool state = false;
    while (true) {
        gpio_set_level(GPIO_NUM_16, state);
        state = !state;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void BSP::initChip() {
    gpio_config_t led = {
        .pin_bit_mask = GPIO_SEL_16,
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&led);
    xTaskCreate(flashLEDTask, "toggle_led", configMINIMAL_STACK_SIZE, nullptr, 3, nullptr);
}

ChipInfo BSP::getChipInfo() {
    esp_chip_info_t chipInfo;
    esp_chip_info(&chipInfo);

    return (ChipInfo){.m_cores = chipInfo.cores, .m_osType = ChipInfo::ESP};
}

