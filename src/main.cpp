#include "bsp/Container.h"
#include "logger/Logger.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

#ifdef __cplusplus
extern "C" {
#endif

void dummyTask(void* param) {
    (void)param;

    /* Print chip information */
    IBSP* bsp = &BspContainer::getBSP();
    ChipInfo info = bsp->getChipInfo();
    Logger logger = Logger(LogLevel::Info, BspContainer::getUserInterface());
    logger.log(LogLevel::Info, "Hi!");
    while (true) {
        ISpiStm* spiStm = &BspContainer::getSpiStm();
        if (!spiStm->isBusy()) {
            char message[] = "Hello STM";
            spiStm->send((uint8_t*)message, sizeof(message));
        }
        else {
            //logger.log(LogLevel::Info, "Spi driver is busy...");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    IBSP* bsp = &BspContainer::getBSP();
    bsp->initChip();

    xTaskCreate(dummyTask, "dumb", configMINIMAL_STACK_SIZE * 4, NULL, 0 + 2, NULL);
}

#ifdef __cplusplus
}
#endif
