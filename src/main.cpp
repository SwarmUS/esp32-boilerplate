#include <stdio.h>

#include "bsp/Factory.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

#ifdef __cplusplus
extern "C" {
#endif

void dummyTask(void* param) {
    (void)param;
    printf("Hello world!\n");

    /* Print chip information */
    std::shared_ptr<IBSP> bsp = BspFactory::getBSP();
    ChipInfo info = bsp->getChipInfo();

    while (true) {
        printf("System has %d cores\n\r", info.m_cores);
        if (info.m_osType == ChipInfo::ESP) {
            printf("System is running on target\n\r");
        } else {
            printf("System is running locally\n\r");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    std::shared_ptr<IBSP> bsp = BspFactory::getBSP();
    bsp->initChip();

    xTaskCreate(dummyTask, "dumb", configMINIMAL_STACK_SIZE * 4, NULL, 0 + 2, NULL);
}

#ifdef __cplusplus
}
#endif
