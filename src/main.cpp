#include <stdio.h>

#include "bsp/Container.h"
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
    IBSP* bsp = &BspContainer::getBSP();
    ChipInfo info = bsp->getChipInfo();

    while (true) {
        bsp->log(INFO, "System has %d cores", info.m_cores);
        if (info.m_osType == ChipInfo::ESP) {
            bsp->log(INFO, "System is running on target");
        } else {
            bsp->log(INFO, "System is running locally");
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
