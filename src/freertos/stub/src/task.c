#include "freertos/task.h"
#include <unistd.h>

void vTaskDelay(const uint32_t xTicksToDelay) { sleep(xTicksToDelay); }