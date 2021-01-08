#include "esp_system.h"
#include <setjmp.h>
#include <stdio.h>
#include <unistd.h>

extern jmp_buf g_buf;

void esp_restart(void) {
    printf("\n");
    sleep(1); // pause for dramatic effect
    longjmp(g_buf, 0);
}

void esp_chip_info(esp_chip_info_t* out_info) {
    out_info->cores = 8;
    out_info->features = (uint32_t)-1;
}