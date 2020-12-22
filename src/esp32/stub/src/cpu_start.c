#include "esp_system.h"
#include <setjmp.h>
#include <stdbool.h>

extern void app_main(void);
jmp_buf buf;

int main(void) {
    setjmp(buf);
    app_main();
}