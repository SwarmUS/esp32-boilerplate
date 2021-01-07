#include "esp_system.h"
#include <setjmp.h>
#include <stdbool.h>
#include "ros/ros.h"

jmp_buf buf;

int main(int argc, char** argv) {

    ros::init(argc, argv, "hive_connect");
    setjmp(buf);
    app_main();
    vTaskStartScheduler(); // This is also done in the hidden esp main
}