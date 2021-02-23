#include "System.h"
#include "ros/ros.h"
#include <Task.h>
#include <csetjmp>


jmp_buf g_buf;

int main(int argc, char** argv) {

    ros::init(argc, argv, "hive_connect");
    std::shared_ptr<ros::NodeHandle> nodeHandle =
        std::make_shared<ros::NodeHandle>(*(new ros::NodeHandle("~")));
    setjmp(g_buf);
    app_main();
    Task::startScheduler(); // This is also done in the hidden esp main
}