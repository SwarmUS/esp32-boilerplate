#include "CommunicationBroker.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "communication_broker");

    CommunicationBroker communicationBroker;
    ros::spin();

    return 0;
};