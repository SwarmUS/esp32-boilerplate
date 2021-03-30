#include "CommunicationBroker.h"

CommunicationBroker::CommunicationBroker() {
    ROS_INFO("Communication Broker initialization");
    for (std::uint16_t& robotID : getRobotList()) {
        std::string publishingTopic = "CommunicationBroker/" + std::to_string(robotID);
        std::string subscribingTopic = "/" + std::to_string(robotID) + "/broadcast";

        ros::Publisher pub = m_nodeHandle.advertise<std_msgs::String>(publishingTopic, 1000);
        ros::Subscriber sub = m_nodeHandle.subscribe(
            subscribingTopic, 1000, &CommunicationBroker::communicationCallback, this);

        m_publishersMap.emplace(robotID, pub);
        m_subscribersList.push_back(sub);
    }
}

std::vector<uint16_t> CommunicationBroker::getRobotList() { return {}; }

void CommunicationBroker::communicationCallback(const hive_connect::Broadcast& msg) {
    for (auto& pair : m_publishersMap) {
        if (pair.first != msg.source_robot) {
            pair.second.publish(msg);
        }
    }
}
