#include "CommunicationBroker.h"

CommunicationBroker::CommunicationBroker() {
    ROS_INFO("Communication Broker initialization");
    for (std::uint16_t& robotID : getRobotList()) {
        std::string subscribingTopic = "/Communication/broadcastOutput/" + std::to_string(robotID);
        std::string publishingTopic = "/Communication/broadcastInput/" + std::to_string(robotID);

        ros::Publisher pub = m_nodeHandle.advertise<std_msgs::String>(publishingTopic, 1000);
        ros::Subscriber sub = m_nodeHandle.subscribe(
            subscribingTopic, 1000, &CommunicationBroker::communicationCallback, this);

        m_publishersMap.emplace(robotID, pub);
        m_subscribersList.push_back(sub);
    }
}

std::vector<uint16_t> CommunicationBroker::getRobotList() {
    std::vector<uint16_t> robotList;
    ros::NodeHandle nh("~");

    XmlRpc::XmlRpcValue configRobotList;
    std::string robotListParamName =
        nh.param(std::string("configList"), std::string("/Broker/robots"));
    if (!nh.getParam("/Broker/robots", configRobotList)) {
        ROS_ERROR("No list was found");
        return {};
    }

    if (configRobotList.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("List not an array");
        return {};
    }

    for (int i = 0; i < configRobotList.size(); i++) {
        robotList.push_back((int)configRobotList[i]);
    }
    ROS_INFO("Subscribing to %zu agents", robotList.size());

    return robotList;
}

void CommunicationBroker::communicationCallback(const hive_connect::Broadcast& msg) {
    for (auto& pair : m_publishersMap) {
        if (pair.first != msg.source_robot) {
            pair.second.publish(msg);
        }
    }
}
