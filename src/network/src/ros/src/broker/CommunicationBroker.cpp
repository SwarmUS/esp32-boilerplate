#include "CommunicationBroker.h"
#include "config/TopicDefines.h"

CommunicationBroker::CommunicationBroker() {
    ROS_INFO("Communication Broker initialization");
    for (std::uint16_t& robotID : getRobotList()) {
        std::string subscribingTopic = BROADCAST_OUTPUT_TOPIC + std::to_string(robotID);
        std::string publishingTopic = BROADCAST_INPUT_TOPIC + std::to_string(robotID);

        ros::Publisher pub = m_nodeHandle.advertise<hive_connect::Broadcast>(publishingTopic, 1000);
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
    if (!nh.getParam(robotListParamName, configRobotList)) {
        ROS_ERROR("No list was found");
        return {};
    }

    if (configRobotList.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("No robot found in broker config");
        return {};
    }

    for (int i = 0; i < configRobotList.size(); i++) {
        robotList.push_back((int)configRobotList[i]);
    }
    ROS_INFO("Subscribing to %zu agents", robotList.size());

    return robotList;
}

void CommunicationBroker::communicationCallback(const hive_connect::Broadcast& msg) {
    for (auto& [id, publisher] : m_publishersMap) {
        if (id != msg.source_robot) {
            ROS_INFO("Forwarding message from agent %d to agent %d", msg.source_robot, id);
            publisher.publish(msg);
        }
    }
}
