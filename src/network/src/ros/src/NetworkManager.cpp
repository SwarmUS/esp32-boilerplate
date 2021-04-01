#include "NetworkManager.h"
#include <ros/ros.h>

NetworkManager::NetworkManager(ILogger& logger,
                               IHashMap<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP>& hashMap) :
    m_logger(logger), m_hashMap(hashMap) {}

NetworkStatus NetworkManager::getNetworkStatus() { return NetworkStatus::Connected; }

uint32_t NetworkManager::getSelfIP() {
    ros::NodeHandle nodeHandle("~");
    return nodeHandle.param("tcp_listen_port", 54321);
}

std::optional<uint32_t> NetworkManager::getIPFromAgentID(uint16_t agentID) const {

    auto obj = m_hashMap.at(agentID);
    if (obj.has_value()) {
        return obj.value().get();
    }
    return {};
}

bool NetworkManager::registerAgent(uint16_t agentID, uint32_t port) {
    if (m_hashMap.at(agentID).has_value() &&
        m_hashMap.upsert(std::pair<uint16_t, uint16_t>(agentID, port))) {
        m_logger.log(LogLevel::Info, "Updated port of agent %d with value %d", agentID, port);
        return true;
    }
    if (m_hashMap.insert(std::pair<uint16_t, uint16_t>(agentID, port))) {
        m_logger.log(LogLevel::Info, "Registered new agent %d with value %d", agentID, port);
        return true;
    }
    return false;
}
