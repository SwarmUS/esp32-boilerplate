#include "NetworkManager.h"
#include <ros/ros.h>

NetworkManager::NetworkManager(ILogger& logger,
                               IHashMap<uint16_t, uint16_t, gs_MAX_AGENT_IN_MAP>& hashMap) :
    m_logger(logger), m_hashMap(hashMap) {}

NetworkStatus NetworkManager::getNetworkStatus() { return NetworkStatus::Connected; }

bool NetworkManager::getSelfIP(char* buffer, size_t maxLength) {
    if (buffer == nullptr) {
        m_logger.log(LogLevel::Error, "Invalid parameters passed to network manager");
        return false;
    }
    ros::NodeHandle nodeHandle("~");
    int port = nodeHandle.param("tcp_listen_port", 54321);
    if (snprintf(buffer, maxLength, "%d", port) > 0) {
        m_logger.log(LogLevel::Error, "Failed to write un buffer supplied");
        return false;
    }
    return true;
}

bool NetworkManager::getIPFromAgentID(uint16_t agentID, char* buffer, size_t maxLength) const {
    if (buffer == nullptr) {
        m_logger.log(LogLevel::Error, "Invalid parameters passed to network manager");
        return false;
    }
    auto obj = m_hashMap.at(agentID);
    if (obj.has_value() && snprintf(buffer, maxLength, "%d", obj.value().get()) > 0) {
        m_logger.log(LogLevel::Info, "Successfully obtained port from agent ID");
        return true;
    }
    return false;
}

bool NetworkManager::registerAgent(uint16_t agentID, const char* ip) {
    if (ip == nullptr) {
        m_logger.log(LogLevel::Error, "Invalid parameters passed to network manager");
        return false;
    }
    uint16_t agentPort = atoi(ip);
    if (m_hashMap.at(agentID).has_value() &&
        m_hashMap.upsert(std::pair<uint16_t, uint16_t>(agentID, agentPort))) {
        m_logger.log(LogLevel::Info, "Updated port of agent %d with value %d", agentID, agentPort);
        return true;
    }
    if (m_hashMap.insert(std::pair<uint16_t, uint16_t>(agentID, agentPort))) {
        m_logger.log(LogLevel::Info, "Registered new agent %d with value %d", agentID, agentPort);
        return true;
    }
    return false;
}
