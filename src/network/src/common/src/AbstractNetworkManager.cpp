#include "AbstractNetworkManager.h"

std::optional<uint32_t> AbstractNetworkManager::getIPFromAgentID(uint16_t agentID) const {
    auto agent = m_hashMap.at(agentID);
    if (agent.has_value()) {
        return agent.value().get();
    }
    return {};
}

bool AbstractNetworkManager::registerAgent(uint16_t agentID, uint32_t ip) {
    if (m_hashMap.at(agentID).has_value() &&
        m_hashMap.upsert(std::pair<uint16_t, uint16_t>(agentID, ip))) {
        m_logger.log(LogLevel::Info, "Updated port of agent %d with value %d", agentID, ip);
        return true;
    }
    if (m_hashMap.insert(std::pair<uint16_t, uint16_t>(agentID, ip))) {
        m_logger.log(LogLevel::Info, "Registered new agent %d with value %d", agentID, ip);
        return true;
    }
    return false;
}