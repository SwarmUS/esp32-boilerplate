#include "AbstractNetworkManager.h"

std::optional<uint32_t> AbstractNetworkManager::getIPFromAgentID(uint16_t agentID) const {
    auto agent = m_hashMap.at(agentID);
    if (agent) {
        return agent.value().get();
    }
    return {};
}

bool AbstractNetworkManager::registerAgent(uint16_t agentID, uint32_t ip) {
    if (m_hashMap.upsert(agentID, ip)) {
        m_logger.log(LogLevel::Debug, "Registered agent %d with value %d", agentID, ip);
        return true;
    }
    return false;
}