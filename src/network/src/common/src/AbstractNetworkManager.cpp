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
        unsigned char bytes[4];
        bytes[0] = ip & 0xFF;
        bytes[1] = (ip >> 8) & 0xFF;
        bytes[2] = (ip >> 16) & 0xFF;
        bytes[3] = (ip >> 24) & 0xFF;
        m_logger.log(LogLevel::Info, "Registered agent %d with value %d.%d.%d.%d", agentID,
                     bytes[0], bytes[1], bytes[2], bytes[3]);
        return true;
    }
    return false;
}