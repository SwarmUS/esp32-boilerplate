#ifndef HIVE_CONNECT_IABSTRACTNETWORKMANAGER_H
#define HIVE_CONNECT_IABSTRACTNETWORKMANAGER_H

#include "cpp-common/HashMap.h"
#include "logger/ILogger.h"
#include <optional>
#include <string>

enum class NetworkStatus { NotConnected = 0, Connecting, Connected };
static constexpr uint16_t gs_MAX_AGENT_IN_MAP = 48;

class IAbstractNetworkManager {
  public:
    IAbstractNetworkManager(ILogger& logger,
                            IHashMap<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP>& hashMap) :
        m_logger(logger), m_hashMap(hashMap) {}
    virtual ~IAbstractNetworkManager() = default;

    /**
     * @brief Starts the network manager and attempts connection
     */
    virtual void start() = 0;

    /**
     * @brief Gets the status of the network
     */
    virtual NetworkStatus getNetworkStatus() = 0;

    /**
     * @brief Returns ID of the module in the network as a char array. In the wifi network, this
     * will be a tcp port
     * @return True if operation successful, false otherwise
     */
    virtual uint32_t getSelfIP() = 0;

    /**
     * @brief Returns the IP from a agent ID
     * @param [in] agentID The target agent ID
     * @return True the IP/port if the agent was known or an empty optional otherwise
     * otherwise.
     */
    virtual std::optional<uint32_t> getIPFromAgentID(uint16_t agentID) const {
        uint32_t ip;
        if (m_hashMap.get(agentID, ip)) {
            return ip;
        }
        return {};
    }

    /**
     * @brief Add a agent to the the list of known agents
     * @param [in] agentID the id to the agent to register
     * @param [in] ip The ip/port of the agent to register
     * @return true if the agent was added or already present, false if the agent could not be
     * registered
     */
    virtual bool registerAgent(uint16_t agentID, uint32_t ip) {
        uint32_t val;
        if (m_hashMap.get(agentID, val) &&
            m_hashMap.upsert(std::pair<uint16_t, uint16_t>(agentID, ip))) {
            m_logger.log(LogLevel::Info, "Updated port of agent %d with value %d", agentID, ip);
            return true;
        }
        if (m_hashMap.insert(std::pair<uint16_t, uint16_t>(agentID, ip))) {
            m_logger.log(LogLevel::Info, "Registered new agent %d with value %d", agentID, ip);
            return true;
        }
        return false;
    };

  protected:
    ILogger& m_logger;
    IHashMap<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP>& m_hashMap;
};

#endif // HIVE_CONNECT_IABSTRACTNETWORKMANAGER_H
