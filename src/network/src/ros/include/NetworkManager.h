#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "INetworkManager.h"
#include "cpp-common/IHashMap.h"
#include "logger/ILogger.h"

static constexpr uint16_t gs_MAX_AGENT_IN_MAP = 48;

class NetworkManager : public INetworkManager {
  public:
    NetworkManager(ILogger& logger, IHashMap<uint16_t, uint16_t, gs_MAX_AGENT_IN_MAP>& hashMap);
    ~NetworkManager() = default;
    void start() override {}

    /**
     * @brief Gets the status of the network
     */
    NetworkStatus getNetworkStatus() override;

    /**
     * @brief Returns ID of the module in the network as a char array. In the wifi network, this
     * will be an IP address and on ROS probably a topic.
     * @return The networking ID for the module
     */
    bool getSelfIP(char* buffer, size_t maxLength) override;

    bool getIPFromAgentID(uint16_t agentID, char* buffer, size_t maxLength) const override;

    bool registerAgent(uint16_t agentID, const char* ip) override;

  private:
    ILogger& m_logger;
    IHashMap<uint16_t, uint16_t, gs_MAX_AGENT_IN_MAP>& m_hashMap;
};

#endif // HIVE_CONNECT_NETWORKMANAGER_H
