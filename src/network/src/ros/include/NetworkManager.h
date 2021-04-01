#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "INetworkManager.h"
#include "cpp-common/IHashMap.h"
#include "logger/ILogger.h"

static constexpr uint16_t gs_MAX_AGENT_IN_MAP = 48;

class NetworkManager : public INetworkManager {
  public:
    NetworkManager(ILogger& logger, IHashMap<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP>& hashMap);
    ~NetworkManager() = default;
    void start() override {}

    /**
     * @brief Gets the status of the network
     */
    NetworkStatus getNetworkStatus() override;

    /**
     * @brief Returns ID the tcp port used for unicast communication
     * @return The networking ID for the module
     */
    uint32_t getSelfIP() override;

    std::optional<uint32_t> getIPFromAgentID(uint16_t agentID) const override;

    bool registerAgent(uint16_t agentID, uint32_t port) override;

  private:
    ILogger& m_logger;
    IHashMap<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP>& m_hashMap;
};

#endif // HIVE_CONNECT_NETWORKMANAGER_H
