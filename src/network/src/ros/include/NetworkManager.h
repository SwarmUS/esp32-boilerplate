#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "IAbstractNetworkManager.h"
#include "cpp-common/IHashMap.h"
#include "logger/ILogger.h"

class NetworkManager : public IAbstractNetworkManager {
  public:
    NetworkManager(ILogger& logger, IHashMap<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP>& hashMap);
    ~NetworkManager() override = default;
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

  private:
};

#endif // HIVE_CONNECT_NETWORKMANAGER_H
