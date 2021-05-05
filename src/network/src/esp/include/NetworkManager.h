#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "../../common/include/AbstractNetworkManager.h"
#include "TCPClient.h"
#include "TCPServer.h"
#include "esp_wifi.h"
#include "logger/ILogger.h"
#include <BaseTask.h>
#include <Task.h>

/**
 * @brief The network manager class. Handles the connection to the network
 */
class NetworkManager : public AbstractNetworkManager {
  public:
    NetworkManager(ILogger& logger,
                   INetworkInputStream& server,
                   IHashMap<uint16_t, uint32_t>& hashMap);
    ~NetworkManager() = default;

    void start() override;
    NetworkStatus getNetworkStatus() const override;
    uint32_t getSelfIP() const override;

    /**
     * @brief Execution loop, called internally
     */
    void execute();

  private:
    INetworkInputStream& m_server;
    BaseTask<configMINIMAL_STACK_SIZE * 4> m_networkExecuteTask;
    esp_ip4_addr_t m_ipAddress;
    esp_netif_obj* m_networkInterfaceHandle;
    enum class NetworkManagerState {
        INIT = 0,
        LOOKING_FOR_NETWORK,
        CONNECTED,
        RUNNING,
        DISCONNECTED
    } m_state;

    static void eventHandler(void* context,
                             esp_event_base_t eventBase,
                             int32_t eventId,
                             void* eventData);
    bool initNetworkInterface();
};

#endif // HIVE_CONNECT_NETWORKMANAGER_H
