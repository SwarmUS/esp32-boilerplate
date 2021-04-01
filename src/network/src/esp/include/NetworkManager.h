#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "INetworkManager.h"
#include "TCPClient.h"
#include "TCPServer.h"
#include "cpp-common/HashMap.h"
#include "esp_wifi.h"
#include "logger/ILogger.h"
#include <BaseTask.h>
#include <Task.h>

constexpr uint16_t g_MaxSwarmAgents = 16;
/**
 * @brief The network manager class. Handles the connection to the network
 */
class NetworkManager : public INetworkManager {
  public:
    NetworkManager(ILogger& logger,
                   INetworkInputStream& server,
                   IHashMap<uint16_t, uint32_t, g_MaxSwarmAgents>& hashMap);
    ~NetworkManager() = default;

    void start() override;
    NetworkStatus getNetworkStatus() override;
    bool getSelfIP(char* buffer, size_t maxLength) override;
    bool getIPFromAgentID(uint16_t agentID, char* buffer, size_t maxLength) const override;
    bool registerAgent(uint16_t agentID, const char* ip) override;

    /**
     * @brief Execution loop, called internally
     */
    void execute();

    /**
     * @brief Returns the ip address of the module
     * @return IPv4 address
     */
    esp_ip4_addr_t getIP() const;

  private:
    ILogger& m_logger;
    INetworkInputStream& m_server;
    BaseTask<configMINIMAL_STACK_SIZE * 4> m_networkExecuteTask;
    esp_ip_addr_t m_ipAddress;
    IHashMap<uint16_t, uint32_t, g_MaxSwarmAgents>& m_hashMap;
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
