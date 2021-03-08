#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "TCPServer.h"
#include "logger/ILogger.h"
#include "memory"
#include <BaseTask.h>
#include <Task.h>
#include <optional>

#include "esp_wifi.h"

/**
 * @brief The network manager class. Handles the connnection to the network
 */
class NetworkManager {
  public:
    NetworkManager(ILogger& logger);
    ~NetworkManager() = default;

    /**
     * @brief Starts the network manager
     */
    void start();

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
    TCPServer m_server;
    BaseTask<configMINIMAL_STACK_SIZE * 4> m_networkExecuteTask;
    esp_ip_addr_t m_ipAddress;
    enum class NetworkState {
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
    void initNetworkInterface();
};

#endif // HIVE_CONNECT_NETWORKMANAGER_H
