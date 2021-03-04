#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "logger/ILogger.h"
#include <BaseTask.h>
#include <Task.h>

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
    BaseTask<configMINIMAL_STACK_SIZE * 3> m_networkExecuteTask;
    esp_ip_addr_t m_ipAddress;
    enum class NetworkState { CONNECTING = 0, CONNECTED, DISCONNECTED } m_state;

    static void eventHandler(void* context,
                             esp_event_base_t eventBase,
                             int32_t eventId,
                             void* eventData);
    void initNetworkInterface();
};

#endif // HIVE_CONNECT_NETWORKMANAGER_H
