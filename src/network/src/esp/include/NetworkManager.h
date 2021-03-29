#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "INetworkManager.h"
#include "TCPClient.h"
#include "TCPServer.h"
#include "logger/ILogger.h"
#include <BaseTask.h>
#include <Task.h>

#include "esp_wifi.h"

/**
 * @brief The network manager class. Handles the connection to the network
 */
class NetworkManager : public INetworkManager {
  public:
    NetworkManager(ILogger& logger, INetworkInputStream& server);
    ~NetworkManager() = default;

    void start() override;
    NetworkStatus getNetworkStatus() override;
    bool getSelfIP(char* buffer, size_t maxLength) override;

    // TODO: implement this function
    bool getIPFromRobotID(uint32_t robotID, char* buffer, size_t maxLength) override {
        m_logger.log(LogLevel::Error, "No list initialised");
        return false;
    }

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
