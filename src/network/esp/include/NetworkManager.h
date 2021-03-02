#ifndef HIVE_CONNECT_NETWORKMANAGER_H
#define HIVE_CONNECT_NETWORKMANAGER_H

#include "logger/ILogger.h"
#include <BaseTask.h>
#include <Task.h>

#include "esp_wifi.h"

class NetworkManager {
  public:
    NetworkManager(ILogger& logger);
    ~NetworkManager() = default;

    void initNetworkInterface();
    void start();
    void execute();
    esp_ip4_addr_t getIP() const;


  private:
    ILogger& m_logger;
    BaseTask<configMINIMAL_STACK_SIZE * 3> m_driverTask;
    esp_ip_addr_t m_ipAddress;

    static void eventHandler(void* context,
                             esp_event_base_t eventBase,
                             int32_t eventId,
                             void* eventData);

    enum NetworkState {
        START = 0,
        MONITOR,
        ERROR
    } m_state;
};

#endif // HIVE_CONNECT_NETWORKMANAGER_H
