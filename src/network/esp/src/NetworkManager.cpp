#include "NetworkManager.h"
#include "NetworkConfig.h"

static void task(void* context) {
    if (context != nullptr) {
        while (true) {
            static_cast<NetworkManager*>(context)->execute();
        }
    }
}

NetworkManager::NetworkManager(ILogger& logger) :
    m_logger(logger), m_driverTask("stm_spi_driver", tskIDLE_PRIORITY + 1, task, this) {
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandler, this));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandler, this));
    // Initialise to 0.0.0.0
    m_ipAddress.u_addr.ip4.addr = 0;
}

void NetworkManager::initNetworkInterface() {
    ESP_ERROR_CHECK(esp_netif_init());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Initialize default station as network interface instance. Will change in the future when
    // enabling mesh capabilities
    auto* networkInterface = esp_netif_create_default_wifi_sta();
    assert(networkInterface);
}

void NetworkManager::eventHandler(void* context,
                                  esp_event_base_t eventBase,
                                  int32_t eventId,
                                  void* eventData) {
    auto* manager = static_cast<NetworkManager*>(context);

    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED) {
        manager->m_logger.log(LogLevel::Error, "Network error, attempting to reconnect");
        manager->m_state = NetworkState::CONNECTED;
    } else if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        auto* event = (ip_event_got_ip_t*)eventData;
        manager->m_logger.log(LogLevel::Info, "Network manager got ip:" IPSTR,
                              IP2STR(&event->ip_info.ip));
        manager->m_ipAddress.u_addr.ip4 = event->ip_info.ip;
        manager->m_state = NetworkState::CONNECTED;
    }
}

void NetworkManager::start() {
    initNetworkInterface();
    m_driverTask.start();
}

void NetworkManager::execute() {
    switch (m_state) {

    case NetworkState::CONNECTING:
        ESP_ERROR_CHECK(esp_wifi_set_mode(NetworkConfig::getMode()));
        ESP_ERROR_CHECK(esp_wifi_set_config(NetworkConfig::getInterface(),
                                            NetworkConfig::getDefaultNetworkConfig()));
        ESP_ERROR_CHECK(esp_wifi_start());
        break;
    case NetworkState::CONNECTED:
        // TODO: start tcp server to monitor messages and such
        break;
    }
    Task::delay(100);
}

esp_ip4_addr_t NetworkManager::getIP() const { return m_ipAddress.u_addr.ip4; }