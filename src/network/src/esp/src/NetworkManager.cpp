#include "NetworkManager.h"
#include "NetworkConfig.h"
#include "NetworkContainer.h"
#include "SocketFactory.h"

constexpr uint8_t g_LOOP_RATE = 100;

static void networkExecuteTask(void* context) {
    if (context != nullptr) {
        while (true) {
            static_cast<NetworkManager*>(context)->execute();
            Task::delay(g_LOOP_RATE);
        }
    }
}

NetworkManager::NetworkManager(ILogger& logger,
                               INetworkInputStream& server,
                               IHashMap<uint16_t, uint32_t>& hashMap) :
    AbstractNetworkManager(logger, hashMap),

    m_networkExecuteTask("network_manager", tskIDLE_PRIORITY + 1, networkExecuteTask, this),
    m_server(server) {

    // Initialise to 0.0.0.0
    m_ipAddress.addr = 0;
    m_state = NetworkManagerState::INIT;
}

bool NetworkManager::initNetworkInterface() {
    // Sadly the event_handler_arg copied in the event handler, therefore we cannot use it to pass
    // pointer to this instance
    ESP_ERROR_CHECK(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandler, nullptr));
    ESP_ERROR_CHECK(
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandler, nullptr));

    ESP_ERROR_CHECK(esp_netif_init());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Initialize default station as network interface instance. Will change in the future when
    // enabling mesh capabilities
    if (NetworkConfig::getMode() == WIFI_MODE_AP) {
        m_networkInterfaceHandle = esp_netif_create_default_wifi_ap();
        ESP_ERROR_CHECK(esp_netif_dhcps_start(m_networkInterfaceHandle));
    } else {
        m_networkInterfaceHandle = esp_netif_create_default_wifi_sta();
    }
    return m_networkInterfaceHandle != nullptr;
}

void NetworkManager::eventHandler(void* context,
                                  esp_event_base_t eventBase,
                                  int32_t eventId,
                                  void* eventData) {
    (void)context;
    auto* manager = (NetworkManager*)(&NetworkContainer::getNetworkManager());

    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START) {
        manager->m_logger.log(LogLevel::Info, "Started wifi, attempting to connect...");
        esp_wifi_connect();
    } else if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED) {
        manager->m_logger.log(LogLevel::Warn, "Network error, attempting to reconnect");
        manager->m_state = NetworkManagerState::DISCONNECTED;
    } else if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_AP_START) {
        manager->m_logger.log(LogLevel::Info, "Started wifi acces point!");
        esp_netif_ip_info_t ipInfo;
        ESP_ERROR_CHECK(esp_netif_get_ip_info(manager->m_networkInterfaceHandle, &ipInfo));
        manager->m_ipAddress = ipInfo.ip;
        manager->m_logger.log(LogLevel::Info, "Network manager got ip:" IPSTR,
                              IP2STR(&manager->m_ipAddress));
        manager->m_state = NetworkManagerState::CONNECTED;
    }

    else if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        auto* event = (ip_event_got_ip_t*)eventData;
        manager->m_state = NetworkManagerState::CONNECTED;
        manager->m_logger.log(LogLevel::Info, "Network manager got ip:" IPSTR,
                              IP2STR(&event->ip_info.ip));
        manager->m_ipAddress = event->ip_info.ip;
    }
}

void NetworkManager::start() {
    initNetworkInterface();
    m_networkExecuteTask.start();
}

NetworkStatus NetworkManager::getNetworkStatus() const {
    switch (m_state) {
    case NetworkManagerState::LOOKING_FOR_NETWORK:
        return NetworkStatus::Connecting;
    case NetworkManagerState::CONNECTED:
    case NetworkManagerState::RUNNING:
        return NetworkStatus::Connected;

    case NetworkManagerState::INIT:
    case NetworkManagerState::DISCONNECTED:
    default:
        return NetworkStatus::NotConnected;
    }
}

uint32_t NetworkManager::getSelfIP() const { return m_ipAddress.addr; }

void NetworkManager::execute() {
    switch (m_state) {

    case NetworkManagerState::INIT:
        m_logger.log(LogLevel::Info, "Connecting to network...");
        ESP_ERROR_CHECK(esp_wifi_set_mode(NetworkConfig::getMode()));
        ESP_ERROR_CHECK(esp_wifi_set_config(NetworkConfig::getInterface(),
                                            NetworkConfig::getDefaultNetworkConfig()));
        ESP_ERROR_CHECK(esp_wifi_start());
        m_state = NetworkManagerState::LOOKING_FOR_NETWORK;
        break;
    case NetworkManagerState::LOOKING_FOR_NETWORK:
        // Idle state
        break;
    case NetworkManagerState::CONNECTED:
        m_logger.log(LogLevel::Info, "Connected to network!");

        if (!m_server.start()) {
            m_logger.log(LogLevel::Info, "Failed to start TCP server socket");
        } else if (!NetworkContainer::getNetworkBroadcast().start()) {
            m_logger.log(LogLevel::Info, "Failed to start UDP socket");
        } else {
            m_logger.log(LogLevel::Error, "TCP and UDP socket started!");
        }
        m_state = NetworkManagerState::RUNNING;
        break;
    case NetworkManagerState::RUNNING:
        // Idle state.
        break;
    case NetworkManagerState::DISCONNECTED:
        m_logger.log(LogLevel::Error, "Handling the network error");
        m_server.stop();

        // Behavior to validate
        ESP_ERROR_CHECK(esp_wifi_stop());

        m_state = NetworkManagerState::INIT;
        break;
    default:
        m_logger.log(LogLevel::Warn, "Reached unintended case within network manager");
        m_state = NetworkManagerState::INIT; // Probably the best way to handle it
        break;
    }
}