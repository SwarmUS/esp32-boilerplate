#include "NetworkConfig.h"

#include "esp_event.h"
#include "esp_mesh.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "DefaultNetworkConfig.h"

static const char* TAG = "scan";

static void event_handler(void* arg,
                          esp_event_base_t event_base,
                          int32_t event_id,
                          void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void networkInit() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    // Initialize default station as network interface instance (esp-netif)
    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // Initialize and start WiFi
    /*wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = DEFAULT_SSID,
                .password = DEFAULT_PWD,
                .scan_method = DEFAULT_SCAN_METHOD,
                .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
                .threshold.rssi = DEFAULT_RSSI,
                .threshold.authmode = DEFAULT_AUTHMODE,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());*/
}

wifi_mode_t NetworkConfig::getMode() { return WIFI_MODE_STA; }

wifi_config_t* NetworkConfig::getDefaultNetworkConfig() {
    static wifi_config_t s_wifiConfig;
    std::memcpy(s_wifiConfig.sta.ssid, DEFAULT_SSID, strlen(DEFAULT_SSID));
    std::memcpy(s_wifiConfig.sta.password, DEFAULT_PASSWORD, strlen(DEFAULT_PASSWORD));
    s_wifiConfig.sta.scan_method = WIFI_FAST_SCAN;
    s_wifiConfig.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    s_wifiConfig.sta.threshold.rssi = -70;
    s_wifiConfig.sta.threshold.authmode = DEFAULT_AUTH_MODE;
    return &s_wifiConfig;
}

esp_interface_t NetworkConfig::getInterface() { return ESP_IF_WIFI_STA; }