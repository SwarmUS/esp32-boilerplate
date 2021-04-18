#include "NetworkConfig.h"

#include "DefaultNetworkConfig.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <cstring>

wifi_mode_t NetworkConfig::getMode() { return WIFI_MODE_STA; }

wifi_config_t* NetworkConfig::getDefaultNetworkConfig() {
    static wifi_config_t s_wifiConfig;
    // +1 to have the \0 for the string
    strlcpy((char*)(s_wifiConfig.sta.ssid), DEFAULT_SSID, strlen(DEFAULT_SSID) + 1);
    strlcpy((char*)(s_wifiConfig.sta.password), DEFAULT_PASSWORD, strlen(DEFAULT_PASSWORD) + 1);

    s_wifiConfig.sta.scan_method = WIFI_FAST_SCAN;
    s_wifiConfig.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    s_wifiConfig.sta.threshold.rssi = INT8_MIN;
    s_wifiConfig.sta.threshold.authmode = DEFAULT_AUTH_MODE;
    return &s_wifiConfig;
}

esp_interface_t NetworkConfig::getInterface() { return ESP_IF_WIFI_STA; }

uint16_t NetworkConfig::getCommunicationPort() { return (uint16_t)DEFAULT_UNICAST_PORT; }

uint16_t NetworkConfig::getBroadcastInputPort() { return (uint16_t)DEFAULT_BROADCAST_INPUT_PORT;}

uint16_t NetworkConfig::getBroadcastOutputPort() {return (uint16_t)DEFAULT_BROADCAST_OUTPUT_PORT;}
