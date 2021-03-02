#ifndef HIVE_CONNECT_NETWORKCONFIG_H
#define HIVE_CONNECT_NETWORKCONFIG_H

#include "esp_event.h"

namespace NetworkConfig {
wifi_mode_t getMode();
wifi_config_t* getDefaultNetworkConfig();
esp_interface_t getInterface();
} // namespace NetworkConfig

#endif // HIVE_CONNECT_NETWORKCONFIG_H
