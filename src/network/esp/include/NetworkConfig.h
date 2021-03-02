#ifndef HIVE_CONNECT_NETWORKCONFIG_H
#define HIVE_CONNECT_NETWORKCONFIG_H

#include "esp_event.h"

using network_event_handler_t = void*(void*, esp_event_base_t, int32_t, void*);

void networkInit();

#endif // HIVE_CONNECT_NETWORKCONFIG_H
