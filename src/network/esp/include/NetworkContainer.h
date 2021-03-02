#ifndef __NETWORKCONTAINER_H__
#define __NETWORKCONTAINER_H__

#include <array>

enum class NetworkManagerStates {
    INIT_NET_INTERFACE = 0,
    JOIN_NETWORK,
    MONITOR_MESSAGES,
    LEAVE_NETWORK,

    STATE_COUNT
};

#endif // __NETWORKCONTAINER_H__
