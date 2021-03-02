#ifndef __NETWORKCONTAINER_H__
#define __NETWORKCONTAINER_H__

#include "NetworkManager.h"
#include "logger/LoggerContainer.h"

namespace NetworkContainer {

NetworkManager& getNetworkManager() {
    static NetworkManager s_networkManager(LoggerContainer::getLogger());

    return s_networkManager;
}
} // namespace NetworkContainer

#endif // __NETWORKCONTAINER_H__
