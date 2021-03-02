#include "NetworkContainer.h"

NetworkManager& NetworkContainer::getNetworkManager() {
    static NetworkManager s_networkManager(LoggerContainer::getLogger());

    return s_networkManager;
}
