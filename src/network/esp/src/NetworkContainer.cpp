#include "NetworkContainer.h"
#include "SocketFactory.h"

constexpr uint16_t g_SERVER_PORT = 8000;

NetworkManager& NetworkContainer::getNetworkManager() {

    static NetworkManager s_networkManager(LoggerContainer::getLogger());

    return s_networkManager;
}
