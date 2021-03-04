#include "NetworkContainer.h"
#include "SocketFactory.h"

NetworkManager& NetworkContainer::getNetworkManager() {
    std::optional<TCPServer> server = SocketFactory::createTCPServer(8000);

    static NetworkManager s_networkManager(LoggerContainer::getLogger(), server.value());

    return s_networkManager;
}
