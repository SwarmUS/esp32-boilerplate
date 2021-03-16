#include "NetworkContainer.h"
#include "NetworkManager.h"
#include "SocketFactory.h"

INetworkManager& NetworkContainer::getNetworkManager() {
    static NetworkManager s_networkManager(LoggerContainer::getLogger(), getNetworkInputStream());

    return s_networkManager;
}

INetworkInputStream& NetworkContainer::getNetworkInputStream() {
    static TCPServer s_server(LoggerContainer::getLogger());

    return s_server;
}

INetworkOutputStream& NetworkContainer::getNetworkOutputStream() {
    static TCPClient s_client(LoggerContainer::getLogger());

    return s_client;
}
