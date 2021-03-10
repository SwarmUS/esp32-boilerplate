#include "NetworkContainer.h"
#include "SocketFactory.h"

NetworkManager& NetworkContainer::getNetworkManager() {
    static NetworkManager s_networkManager(LoggerContainer::getLogger(), getTCPServer(), getTCPClient());

    return s_networkManager;
}

TCPServer & NetworkContainer::getTCPServer() {
    static TCPServer s_server(LoggerContainer::getLogger());

    return s_server;
}


TCPClient & NetworkContainer::getTCPClient() {
    static TCPClient s_client(LoggerContainer::getLogger());

    return s_client;
}
