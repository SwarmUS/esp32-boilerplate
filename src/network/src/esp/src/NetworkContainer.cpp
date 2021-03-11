#include "NetworkContainer.h"
#include "SocketFactory.h"
#include "NetworkManager.h"

INetworkManager& NetworkContainer::getNetworkManager() {
    static NetworkManager s_networkManager(LoggerContainer::getLogger(), getTCPServer());

    return s_networkManager;
}

INetworkDeserializer& NetworkContainer::getTCPServer() {
    static TCPServer s_server(LoggerContainer::getLogger());

    return s_server;
}

INetworkSerializer& NetworkContainer::getTCPClient() {
    static TCPClient s_client(LoggerContainer::getLogger());

    return s_client;
}
