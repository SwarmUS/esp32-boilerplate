#include "NetworkContainer.h"
#include "NetworkManager.h"
#include "SocketFactory.h"

INetworkManager& NetworkContainer::getNetworkManager() {
    static NetworkManager s_networkManager(LoggerContainer::getLogger(), getDeserializer());

    return s_networkManager;
}

INetworkDeserializer& NetworkContainer::getDeserializer() {
    static TCPServer s_server(LoggerContainer::getLogger());

    return s_server;
}

INetworkSerializer& NetworkContainer::getSerializer() {
    static TCPClient s_client(LoggerContainer::getLogger());

    return s_client;
}
