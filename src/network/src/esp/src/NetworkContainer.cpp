#include "NetworkContainer.h"
#include "NetworkBroadcast.h"
#include "NetworkManager.h"
#include "SocketFactory.h"
#include "cpp-common/HashMapStack.h"

constexpr uint16_t gs_MAX_AGENT_IN_MAP = 32;
INetworkManager& NetworkContainer::getNetworkManager() {
    static HashMapStack<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP> s_hashMap;
    static NetworkManager s_networkManager(LoggerContainer::getLogger(), getNetworkInputStream(),
                                           s_hashMap);

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

INetworkBroadcast& NetworkContainer::getNetworkBroadcast() {
    static NetworkBroadcast s_broadcast;
    return s_broadcast;
}
