#include "NetworkContainer.h"
#include "DummyNetworkManager.h"
#include "DummyNetworkSerializer.h"

INetworkManager& NetworkContainer::getNetworkManager() {
    static DummyNetworkManager s_networkManager;

    return s_networkManager;
}

INetworkSerializer& NetworkContainer::getSerializer() {
    static DummyNetworkSerializer s_networkSerializer;

    return s_networkSerializer;
}