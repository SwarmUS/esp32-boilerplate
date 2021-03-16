#include "NetworkContainer.h"
#include "DummyNetworkInputStream.h"
#include "DummyNetworkManager.h"
#include "DummyNetworkOutputStream.h"

INetworkManager& NetworkContainer::getNetworkManager() {
    static DummyNetworkManager s_networkManager;

    return s_networkManager;
}

INetworkInputStream& NetworkContainer::getNetworkInputStream() {
    static DummyNetworkInputStream s_inputStream;

    return s_inputStream;
}

INetworkOutputStream& NetworkContainer::getNetworkOutputStream() {
    static DummyNetworkOutputStream s_outputStream;

    return s_outputStream;
}