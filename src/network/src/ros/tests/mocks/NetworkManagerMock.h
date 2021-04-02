#ifndef HIVE_CONNECT_NETWORKMANAGERMOCK_H
#define HIVE_CONNECT_NETWORKMANAGERMOCK_H

#include "AbstractNetworkManager.h"
#include <gmock/gmock.h>

class NetworkManagerMock : public AbstractNetworkManager {

  public:
    NetworkManagerMock() = default;
    MOCK_METHOD(void, start, (), (override));
    MOCK_METHOD(NetworkStatus, getNetworkStatus, (), (const, override));
    MOCK_METHOD(uint32_t, getSelfIP, (), (const, override));
};

#endif // HIVE_CONNECT_NETWORKMANAGERMOCK_H
