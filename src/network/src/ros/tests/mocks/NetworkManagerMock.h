#ifndef HIVE_CONNECT_NETWORKMANAGERMOCK_H
#define HIVE_CONNECT_NETWORKMANAGERMOCK_H

#include "INetworkManager.h"
#include <gmock/gmock.h>

class NetworkManagerMock : public INetworkManager {

  public:
    NetworkManagerMock() = default;
    MOCK_METHOD(void, start, (), (override));
    MOCK_METHOD(NetworkStatus, getNetworkStatus, (), (override));
    MOCK_METHOD(bool, getSelfIP, (char* buffer, size_t maxLength), (override));
    MOCK_METHOD(bool,
                getIPFromRobotID,
                (uint32_t robotID, char* buffer, size_t maxLength),
                (override));
};

#endif // HIVE_CONNECT_NETWORKMANAGERMOCK_H
