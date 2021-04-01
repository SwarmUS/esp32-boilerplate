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
                getIPFromAgentID,
                (uint16_t agentID, char* buffer, size_t maxLength),
                (const override));
    MOCK_METHOD(bool, registerAgent, (uint16_t agentID, const char* ip), (override));
};

#endif // HIVE_CONNECT_NETWORKMANAGERMOCK_H
