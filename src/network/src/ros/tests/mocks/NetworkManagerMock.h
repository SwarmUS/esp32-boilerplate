#ifndef HIVE_CONNECT_NETWORKMANAGERMOCK_H
#define HIVE_CONNECT_NETWORKMANAGERMOCK_H

#include "IAbstractNetworkManager.h"
#include <gmock/gmock.h>

class NetworkManagerMock : public IAbstractNetworkManager {

  public:
    NetworkManagerMock() = default;
    MOCK_METHOD(void, start, (), (override));
    MOCK_METHOD(NetworkStatus, getNetworkStatus, (), (const, override));
    MOCK_METHOD(uint32_t, getSelfIP, (), (const, override));
    MOCK_METHOD(std::optional<uint32_t>, getIPFromAgentID, (uint16_t agentID), (const override));
    MOCK_METHOD(bool, registerAgent, (uint16_t agentID, uint32_t ip), (override));
};

#endif // HIVE_CONNECT_NETWORKMANAGERMOCK_H
