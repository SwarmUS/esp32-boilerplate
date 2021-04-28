#include "INetworkBroadcast.h"

class BroadcastMock : public INetworkBroadcast {
  public:
    BroadcastMock();
    ~BroadcastMock();

    MOCK_METHOD(bool, send, (const uint8_t* data, uint16_t length), (override));
    MOCK_METHOD(bool, receive, (uint8_t * data, uint16_t length), (override));
    MOCK_METHOD(bool, start, (), (override));
    MOCK_METHOD(bool, stop, (), (override));
    MOCK_METHOD(bool, isStarted, (), (const override));
};