#ifndef HIVE_CONNECT_NETWORKAPIHANDLERMOCK_H
#define HIVE_CONNECT_NETWORKAPIHANDLERMOCK_H

#include "message-handler/INetworkAPIHandler.h"
#include "gmock/gmock.h"

class NetworkAPIHandlerMock : public INetworkAPIHandler {
  public:
    ~NetworkAPIHandlerMock() override = default;
    MOCK_METHOD((std::variant<ErrorNum, std::optional<NetworkApiDTO>>),
                handleApiCall,
                (const NetworkApiDTO& apiCall),
                (override));
};

#endif // HIVE_CONNECT_NETWORKAPIHANDLERMOCK_H
