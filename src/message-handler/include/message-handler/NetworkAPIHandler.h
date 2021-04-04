#ifndef HIVE_CONNECT_NETWORKAPIHANDLER_H
#define HIVE_CONNECT_NETWORKAPIHANDLER_H

#include "INetworkAPIHandler.h"
#include "INetworkManager.h"
#include "bsp/IBSP.h"
#include "logger/ILogger.h"

class NetworkAPIHandler : public INetworkAPIHandler {
  public:
    NetworkAPIHandler(IBSP& bsp, ILogger& logger, INetworkManager& networkManager);

    ~NetworkAPIHandler() override = default;

    std::variant<ErrorNum, std::optional<NetworkApiDTO>> handleApiCall(
        uint16_t sourceID, const NetworkApiDTO& apiCall) override;

  private:
    ILogger& m_logger;
    IBSP& m_bsp;
    INetworkManager& m_networkManager;
};

#endif // HIVE_CONNECT_NETWORKAPIHANDLER_H
