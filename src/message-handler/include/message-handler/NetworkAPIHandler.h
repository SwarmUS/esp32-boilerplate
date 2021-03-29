#ifndef HIVE_CONNECT_NETWORKAPIHANDLER_H
#define HIVE_CONNECT_NETWORKAPIHANDLER_H

#include "INetworkAPIHandler.h"
#include "bsp/IBSP.h"
#include "logger/ILogger.h"

class NetworkAPIHandler : public INetworkAPIHandler {
  public:
    NetworkAPIHandler(IBSP& bsp, ILogger& logger);

    ~NetworkAPIHandler() override = default;

    std::variant<ErrorNum, std::optional<NetworkApiDTO>> handleApiCall(
        const NetworkApiDTO& apiCall) override;

  private:
    ILogger& m_logger;
    IBSP& m_bsp;
};

#endif // HIVE_CONNECT_NETWORKAPIHANDLER_H
