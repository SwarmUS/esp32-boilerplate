#ifndef HIVE_CONNECT_NETWORKAPIHANDLER_H
#define HIVE_CONNECT_NETWORKAPIHANDLER_H

#include "AbstractNetworkManager.h"
#include "INetworkAPIHandler.h"
#include "bsp/IBSP.h"
#include "logger/ILogger.h"

class NetworkAPIHandler : public INetworkAPIHandler {
  public:
    NetworkAPIHandler(IBSP& bsp, ILogger& logger, AbstractNetworkManager& networkManager);

    ~NetworkAPIHandler() override = default;

    std::variant<ErrorNum, std::optional<NetworkApiDTO>> handleApiCall(
        const MessageDTO& message, const NetworkApiDTO& apiCall) override;

  private:
    ILogger& m_logger;
    IBSP& m_bsp;
    AbstractNetworkManager& m_networkManager;
};

#endif // HIVE_CONNECT_NETWORKAPIHANDLER_H
