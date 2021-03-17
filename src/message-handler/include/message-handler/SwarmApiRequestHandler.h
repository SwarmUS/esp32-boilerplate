#ifndef HIVE_CONNECT_SWARMAPIREQUESTHANDLER_H
#define HIVE_CONNECT_SWARMAPIREQUESTHANDLER_H

#include "ISwarmApiRequestHandler.h"
#include "bsp/IBSP.h"
#include "logger/ILogger.h"

class SwarmApiRequestHandler : public ISwarmApiRequestHandler {
  public:
    SwarmApiRequestHandler(IBSP& bsp, ILogger& logger);

    ~SwarmApiRequestHandler() override = default;

    SwarmApiResponseDTO handleRequest(const SwarmApiRequestDTO &request) override;

  private:
    ILogger& m_logger;
    IBSP& m_bsp;

};

#endif // HIVE_CONNECT_SWARMAPIREQUESTHANDLER_H
