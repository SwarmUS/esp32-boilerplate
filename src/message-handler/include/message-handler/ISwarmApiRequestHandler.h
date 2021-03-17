#ifndef HIVE_CONNECT_ISWARMAPIREQUESTHANDLER_H
#define HIVE_CONNECT_ISWARMAPIREQUESTHANDLER_H


#include <hivemind-host/SwarmApiRequestDTO.h>
#include <hivemind-host/MessageDTO.h>

/**
 *@brief Handles SwarmApi requests and sends the response to the appropriate target */
class ISwarmApiRequestHandler {
  public:
    virtual ~ISwarmApiRequestHandler() = default;

    /**
     *@brief handles a request and sends a response to the appropriate target
     *@param request the request to handle
     *@return the response to the request */
    virtual SwarmApiResponseDTO handleRequest(const SwarmApiRequestDTO& request) = 0;
};


#endif // HIVE_CONNECT_ISWARMAPIREQUESTHANDLER_H
