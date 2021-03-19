#ifndef HIVE_CONNECT_INETWORKAPIHANDLER_H
#define HIVE_CONNECT_INETWORKAPIHANDLER_H

#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/NetworkApiDTO.h>
#include <optional>

/**
 *@brief Handles SwarmApi requests and sends the response to the appropriate target */
class INetworkAPIHandler {
  public:
    virtual ~INetworkAPIHandler() = default;

    /**
     *@brief handles the call and generate an optional response to the call
     *@param apiCall the call to handle
     *@return The response to dispatch. If no response need to be sent, will return an the response
     *will be a std::monostate. If the handling failed, it will return an empty std::optional.
     */
    virtual std::optional<NetworkApiDTO> handleApiCall(const NetworkApiDTO& apiCall) = 0;
};

#endif // HIVE_CONNECT_INETWORKAPIHANDLER_H
