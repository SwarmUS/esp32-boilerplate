#ifndef HIVE_CONNECT_INETWORKAPIHANDLER_H
#define HIVE_CONNECT_INETWORKAPIHANDLER_H

#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/NetworkApiDTO.h>
#include <variant>

enum class ErrorNum { UNKNOWN_CALL = 1 };

/**
 *@brief Handles SwarmApi requests and sends the response to the appropriate target */
class INetworkAPIHandler {
  public:
    virtual ~INetworkAPIHandler() = default;

    /**
     *@brief handles the call and generate an optional response to the call
     *@param apiCall the call to handle
     *@return The response to dispatch. If no response need to be sent, will return a NetworkAPIDTO
     *with an std::monostate. If the handling failed, it will return an ErrorNum to diagnose the
     *type of error that occured.
     */
    virtual std::variant<ErrorNum, NetworkApiDTO> handleApiCall(const NetworkApiDTO& apiCall) = 0;
};

#endif // HIVE_CONNECT_INETWORKAPIHANDLER_H
