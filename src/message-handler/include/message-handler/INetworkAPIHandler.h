#ifndef HIVE_CONNECT_INETWORKAPIHANDLER_H
#define HIVE_CONNECT_INETWORKAPIHANDLER_H

#include <hivemind-host/MessageDTO.h>
#include <hivemind-host/NetworkApiDTO.h>
#include <optional>
#include <variant>

enum class ErrorNum { UNKNOWN_CALL = 1 };

/**
 *@brief Handles SwarmApi requests and sends the response to the appropriate target */
class INetworkAPIHandler {
  public:
    virtual ~INetworkAPIHandler() = default;

    /**
     *@brief handles the call and generate an optional response to the call or an error number
     *@param message Message container of api call
     *@param apiCall the call to handle
     *@return The response to dispatch. If no response need to be sent, will return an empty
     *optional. If the handling failed, it will return an ErrorNum to diagnose the type of error
     *that occured.
     */
    virtual std::variant<ErrorNum, std::optional<NetworkApiDTO>> handleApiCall(
        const MessageDTO& message, const NetworkApiDTO& apiCall) = 0;
};

#endif // HIVE_CONNECT_INETWORKAPIHANDLER_H
