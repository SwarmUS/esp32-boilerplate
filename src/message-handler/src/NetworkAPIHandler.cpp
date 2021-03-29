#include "NetworkAPIHandler.h"

NetworkAPIHandler::NetworkAPIHandler(IBSP& bsp, ILogger& logger) : m_bsp(bsp), m_logger(logger) {}

std::variant<ErrorNum, std::optional<NetworkApiDTO>> NetworkAPIHandler::handleApiCall(
    const NetworkApiDTO& apiCall) {
    const NetworkApiDTOType& call = apiCall.getApiCall();
    if (const auto* ipRequest = std::get_if<IPDiscoveryDTO>(&call)) {
        // Todo: Properly handle call. Return an empty DTO for now.
        return std::optional<NetworkApiDTO>({});
    }
    return ErrorNum(ErrorNum::UNKNOWN_CALL);
}