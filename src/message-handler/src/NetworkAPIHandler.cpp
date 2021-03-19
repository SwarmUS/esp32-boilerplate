#include "NetworkAPIHandler.h"

NetworkAPIHandler::NetworkAPIHandler(IBSP& bsp, ILogger& logger) : m_bsp(bsp), m_logger(logger) {}

std::optional<NetworkApiDTO> NetworkAPIHandler::handleApiCall(const NetworkApiDTO& apiCall) {
    const NetworkApiDTOType& call = apiCall.getApiCall();
    if (const auto* ipRequest = std::get_if<IPDiscoveryDTO>(&call)) {
        // Todo: Properly handle call. Return an empty DTO for now.
        // Returning a monostate DTO is not interpreted as an error. It just means that the handinlg
        // of the api call does not generate another api call.
        return NetworkApiDTO();
    }

    // Return an empty optional to signal an error occurred while handling apiCall.
    return {};
}