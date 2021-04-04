#include "NetworkAPIHandler.h"

NetworkAPIHandler::NetworkAPIHandler(IBSP& bsp, ILogger& logger, INetworkManager& networkManager) :
    m_bsp(bsp), m_logger(logger), m_networkManager(networkManager) {}

std::variant<ErrorNum, std::optional<NetworkApiDTO>> NetworkAPIHandler::handleApiCall(
    uint16_t sourceID, const NetworkApiDTO& apiCall) {
    const NetworkApiDTOType& call = apiCall.getApiCall();
    if (const auto* ipDiscovery = std::get_if<IPDiscoveryDTO>(&call)) {
        if (m_networkManager.registerAgent(sourceID, ipDiscovery->getIP())) {
            m_logger.log(LogLevel::Info, "Succesfull handler ipDiscovery call");
            // Return an empty optional since no action to take afterwards
            return std::optional<NetworkApiDTO>({});
        }
    }
    return ErrorNum(ErrorNum::UNKNOWN_CALL);
}