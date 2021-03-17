#include "SwarmApiRequestHandler.h"

SwarmApiRequestHandler::SwarmApiRequestHandler(IBSP& bsp, ILogger& logger)  : m_bsp(bsp), m_logger(logger)  {}

SwarmApiResponseDTO SwarmApiRequestHandler::handleRequest(const SwarmApiRequestDTO& request) {
    const std::variant<std::monostate, IdRequestDTO>& vReq = request.getRequest();

    if (std::holds_alternative<IdRequestDTO>(vReq)) {
        return SwarmApiResponseDTO(IdResponseDTO(m_bsp.getUUID()));
    }

    return SwarmApiResponseDTO(GenericResponseDTO(GenericResponseStatusDTO::BadRequest, ""));
}