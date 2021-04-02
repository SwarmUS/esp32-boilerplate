#include "NetworkSerializer.h"
#include <pb_encode.h>

NetworkSerializer::NetworkSerializer(INetworkOutputStream& stream,
                                     IAbstractNetworkManager& manager) :
    m_outputStream(stream), m_networkManager(manager), m_hivemindHostSerializer(stream) {}

bool NetworkSerializer::serializeToStream(const MessageDTO& message) {
    Message msg;
    message.serialize(msg);

    auto ip = m_networkManager.getIPFromAgentID(message.getDestinationId());
    if (!ip.has_value()) {
        return false;
    }

    if (!m_outputStream.setDestination(ip.value())) {
        return false;
    }

    return m_hivemindHostSerializer.serializeToStream(message);
}