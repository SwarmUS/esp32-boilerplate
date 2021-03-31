#include "NetworkSerializer.h"
#include <pb_encode.h>

NetworkSerializer::NetworkSerializer(INetworkOutputStream& stream, INetworkManager& manager) :
    m_outputStream(stream), m_networkManager(manager), m_hivemindHostSerializer(stream) {}

bool NetworkSerializer::serializeToStream(const MessageDTO& message) {
    Message msg;
    message.serialize(msg);

    char destination[16];
    if (!m_networkManager.getIPFromAgentID(message.getDestinationId(), destination,
                                           sizeof(destination))) {
        return false;
    }

    if (!m_outputStream.setDestination(destination)) {
        return false;
    }

    return m_hivemindHostSerializer.serializeToStream(message);
}