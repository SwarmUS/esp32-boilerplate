#include "NetworkSerializer.h"
#include <pb_encode.h>

NetworkSerializer::NetworkSerializer(INetworkOutputStream& stream, INetworkManager& manager) :
    m_outputStream(stream), m_networkManager(manager) {}

bool NetworkSerializer::serializeToStream(const MessageDTO& message) {
    Message msg;
    message.serialize(msg);

    char destination[16];
    if (!m_networkManager.getIPFromRobotID(message.getDestinationId(), destination,
                                           sizeof(destination))) {
        return false;
    }

    if (!m_outputStream.setDestination(destination)) {
        return false;
    }

    pb_ostream_s outputStream{NetworkSerializer::streamCallback, this, SIZE_MAX, 0, 0};

    return pb_encode_ex(&outputStream, Message_fields, &msg, PB_ENCODE_DELIMITED);
}

bool NetworkSerializer::streamCallback(pb_ostream_t* stream, const pb_byte_t* buf, size_t count) {
    auto* serializer = (NetworkSerializer*)stream->state;
    return serializer->m_outputStream.send(buf, count);
}