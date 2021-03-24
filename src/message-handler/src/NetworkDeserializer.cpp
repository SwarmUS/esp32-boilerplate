#include "NetworkDeserializer.h"
#include <pb_decode.h>

NetworkDeserializer::NetworkDeserializer(INetworkInputStream& inputStream,
                                         INetworkManager& networkManager) :
    m_inputStream(inputStream), m_networkManager(networkManager) {}

bool NetworkDeserializer::deserializeFromStream(MessageDTO& message) {
    Message msgReceive = Message_init_default;

    pb_istream_t inputStream{NetworkDeserializer::streamCallback, this, SIZE_MAX, 0};

    bool status = pb_decode_ex(&inputStream, Message_fields, &msgReceive, PB_DECODE_DELIMITED);

    if (status) {
        message = MessageDTO(msgReceive);
    }

    return status;
}

bool NetworkDeserializer::streamCallback(pb_istream_t* stream, pb_byte_t* buf, size_t count) {
    auto* deserializer = (NetworkDeserializer*)stream->state;
    return deserializer->m_inputStream.receive(buf, count);
}