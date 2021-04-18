#include "NetworkDeserializer.h"

NetworkDeserializer::NetworkDeserializer(INetworkInputStream& stream) :
    m_inputStream(stream), m_hiveMindHostDeserializer(stream) {}

bool NetworkDeserializer::deserializeFromStream(MessageDTO& message) {
    bool ret = m_hiveMindHostDeserializer.deserializeFromStream(message);
    m_inputStream.closeCurrentClient();
    return ret;
}