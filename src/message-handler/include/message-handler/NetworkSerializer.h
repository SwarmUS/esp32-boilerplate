#ifndef HIVE_CONNECT_NETWORKSERIALIZER_H
#define HIVE_CONNECT_NETWORKSERIALIZER_H

#include "INetworkManager.h"
#include "INetworkOutputStream.h"
#include "common/IProtobufOutputStream.h"
#include "hivemind-host/IHiveMindHostSerializer.h"

class NetworkSerializer : public IHiveMindHostSerializer {
  public:
    NetworkSerializer(INetworkOutputStream& stream, INetworkManager& networkManager);
    virtual ~NetworkSerializer() = default;

    bool serializeToStream(const MessageDTO& message) override;

  private:
    INetworkOutputStream& m_outputStream;
    INetworkManager& m_networkManager;

    static bool streamCallback(pb_ostream_t* stream, const pb_byte_t* buf, size_t count);
};

#endif // HIVE_CONNECT_NETWORKSERIALIZER_H
