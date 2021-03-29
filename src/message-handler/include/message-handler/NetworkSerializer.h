#ifndef HIVE_CONNECT_NETWORKSERIALIZER_H
#define HIVE_CONNECT_NETWORKSERIALIZER_H

#include "INetworkManager.h"
#include "INetworkOutputStream.h"
#include "common/IProtobufOutputStream.h"
#include "hivemind-host/HiveMindHostSerializer.h"

class NetworkSerializer : public IHiveMindHostSerializer {
  public:
    NetworkSerializer(INetworkOutputStream& stream, INetworkManager& networkManager);
    virtual ~NetworkSerializer() = default;

    bool serializeToStream(const MessageDTO& message) override;

  private:
    INetworkOutputStream& m_outputStream;
    INetworkManager& m_networkManager;
    HiveMindHostSerializer m_hivemindHostSerializer;
};

#endif // HIVE_CONNECT_NETWORKSERIALIZER_H
