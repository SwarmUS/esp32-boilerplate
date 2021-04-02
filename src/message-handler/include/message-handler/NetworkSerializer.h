#ifndef HIVE_CONNECT_NETWORKSERIALIZER_H
#define HIVE_CONNECT_NETWORKSERIALIZER_H

#include "IAbstractNetworkManager.h"
#include "INetworkOutputStream.h"
#include "common/IProtobufOutputStream.h"
#include "hivemind-host/HiveMindHostSerializer.h"

class NetworkSerializer : public IHiveMindHostSerializer {
  public:
    NetworkSerializer(INetworkOutputStream& stream, IAbstractNetworkManager& networkManager);
    virtual ~NetworkSerializer() = default;

    bool serializeToStream(const MessageDTO& message) override;

  private:
    INetworkOutputStream& m_outputStream;
    IAbstractNetworkManager& m_networkManager;
    HiveMindHostSerializer m_hivemindHostSerializer;
};

#endif // HIVE_CONNECT_NETWORKSERIALIZER_H
