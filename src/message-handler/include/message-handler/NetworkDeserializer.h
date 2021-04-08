#ifndef HIVE_CONNECT_NETWORKDESERIALIZER_H
#define HIVE_CONNECT_NETWORKDESERIALIZER_H

#include "INetworkInputStream.h"
#include "INetworkManager.h"
#include "pheromones/HiveMindHostDeserializer.h"
#include "pheromones/IProtobufInputStream.h"

class NetworkDeserializer : public IHiveMindHostDeserializer {
  public:
    NetworkDeserializer(INetworkInputStream& stream);
    virtual ~NetworkDeserializer() = default;

    bool deserializeFromStream(MessageDTO& message) override;

  private:
    INetworkInputStream& m_inputStream;
    HiveMindHostDeserializer m_hiveMindHostDeserializer;
};

#endif // HIVE_CONNECT_NETWORKDESERIALIZER_H
