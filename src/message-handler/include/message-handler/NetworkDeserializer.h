#ifndef HIVE_CONNECT_NETWORKDESERIALIZER_H
#define HIVE_CONNECT_NETWORKDESERIALIZER_H

#include "INetworkInputStream.h"
#include "INetworkManager.h"
#include "hivemind-host/IHiveMindHostDeserializer.h"

class NetworkDeserializer : public IHiveMindHostDeserializer {
  public:
    NetworkDeserializer(INetworkInputStream& inputStream, INetworkManager& networkManager);
    ~NetworkDeserializer() override = default;

    bool deserializeFromStream(MessageDTO& message) override;

  private:
    INetworkInputStream& m_inputStream;
    INetworkManager& m_networkManager;

    static bool streamCallback(pb_istream_t* stream, pb_byte_t* buf, size_t count);
};

#endif // HIVE_CONNECT_NETWORKDESERIALIZER_H
