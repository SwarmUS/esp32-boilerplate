#ifndef HIVE_CONNECT_NETWORKBROADCAST_H
#define HIVE_CONNECT_NETWORKBROADCAST_H

#include "INetworkBroadcast.h"
#include "logger/ILogger.h"

class NetworkBroadcast : public INetworkBroadcast {
  public:
    NetworkBroadcast(ILogger& logger);
    ~NetworkBroadcast() override;

    bool send(const uint8_t* data, uint16_t length) override;

    bool receive(uint8_t* data, uint16_t length) override;

    bool start() override;

    bool stop() override;

  private:
    ILogger& m_logger;
    int m_socket;
    bool m_started;
};

#endif // HIVE_CONNECT_NETWORKBROADCAST_H
