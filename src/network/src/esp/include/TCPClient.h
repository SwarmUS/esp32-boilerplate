#ifndef HIVE_CONNECT_TCPCLIENT_H
#define HIVE_CONNECT_TCPCLIENT_H

#include "INetworkSerializer.h"
#include <logger/ILogger.h>
#include <lwip/sockets.h>

class TCPClient : public INetworkSerializer {
  public:
    TCPClient(ILogger& logger);

    ~TCPClient() = default;

    bool send(const uint8_t* data, uint16_t length) override;
    bool receive(uint8_t* data, uint16_t length) override;
    bool setDestination(const char* destination) override;
    bool close() override;

  private:
    ILogger& m_logger;
    int m_socketFd;
    bool m_hasSocket;
};

#endif // HIVE_CONNECT_TCPCLIENT_H
