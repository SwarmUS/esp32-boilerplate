#ifndef HIVE_CONNECT_TCPCLIENT_H
#define HIVE_CONNECT_TCPCLIENT_H

#include "INetworkOutputStream.h"
#include <logger/ILogger.h>
#include <lwip/sockets.h>

class TCPClient : public INetworkOutputStream {
  public:
    TCPClient(ILogger& logger);

    ~TCPClient() override;

    bool send(const uint8_t* data, uint16_t length) override;
    bool setDestination(const char* destination) override;
    bool close() override;

  private:
    ILogger& m_logger;
    int m_socketFd;
};

#endif // HIVE_CONNECT_TCPCLIENT_H
