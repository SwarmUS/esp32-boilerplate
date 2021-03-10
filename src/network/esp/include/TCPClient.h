#ifndef HIVE_CONNECT_TCPCLIENT_H
#define HIVE_CONNECT_TCPCLIENT_H

#include <logger/ILogger.h>
#include <lwip/sockets.h>

class TCPClient {
  public:
    TCPClient(ILogger& logger);

    ~TCPClient() = default;

    bool send(const uint8_t* data, uint16_t length);
    bool setDestination(const char* address);
    bool isReady() const;
    void reset();

  private:
    ILogger& m_logger;
    int m_socketFd;
    bool m_isBusy;
    bool m_hasSocket;
};

#endif // HIVE_CONNECT_TCPCLIENT_H
