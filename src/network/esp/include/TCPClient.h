#ifndef HIVE_CONNECT_TCPCLIENT_H
#define HIVE_CONNECT_TCPCLIENT_H

#include <logger/ILogger.h>
#include <lwip/sockets.h>

class TCPClient {
  public:
    TCPClient(int socket, sockaddr_in address, ILogger& logger);

    ~TCPClient() = default;

    bool receive(uint8_t* data, uint16_t length);

    bool send(const uint8_t* data, uint16_t length);

    bool close();

  private:
    ILogger& m_logger;
    const int m_socketFd;
    const sockaddr_in m_address;
};


#endif // HIVE_CONNECT_TCPCLIENT_H
