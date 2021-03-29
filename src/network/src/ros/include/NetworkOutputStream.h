#ifndef HIVE_CONNECT_NETWORKOUTPUTSTREAM_H
#define HIVE_CONNECT_NETWORKOUTPUTSTREAM_H

#include "INetworkOutputStream.h"
#include "logger/ILogger.h"

class NetworkOutputStream : public INetworkOutputStream {
  public:
    NetworkOutputStream(ILogger& logger);
    ~NetworkOutputStream() override;

    bool send(const uint8_t* data, uint16_t length) override;

    bool setDestination(const char* destination) override;

    bool close() override;

  private:
    ILogger& m_logger;
    int m_socketFd;
};

#endif // HIVE_CONNECT_NETWORKOUTPUTSTREAM_H
