#ifndef HIVE_CONNECT_SPISTMMOCK_H
#define HIVE_CONNECT_SPISTMMOCK_H

#include "bsp/ISpiStm.h"
#include "logger/ILogger.h"
#include "netinet/in.h"

class SpiStmMock : public ISpiStm {
  public:
    SpiStmMock(ILogger& logger, const char* address, int port);
    ~SpiStmMock() override;

    bool send(const uint8_t* buffer, uint16_t length) override;
    bool receive(uint8_t* data, uint16_t length) override;
    bool isBusy() const override;
    bool isConnected() const override;

    bool connect();
    void close();

  private:
    ILogger& m_logger;
    int m_socket;
    sockaddr_in m_address;
};

#endif // HIVE_CONNECT_SPISTMMOCK_H
