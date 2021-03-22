#ifndef HIVE_CONNECT_SPISTMMOCK_H
#define HIVE_CONNECT_SPISTMMOCK_H

#include "bsp/ISpiStm.h"
#include "logger/ILogger.h"
#include "netinet/in.h"

class SpiStmMock : public ISpiStm {
  public:
    SpiStmMock(ILogger& logger);
    ~SpiStmMock() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;
    bool receive(uint8_t *data, uint16_t length) override;
    bool isBusy() const override;
    bool isConnected() const override;

    void openSocket(int port);
    bool connect();
    void close() const;

  private:
    ILogger& m_logger;

    int m_port;
    int m_socket;
    int m_addressLength;
    struct sockaddr_in m_address {};
};

#endif // HIVE_CONNECT_SPISTMMOCK_H
