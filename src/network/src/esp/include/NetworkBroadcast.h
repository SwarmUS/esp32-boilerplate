#ifndef HIVE_CONNECT_NETWORKBROADCAST_H
#define HIVE_CONNECT_NETWORKBROADCAST_H

#include "BaseTask.h"
#include "DefaultNetworkConfig.h"
#include "INetworkBroadcast.h"
#include "c-common/circular_buff.h"
#include "logger/ILogger.h"
#include <array>

class NetworkBroadcast : public INetworkBroadcast {
  public:
    NetworkBroadcast(ILogger& logger);
    ~NetworkBroadcast() override;

    bool send(const uint8_t* data, uint16_t length) override;

    bool receive(uint8_t* data, uint16_t length) override;

    bool start() override;

    bool stop() override;

    void receiveDatagrams();

  private:
    ILogger& m_logger;
    BaseTask<2 * configMINIMAL_STACK_SIZE> m_receivingTask;
    std::array<uint8_t, 512> m_ciruclarBuffData;
    CircularBuff m_circularBuffer;
    TaskHandle_t m_receivingTaskHandle;
    int m_socket;
    bool m_started;
    std::array<uint8_t, 255> m_datagram;
};

#endif // HIVE_CONNECT_NETWORKBROADCAST_H
