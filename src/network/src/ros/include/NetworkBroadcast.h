#ifndef HIVE_CONNECT_NETWORKBROADCAST_H
#define HIVE_CONNECT_NETWORKBROADCAST_H

#include "BaseTask.h"
#include "INetworkBroadcast.h"
#include "logger/ILogger.h"
#include <array>
#include <netinet/in.h>

constexpr uint32_t g_maxBroadcastReceiveSize = 2048;

class NetworkBroadcast : public INetworkBroadcast {
  public:
    NetworkBroadcast(ILogger& logger, int inputPort, int outputPort);
    ~NetworkBroadcast() override;

    bool send(const uint8_t* data, uint16_t length) override;
    bool receive(uint8_t* data, uint16_t length) override;
    bool start() override;
    bool stop() override;

  private:
    ILogger& m_logger;
    BaseTask<3 * configMINIMAL_STACK_SIZE> m_receivingTask;
    std::array<uint8_t, g_maxBroadcastReceiveSize> m_data;

    bool createInputSocket();
    bool createOutputSocket();

    int m_inputPort;
    int m_outputPort;
    int m_inputSocketFd;
    int m_outputSocketFd;
};

#endif // HIVE_CONNECT_NETWORKBROADCAST_H
