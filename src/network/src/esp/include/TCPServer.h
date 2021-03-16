#ifndef HIVE_CONNECT_TCPSERVER_H
#define HIVE_CONNECT_TCPSERVER_H

#include "BaseTask.h"
#include "INetworkInputStream.h"
#include "Mutex.h"
#include "c-common/circular_buff.h"
#include "logger/Logger.h"
#include <array>

constexpr uint16_t g_MAX_BUFFER_SIZE = 1024;

class TCPServer : public INetworkInputStream {
  public:
    TCPServer(ILogger& logger);
    ~TCPServer() override;

    bool receive(uint8_t* data, uint16_t length) override;
    bool start() override;
    bool stop() override;
    bool isReady() override;

    void receiveTask();

  private:
    ILogger& m_logger;
    int m_acceptingSocket;
    int m_clientSocket;
    BaseTask<configMINIMAL_STACK_SIZE * 5> m_serverTask;
    TaskHandle_t m_receivingTaskHandle;
};

#endif // HIVE_CONNECT_TCPSERVER_H
