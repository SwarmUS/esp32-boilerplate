#ifndef HIVE_CONNECT_TCPSERVER_H
#define HIVE_CONNECT_TCPSERVER_H

#include "BaseTask.h"
#include "INetworkDeserializer.h"
#include "Mutex.h"
#include "c-common/circular_buff.h"
#include "logger/Logger.h"
#include <array>

constexpr uint16_t g_MAX_BUFFER_SIZE = 1024;

class TCPServer : public INetworkDeserializer {
  public:
    TCPServer(ILogger& logger);
    ~TCPServer();

    bool receive(uint8_t* data, uint16_t length) override;
    bool send(const uint8_t* data, uint16_t length) override;
    bool start() override;
    bool stop() override;
    bool isReady() override;

    void receiveTask();


  private:
    ILogger& m_logger;
    std::array<uint8_t, g_MAX_BUFFER_SIZE> m_buffer;
    CircularBuff m_circularBuffer;
    int m_socket;
    bool m_isBusy;
    BaseTask<configMINIMAL_STACK_SIZE * 5> m_serverTask;
    Mutex m_serverMutex;
};

#endif // HIVE_CONNECT_TCPSERVER_H
