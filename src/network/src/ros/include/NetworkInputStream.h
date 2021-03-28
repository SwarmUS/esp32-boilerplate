#ifndef HIVE_CONNECT_NETWORKINPUTSTREAM_H
#define HIVE_CONNECT_NETWORKINPUTSTREAM_H

#include "BaseTask.h"
#include "INetworkInputStream.h"
#include "Mutex.h"
#include "logger/ILogger.h"
#include <condition_variable>

class NetworkInputStream : public INetworkInputStream {
  public:
    NetworkInputStream(ILogger& logger, int listeningPort);
    ~NetworkInputStream() override;

    bool receive(uint8_t* data, uint16_t length) override;

    bool isReady() override;

    bool start() override;
    bool stop() override;

    void acceptingClients();

  private:
    ILogger& m_logger;
    int m_acceptingSocket;
    int m_clientSocket;
    bool m_hasClient;
    int m_listeningPort;
    BaseTask<configMINIMAL_STACK_SIZE * 5> m_serverTask;
    std::condition_variable m_conditionVar;
    std::mutex m_mutex;
};

#endif // HIVE_CONNECT_NETWORKINPUTSTREAM_H
