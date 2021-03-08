#ifndef HIVE_CONNECT_TCPSERVER_H
#define HIVE_CONNECT_TCPSERVER_H

#include "BaseTask.h"
#include "logger/Logger.h"

class TCPServer {
  public:
    TCPServer(ILogger& logger);
    ~TCPServer();

    void receiveTask();
    bool start();
    void stop();

    bool isBusy() const;

  private:
    ILogger& m_logger;

    int m_socket;
    bool m_isBusy;
    BaseTask<configMINIMAL_STACK_SIZE * 3> m_serverTask;
};

#endif // HIVE_CONNECT_TCPSERVER_H
