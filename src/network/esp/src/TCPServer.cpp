#include "TCPServer.h"
#include "Task.h"
#include "lwip/sockets.h"

constexpr uint8_t gs_LOOP_RATE = 10;

static void serverReceiveTask(void* context) {
    if (context != nullptr) {
        while (true) {
            static_cast<TCPServer*>(context)->receiveTask();
            Task::delay(gs_LOOP_RATE);
        }
    }
}

TCPServer::TCPServer(int socket, ILogger& logger) :
    m_serverTask("server_socket", tskIDLE_PRIORITY + 1, serverReceiveTask, this),
    m_socket(socket),
    m_logger(logger) {
    m_isBusy = false;
}

TCPServer::~TCPServer() { lwip_close(m_socket); }

void TCPServer::receiveTask() {
    int clientfd;
    sockaddr_in client_addr;
    socklen_t addrlen = sizeof(client_addr);
    char buffer[1024];
    int nbytes;

    clientfd = lwip_accept(m_socket, (struct sockaddr*)&client_addr, &addrlen);
    if (clientfd > 0) {
        m_isBusy = true;
        nbytes = lwip_recv(clientfd, buffer, sizeof(buffer), 0);
        if (nbytes > 0) { // client is active
            do {
                nbytes = lwip_recv(clientfd, buffer, sizeof(buffer), 0);
                if (nbytes > 0)
                    lwip_send(clientfd, buffer, nbytes, 0); // echo action
            } while (nbytes > 0);
        }
        m_logger.log(LogLevel::Info, "Client terminated connection");
        lwip_close(clientfd);
    }
}

void TCPServer::start() {
    m_serverTask.start();
}

bool TCPServer::isBusy() const { return m_isBusy; }