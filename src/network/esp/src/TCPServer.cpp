#include "TCPServer.h"
#include "SocketFactory.h"
#include "Task.h"
#include "lwip/sockets.h"

constexpr uint8_t gs_LOOP_RATE = 10;

constexpr uint16_t gs_SERVER_PORT = 6969;

static void serverReceiveTask(void* context) {

    if (context != nullptr) {
        while (true) {
            static_cast<TCPServer*>(context)->receiveTask();
            Task::delay(gs_LOOP_RATE);
        }
    }
}

TCPServer::TCPServer(ILogger& logger) :
    m_serverTask("server_socket", tskIDLE_PRIORITY + 1, serverReceiveTask, this),
    m_socket(NO_SOCKET),
    m_logger(logger) {
    m_isBusy = false;

    m_serverTask.start();
}

TCPServer::~TCPServer() {
    if (m_socket != NO_SOCKET) {
        lwip_close(m_socket);
    }
}

void TCPServer::receiveTask() {
    // Only receive with a valid socket
    if (m_socket != NO_SOCKET) {
        int clientfd;
        sockaddr_in clientAddr;
        socklen_t addrlen = sizeof(clientAddr);
        char buffer[1024];
        int nbytes;

        // lwip_accept is blocking
        m_logger.log(LogLevel::Info, "Awaiting connection..");
        clientfd = lwip_accept(m_socket, (sockaddr*)&clientAddr, &addrlen);

        if (clientfd > 0) {
            m_isBusy = true;
            nbytes = lwip_recv(clientfd, buffer, sizeof(buffer), 0);
            if (nbytes > 0) { // client is active
                do {
                    if (nbytes > 0) {
                        lwip_send(clientfd, buffer, nbytes, 0); // echo action
                        m_logger.log(LogLevel::Info, "Received: %s", buffer);
                    }
                    nbytes = lwip_recv(clientfd, buffer, sizeof(buffer), 0);
                } while (nbytes > 0);
            }
            m_logger.log(LogLevel::Info, "Client terminated connection");
            lwip_close(clientfd);
            m_isBusy = false;
        }
    } else {
        Task::delay(1000); // Sleep if no socket
    }
}

bool TCPServer::start() {
    m_logger.log(LogLevel::Info, "Starting tcp server");
    m_socket = SocketFactory::createTCPServerSocket(gs_SERVER_PORT);

    return m_socket != NO_SOCKET;
}

void TCPServer::stop() {
    m_logger.log(LogLevel::Info, "Stopping tcp server");
    if (m_socket != NO_SOCKET) {
        lwip_close(m_socket);
    }
    m_socket = NO_SOCKET; // Reset socket
}

bool TCPServer::isBusy() const { return m_isBusy; }
