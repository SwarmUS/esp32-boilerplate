#include "TCPServer.h"
#include "LockGuard.h"
#include "NetworkConfig.h"
#include "SocketFactory.h"
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

TCPServer::TCPServer(ILogger& logger) :
    m_serverTask("server_socket", tskIDLE_PRIORITY + 1, serverReceiveTask, this),
    m_socket(NO_SOCKET),
    m_logger(logger),
    m_serverMutex(10) {
    m_isBusy = false;

    CircularBuff_init(&m_circularBuffer, m_buffer.data(), m_buffer.size());

    m_serverTask.start();
}

bool TCPServer::send(const uint8_t* data, uint16_t length) {
    (void)data;
    (void)length;
    return false;
}

bool TCPServer::receive(uint8_t* data, uint16_t length) {
    if (data == nullptr || length > m_buffer.size()) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for Server receive");
        return false;
    }

    LockGuard lock = LockGuard(m_serverMutex);
    CircularBuff_get(&m_circularBuffer, data, length);
    return true;
}

bool TCPServer::isReady() { return m_socket != NO_SOCKET; }

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
        uint8_t buffer[g_MAX_BUFFER_SIZE];
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
                        LockGuard lock = LockGuard(m_serverMutex);
                        CircularBuff_put(&m_circularBuffer, buffer, nbytes);
                        // TODO: remove this log call
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
    m_socket = SocketFactory::createTCPServerSocket(NetworkConfig::getCommunicationPort());

    return m_socket != NO_SOCKET;
}

bool TCPServer::stop() {
    m_logger.log(LogLevel::Info, "Stopping tcp server");
    if (m_socket != NO_SOCKET && lwip_close(m_socket) == 0) {
        m_socket = NO_SOCKET;
        return true;
    }
    m_logger.log(LogLevel::Error, "Failed to stop TCP server");
    return false;
}
