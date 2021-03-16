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
    m_acceptingSocket(NO_SOCKET),
    m_clientSocket(NO_SOCKET),
    m_logger(logger) {

    m_serverTask.start();
}

TCPServer::~TCPServer() { stop(); }

bool TCPServer::receive(uint8_t* data, uint16_t length) {

    m_receivingTaskHandle = xTaskGetCurrentTaskHandle();
    while (m_clientSocket == NO_SOCKET) {
        // Gets notified everytime a new packet is appended to stream
        ulTaskNotifyTake(pdTRUE, 500);
    }
    m_receivingTaskHandle = NULL;

    ssize_t receivedBytes = lwip_recv(m_clientSocket, data, (length - receivedBytes), 0);

    if (receivedBytes < 0) {
        m_logger.log(LogLevel::Error, "Failed to read data from fresh connection");
        lwip_close(m_clientSocket);
        m_clientSocket = NO_SOCKET;
        return false;
    }

    while (receivedBytes <= length) {
        ssize_t nbytes =
            lwip_recv(m_clientSocket, data + receivedBytes, (length - receivedBytes), 0);

        if (nbytes < 0) {
            // Only close socket after client disconnected. When client is still connected but not
            // transmitting, lwip_rcv will return 0. When the client disconnects, lwip_rcv will
            // return -1.
            m_logger.log(LogLevel::Info, "Client terminated connection");
            lwip_close(m_clientSocket);
            m_clientSocket = NO_SOCKET;
            break;
        }
        receivedBytes += nbytes;
    }

    return receivedBytes == length;
}

bool TCPServer::isReady() { return m_acceptingSocket != NO_SOCKET; }

void TCPServer::receiveTask() {
    // Only receive with a valid server socket
    if (m_acceptingSocket != NO_SOCKET) {
        sockaddr_in clientAddr;
        socklen_t addrlen = sizeof(clientAddr);

        // lwip_accept is blocking
        m_logger.log(LogLevel::Info, "Awaiting connection..");
        m_clientSocket = lwip_accept(m_acceptingSocket, (sockaddr*)&clientAddr, &addrlen);

        // Notify the waiting task that new connection was established
        if (m_receivingTaskHandle != NULL) {
            xTaskNotifyGive(m_receivingTaskHandle);
        }
        while (m_clientSocket != NO_SOCKET) {
            // Wait for client to have disconnected
            Task::delay(10);
        }
    } else {
        Task::delay(1000); // Sleep if no valid server socket
    }
}

bool TCPServer::start() {
    m_logger.log(LogLevel::Info, "Starting tcp server");
    m_acceptingSocket = SocketFactory::createTCPServerSocket(NetworkConfig::getCommunicationPort());

    return m_acceptingSocket != NO_SOCKET;
}

bool TCPServer::stop() {
    m_logger.log(LogLevel::Info, "Stopping tcp server");
    if (m_acceptingSocket != NO_SOCKET && lwip_close(m_acceptingSocket) == 0) {
        m_acceptingSocket = NO_SOCKET;
        return true;
    }
    m_logger.log(LogLevel::Error, "Failed to stop TCP server");
    return false;
}
