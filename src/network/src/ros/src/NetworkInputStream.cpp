#include "NetworkInputStream.h"
#include "Task.h"
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

constexpr uint8_t gs_LOOP_RATE = 10;

static void serverReceiveTask(void* context) {
    if (context != nullptr) {
        while (true) {
            static_cast<NetworkInputStream*>(context)->acceptingClients();
            Task::delay(gs_LOOP_RATE);
        }
    }
}

NetworkInputStream::NetworkInputStream(ILogger& logger, int listeningPort) :
    m_logger(logger),
    m_serverTask("server_socket", tskIDLE_PRIORITY + 1, serverReceiveTask, this),
    m_listeningPort(listeningPort),
    m_clientSocket(-1),
    m_acceptingSocket(-1) {}

NetworkInputStream::~NetworkInputStream() { stop(); }

bool NetworkInputStream::start() {
    if (m_listeningPort == 0) {
        m_logger.log(LogLevel::Info, "TCP server port set to 0. Not initializing server.");
        return false;
    }

    if ((m_acceptingSocket = ::socket(AF_INET, SOCK_STREAM, 0)) <= 0) {
        m_logger.log(LogLevel::Error, "Failed to create tcp server socket");
        return false;
    }

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(m_listeningPort);
    socklen_t addressLength = sizeof(address);

    if (::bind(m_acceptingSocket, (sockaddr*)&address, addressLength) < 0) {
        m_logger.log(LogLevel::Error, "Failed to bind tcp server socket");
        return false;
    }

    if (::listen(m_acceptingSocket, 10) < 0) {
        m_logger.log(LogLevel::Error, "Failed to listen on tcp server socket");
        return false;
    }
    m_logger.log(LogLevel::Info, "Tcp server socket listening on port %d", m_listeningPort);
    return true;
}

bool NetworkInputStream::stop() {
    ::close(m_acceptingSocket);
    ::close(m_clientSocket);
    m_hasClient = false;
    m_acceptingSocket = -1;
    m_clientSocket = -1;
    return true;
}

void NetworkInputStream::acceptingClients() {
    if (m_acceptingSocket > 0) {
        sockaddr_in clientAddr;
        socklen_t addrLen = sizeof(clientAddr);
        m_clientSocket = ::accept(m_acceptingSocket, (sockaddr*)&clientAddr, &addrLen);
        if (m_clientSocket > 0) {
            m_hasClient = true;
            m_conditionVar.notify_one();
        }

        while (m_clientSocket > 0) {
            // Wait for client to have disconnected
            Task::delay(10);
        }
        m_hasClient = false;
    } else {
        Task::delay(1000); // Sleep if no valid server socket
    }
}

bool NetworkInputStream::receive(uint8_t* data, uint16_t length) {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_clientSocket < 0) {
        m_conditionVar.wait(lock, [&] { return m_hasClient; });
    }
    ssize_t receivedBytes = ::recv(m_clientSocket, data, length, MSG_WAITALL);

    // Check for disconnection
    char peekingBuffer[1];
    if (receivedBytes < 0 ||
        ::recv(m_clientSocket, peekingBuffer, sizeof(peekingBuffer), MSG_PEEK) < 0) {
        m_logger.log(LogLevel::Info, "Client terminated connection");
        ::close(m_clientSocket);
        m_hasClient = false;
        m_clientSocket = -1;
    }

    return receivedBytes == length;
}

bool NetworkInputStream::isReady() { return m_clientSocket > 0; }
