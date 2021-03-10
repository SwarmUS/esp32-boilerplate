#include "TCPClient.h"
#include "NetworkConfig.h"
#include "SocketFactory.h"
#include "lwip/sockets.h"

TCPClient::TCPClient(ILogger& logger) : m_logger(logger) {
    m_isBusy = false;
    m_hasSocket = false;
    m_socketFd = NO_SOCKET;
}

bool TCPClient::setDestination(const char* address) {
    if (m_hasSocket) {
        m_logger.log(LogLevel::Error, "Trying to override socket for TCP client");
    } else if ((m_socketFd = SocketFactory::createTCPClient(
                    address, NetworkConfig::getCommunicationPort())) == NO_SOCKET) {
        m_logger.log(LogLevel::Error, "Failed to acquire connected socket for TCP client");
    } else {
        m_hasSocket = true;
        return true;
    }

    return false;
}

bool TCPClient::send(const uint8_t* data, uint16_t length) {

    if (!m_hasSocket) {
        m_logger.log(LogLevel::Error, "Trying to send message with no socket client socket set");
        return false;
    }

    if (lwip_send(m_socketFd, data, length, 0) < 0) {
        m_logger.log(LogLevel::Error, "Failed to send message");
    } else {
        lwip_close(m_socketFd);
        m_socketFd = NO_SOCKET;
        m_hasSocket = false;
        return true;
    }
    return false;
}

bool TCPClient::isReady() const { return m_hasSocket; }

void TCPClient::reset() {
    if (m_socketFd != NO_SOCKET) {
        lwip_close(m_socketFd);
        m_hasSocket = false;
    }
}