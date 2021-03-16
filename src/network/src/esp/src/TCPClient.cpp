#include "TCPClient.h"
#include "NetworkConfig.h"
#include "SocketFactory.h"
#include "lwip/sockets.h"

TCPClient::TCPClient(ILogger& logger) : m_logger(logger) { m_socketFd = NO_SOCKET; }

TCPClient::~TCPClient() { close(); }

bool TCPClient::setDestination(const char* address) {
    if (m_socketFd != NO_SOCKET) {
        m_logger.log(LogLevel::Error, "Trying to override socket for TCP client");
    } else if ((m_socketFd = SocketFactory::createTCPClient(
                    address, NetworkConfig::getCommunicationPort())) == NO_SOCKET) {
        m_logger.log(LogLevel::Error, "Failed to acquire connected socket for TCP client");
    } else {
        return true;
    }

    return false;
}

bool TCPClient::send(const uint8_t* data, uint16_t length) {

    if (m_socketFd == NO_SOCKET) {
        m_logger.log(LogLevel::Error, "Trying to send message with no socket client socket set");
        return false;
    }
    ssize_t sentBytes = lwip_send(m_socketFd, data, length, 0);

    if (sentBytes < 0) {
        m_logger.log(LogLevel::Error, "Failed to send data");
        closesocket(m_socketFd);
        m_socketFd = NO_SOCKET;
        return false;
    }

    while (sentBytes < length) {
        ssize_t nbytes = lwip_send(m_socketFd, data + sentBytes, (length - sentBytes), 0);

        if (nbytes < 0) {

            m_logger.log(LogLevel::Error, "Error while sending data from socket");
            closesocket(m_socketFd);
            m_socketFd = NO_SOCKET;
            return false;
        }
        sentBytes += nbytes;
    }

    return sentBytes == length;
}

bool TCPClient::close() {
    if (m_socketFd != NO_SOCKET && closesocket(m_socketFd) == 0) {
        m_socketFd = NO_SOCKET;
        return true;
    }

    return false;
}