#include "NetworkOutputStream.h"
#include <arpa/inet.h>
#include <cstdlib>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

NetworkOutputStream::NetworkOutputStream(ILogger& logger) : m_logger(logger), m_socketFd(-1) {}

NetworkOutputStream::~NetworkOutputStream() { close(); }

bool NetworkOutputStream::setDestination(const char* destination) {
    sockaddr_in address = {0};

    if ((m_socketFd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) < 0) {
        m_logger.log(LogLevel::Error, "Failed to create tcp client socket");
        return false;
    }

    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port = htons(atoi(destination));
    address.sin_family = AF_INET;

    if (::connect(m_socketFd, (sockaddr*)&address, sizeof(address)) != 0) {
        m_logger.log(LogLevel::Error, "Failed to connect to tcp endpoint");
        return false;
    }

    return true;
}

bool NetworkOutputStream::close() {
    if (m_socketFd) {
        ::close(m_socketFd);
    }
    m_socketFd = -1;
    return true;
}

bool NetworkOutputStream::send(const uint8_t* data, uint16_t length) {
    if (m_socketFd < 0) {
        return false;
    }

    return ::send(m_socketFd, data, length, 0) == length;
}
