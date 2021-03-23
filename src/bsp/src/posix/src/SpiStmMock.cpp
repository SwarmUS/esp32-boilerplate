#include "SpiStmMock.h"
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

SpiStmMock::SpiStmMock(ILogger& logger, int port) : m_logger(logger) {
    m_port = port;
    m_socket = -1;
    m_addressLength = 0;
    m_address = {};
}

bool SpiStmMock::send(const uint8_t* buffer, uint16_t length) {
    if(m_socket < 0) {
        return false;
    }
    return ::send(m_socket, buffer, length, 0) == length;
}

bool SpiStmMock::receive(uint8_t* data, uint16_t length) {
    if(m_socket < 0) {
        return false;
    }
    auto ret = ::recv(m_socket, data, length, MSG_WAITALL);

    if (ret <= 0) {
        m_logger.log(LogLevel::Warn, "Error while reading SPI socket. Client has probably "
                                     "disconnected. Attempting reconnection...");

        ::close(m_socket);
        m_socket = -1;
        // Only return when connection has been restored.
        while (m_socket < 0) {
            this->connect();
        }
        // Returning false since error occurred.
        return false;
    }

    return ret == length;
}

bool SpiStmMock::connect() {
    if(m_socket > 0) {
        m_logger.log(LogLevel::Error, "Trying to override spi mock socket");
        return false;
    }

    m_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(m_socket < 0) {
        m_logger.log(LogLevel::Error, "Could not open socket");
    }
    m_address.sin_family = AF_INET;
    m_address.sin_port = htons(m_port);
    m_address.sin_addr.s_addr = inet_addr("10.0.0.127");

    if(::connect(m_socket, (sockaddr*)&m_address, sizeof(m_address)) < 0 ) {
        m_logger.log(LogLevel::Error, "Could not connect to server");
        ::close(m_socket);
        return false;
    }

    return true;
}

void SpiStmMock::close() {
    ::close(m_socket);
    m_socket = -1;
}
