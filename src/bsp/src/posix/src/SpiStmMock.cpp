#include "SpiStmMock.h"
#include "Task.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

SpiStmMock::SpiStmMock(ILogger& logger, const char* address, int port) : m_logger(logger) {
    m_socket = -1;
    m_address.sin_family = AF_INET;
    m_address.sin_port = htons(port);
    m_address.sin_addr.s_addr = (inet_addr(address));

    while (m_socket < 0) {
        m_logger.log(LogLevel::Error, "Attempting connection to HiveMind");
        this->connect();
        Task::delay(500);
    }
}

SpiStmMock::~SpiStmMock() noexcept { close(); }

bool SpiStmMock::send(const uint8_t* buffer, uint16_t length) {
    if (m_socket < 0) {
        return false;
    }
    if (::send(m_socket, buffer, length, 0) != length) {
        close();
        m_logger.log(LogLevel::Error, "Connection failed while sending data to HiveMind");
        // Only returned when connection has been restored
        while (m_socket < 0) {
            m_logger.log(LogLevel::Error, "Attempting reconnection to HiveMind");
            this->connect();
        }
        return false;
    }
    return true;
}

bool SpiStmMock::receive(uint8_t* data, uint16_t length) {
    if (m_socket < 0) {
        return false;
    }
    auto ret = ::recv(m_socket, data, length, MSG_WAITALL);

    if (ret <= 0) {
        m_logger.log(LogLevel::Warn, "Error while reading SPI socket. Client has probably "
                                     "disconnected. Attempting reconnection...");

        close();
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
    if (m_socket > 0) {
        m_logger.log(LogLevel::Error, "Trying to override spi mock socket");
        return false;
    }

    m_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (m_socket < 0) {
        m_logger.log(LogLevel::Error, "Could not open socket");
    }

    if (::connect(m_socket, (sockaddr*)&m_address, sizeof(m_address)) < 0) {
        m_logger.log(LogLevel::Error, "Could not connect to server");
        ::close(m_socket);
        m_socket = -1;
        return false;
    }
    m_logger.log(LogLevel::Info, "Connected to server!");
    return true;
}

void SpiStmMock::close() {
    ::close(m_socket);
    m_socket = -1;
}

bool SpiStmMock::isConnected() const { return m_socket != -1; }

bool SpiStmMock::isBusy() const { return false; }
