#include "NetworkBroadcast.h"
#include <unistd.h>

static void receivingTask(void* context) {}

NetworkBroadcast::NetworkBroadcast(ILogger& logger, int inputPort, int outputPort) :
    m_receivingTask("broadcast_task", tskIDLE_PRIORITY + 1, receivingTask, this),
    m_logger(logger),
    m_inputPort(inputPort),
    m_outputPort(outputPort),
    m_inputSocketFd(-1),
    m_outputSocketFd(-1) {}

NetworkBroadcast::~NetworkBroadcast() { stop(); }

bool NetworkBroadcast::createInputSocket() {
    if ((m_inputSocketFd = ::socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        m_logger.log(LogLevel::Error, "Failed to create input socket for broadcast");
        return false;
    }

    sockaddr_in address;
    address.sin_port = htons(m_inputPort);
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    socklen_t addressLength = sizeof(address);

    if (::bind(m_inputSocketFd, (sockaddr*)&address, addressLength) < 0) {
        m_logger.log(LogLevel::Error, "Failed to bind udp server socket");
        return false;
    }

    return true;
}

bool NetworkBroadcast::createOutputSocket() {
    if ((m_inputSocketFd = ::socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        m_logger.log(LogLevel::Error, "Failed to create output socket for broadcast");
        return false;
    }

    // Required to enable broadcasting (
    if (::setsockopt(m_inputSocketFd, SOL_SOCKET, SO_BROADCAST, nullptr, 0) != 0) {
        m_logger.log(LogLevel::Error, "Failed to set output socket in broadcast mode");
        return false;
    }

    return true;
}

bool NetworkBroadcast::start() {
    if (createOutputSocket() && createInputSocket()) {
        m_logger.log(LogLevel::Info,
                     "Succesfully created input and output sockets for broadcasting");
        return true;
    }
    m_logger.log(LogLevel::Error, "Error while creating input and output sockets for broadcasting");
    return true;
}

bool NetworkBroadcast::stop() {
    if (m_inputSocketFd > 0) {
        ::close(m_inputSocketFd);
    }
    if (m_outputSocketFd > 0) {
        ::close(m_outputSocketFd);
    }
    return true;
}

bool NetworkBroadcast::send(const uint8_t* data, uint16_t length) { return false; }

bool NetworkBroadcast::receive(uint8_t* data, uint16_t length) { return false; }