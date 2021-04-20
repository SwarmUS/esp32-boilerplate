#include "NetworkBroadcast.h"
#include "NetworkConfig.h"
#include "SocketFactory.h"
#include "Task.h"
#include "lwip/sockets.h"

static void task(void* context) {
    while (true) {
        static_cast<NetworkBroadcast*>(context)->receiveDatagrams();
        Task::delay(20);
    }
}

NetworkBroadcast::NetworkBroadcast(ILogger& logger) :
    m_logger(logger),
    m_socket(NO_SOCKET),
    m_started(false),
    m_receivingTask("udp_receive", tskIDLE_PRIORITY, task, this) {
    CircularBuff_init(&m_circularBuffer, m_data.data(), m_data.size());
    m_receivingTaskHandle = nullptr;
    m_receivingTask.start();
}

NetworkBroadcast::~NetworkBroadcast() { this->stop(); }

bool NetworkBroadcast::start() {
    if (!m_started) {
        m_socket = SocketFactory::createUDPBroadcast(NetworkConfig::getBroadcastInputPort());
        if (m_socket < 0) {
            m_logger.log(LogLevel::Error, "Failed to obtain broadcast socket!");
            return false;
        }
        m_started = true;
    }
    return true;
}

bool NetworkBroadcast::send(const uint8_t* data, uint16_t length) {
    if (data == nullptr || m_socket < 0) {
        return false;
    }

    // Destination
    sockaddr_in broadcast;
    broadcast.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    broadcast.sin_family = AF_INET;
    broadcast.sin_port = htons(NetworkConfig::getBroadcastOutputPort());
    broadcast.sin_len = sizeof(broadcast);

    return lwip_sendto(m_socket, data, length, 0, (sockaddr*)&broadcast, sizeof(broadcast)) ==
           length;
}

bool NetworkBroadcast::receive(uint8_t* data, uint16_t length) {
    if (data == nullptr || m_socket < 0) {
        return false;
    }

    m_receivingTaskHandle = xTaskGetCurrentTaskHandle();
    while (CircularBuff_getLength(&m_circularBuffer) < length) {
        ulTaskNotifyTake(pdTRUE, 500);
    }
    m_receivingTaskHandle = nullptr;

    return CircularBuff_get(&m_circularBuffer, data, length) == length;
}

bool NetworkBroadcast::stop() {
    if (m_socket != -1) {
        lwip_close(m_socket);
    }
    m_socket = NO_SOCKET;
    m_started = false;
    return true;
}

void NetworkBroadcast::receiveDatagrams() {
    if (m_started) {
        sockaddr_in broadcast;
        broadcast.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        broadcast.sin_family = AF_INET;
        broadcast.sin_port = htons(NetworkConfig::getBroadcastOutputPort());
        broadcast.sin_len = sizeof(broadcast);
        socklen_t socklen = sizeof(broadcast);

        // Receiving from a UDP socket de-queues the whole datagram from the receiving queue,
        // regardless of the size read. Here, we receive up to a maximum of the size of m_datagram.
        // The MSG_TRUNC flag will make the lwip_recvfrom return the size of the first datagram in
        // the queue, similar to a MSG_PEEK. We can know the length of the datagram from the size
        // returned.
        ssize_t sizeToReceive =
            lwip_recvfrom(m_socket, m_datagram.data(), m_datagram.size(), MSG_TRUNC | MSG_PEEK,
                          (sockaddr*)&broadcast, &socklen);

        // Receive the datagram
        if (lwip_recvfrom(m_socket, m_datagram.data(), sizeToReceive, 0, (sockaddr*)&broadcast,
                          &socklen) == -1) {
            m_logger.log(LogLevel::Error, "Failed to receive bytes from UDP socket");
            return;
        }

        if (CircularBuff_getFreeSize(&m_circularBuffer) >= sizeToReceive) {
            m_logger.log(LogLevel::Debug, "Received %d bytes from broadcast", sizeToReceive);
            CircularBuff_put(&m_circularBuffer, m_datagram.data(), sizeToReceive);
            if (m_receivingTaskHandle != nullptr) {
                xTaskNotifyGive(m_receivingTaskHandle);
            }
        }
    }
}