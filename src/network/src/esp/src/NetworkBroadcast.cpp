#include "NetworkBroadcast.h"
#include "NetworkConfig.h"
#include "SocketFactory.h"
#include "lwip/sockets.h"

NetworkBroadcast::NetworkBroadcast(ILogger& logger) : m_logger(logger){}

NetworkBroadcast::~NetworkBroadcast()  { this->stop();}

bool NetworkBroadcast::start() {
    m_socket = SocketFactory::createUDPBroadcast(NetworkConfig::getBroadcastInputPort());
    if (m_socket < 0) {
        m_logger.log(LogLevel::Error, "Failed to obtain broadcast socket!");
        return false;
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

    return lwip_sendto(m_socket, data, length, 0, (sockaddr*)&broadcast, sizeof(broadcast)) == length;
}

bool NetworkBroadcast::receive(uint8_t* data, uint16_t length) {
    if (data == nullptr || m_socket < 0) {
        return false;
    }
    sockaddr_in broadcast;
    broadcast.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    broadcast.sin_family = AF_INET;
    broadcast.sin_port = htons(NetworkConfig::getBroadcastOutputPort());
    broadcast.sin_len = sizeof(broadcast);
    socklen_t socklen = sizeof(broadcast);

    ssize_t receivedBytes = 0;
    do {
        ssize_t received = lwip_recvfrom(m_socket, &data[receivedBytes], length-receivedBytes, 0,(sockaddr*)&broadcast,&socklen);
        if (received == -1) {
            break;
        }
        receivedBytes += received;

    } while (receivedBytes != length);

    return receivedBytes == length;
}

bool NetworkBroadcast::stop() {
    if (m_socket != -1) {
        lwip_close(m_socket);
    }
    m_socket = -1;
    return true;
}