#include "SocketFactory.h"
#include "lwip/sockets.h"


int SocketFactory::createTCPServerSocket(uint16_t port) {
    sockaddr_in localAddress = {0};
    int socket = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (socket < 0) {
        // Failed to create socket
        return -1;
    }

    localAddress.sin_family = AF_INET;
    localAddress.sin_len = sizeof(localAddress);
    localAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddress.sin_port = htons(port);

    if (lwip_bind(socket, (sockaddr*)&localAddress, sizeof(localAddress)) < 0) {
        lwip_close(socket);
        return -1;
    }

    // Puts the socket in listening mode. Can only accept one connection at a time
    if (lwip_listen(socket, 1) != 0) {
        lwip_close(socket);
        return -1;
    }

    return socket;
}

std::optional<TCPClient> SocketFactory::createTCPClient(const char* address, uint16_t port) {
    return {};
}
