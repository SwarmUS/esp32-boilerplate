#include "SocketFactory.h"
#include "logger/LoggerContainer.h"

#include "lwip/sockets.h"

std::optional<TCPServer> SocketFactory::createTCPServer(uint16_t port) {
    sockaddr_in localAddress = {0};
    int socket = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (socket < 0) {
        // Failed to create socket
        return {};
    }

    localAddress.sin_family = AF_INET;
    localAddress.sin_len = sizeof(localAddress);
    localAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddress.sin_port = port;

    if (lwip_bind(socket, (sockaddr*)&localAddress, sizeof(localAddress)) < 0) {
        lwip_close(socket);
        return {};
    }

    // Not sure what this does...
    if (lwip_listen(socket, 20) != 0) {
        lwip_close(socket);
        return {};
    }


    return TCPServer(socket, LoggerContainer::getLogger());
}

std::optional<TCPClient> SocketFactory::createTCPClient(const char* address, uint16_t port) {
    return {};
}
