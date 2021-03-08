#ifndef HIVE_CONNECT_SOCKETFACTORY_H
#define HIVE_CONNECT_SOCKETFACTORY_H

#include "TCPClient.h"
#include "TCPServer.h"
#include <memory>
#include <optional>

#define NO_SOCKET (-1)

namespace SocketFactory {

int createTCPServerSocket(uint16_t port);

std::optional<TCPClient> createTCPClient(const char* address, uint16_t port);

} // namespace SocketFactory

#endif // HIVE_CONNECT_SOCKETFACTORY_H
