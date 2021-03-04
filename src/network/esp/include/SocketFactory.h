#ifndef HIVE_CONNECT_SOCKETFACTORY_H
#define HIVE_CONNECT_SOCKETFACTORY_H

#include "TCPClient.h"
#include "TCPServer.h"
#include <optional>
#include <memory>



namespace SocketFactory {

std::optional<TCPServer> createTCPServer(uint16_t port);

std::optional<TCPClient> createTCPClient(const char* address, uint16_t port);

} // namespace SocketFactory

#endif // HIVE_CONNECT_SOCKETFACTORY_H
