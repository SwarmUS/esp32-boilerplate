#ifndef HIVE_CONNECT_SOCKETFACTORY_H
#define HIVE_CONNECT_SOCKETFACTORY_H

#include "TCPClient.h"
#include "TCPServer.h"

#define NO_SOCKET (-1)

namespace SocketFactory {

int createTCPServerSocket(uint16_t port);

int createTCPClient(const char* address, uint16_t port);

} // namespace SocketFactory

#endif // HIVE_CONNECT_SOCKETFACTORY_H
