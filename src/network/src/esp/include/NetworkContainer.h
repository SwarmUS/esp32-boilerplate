#ifndef __NETWORKCONTAINER_H__
#define __NETWORKCONTAINER_H__

#include "NetworkManager.h"
#include "TCPClient.h"
#include "TCPServer.h"
#include "logger/LoggerContainer.h"

namespace NetworkContainer {

/**
 * @return The network manager instance
 */
NetworkManager& getNetworkManager();

/**
 * @return The TCP server instance
 */
TCPServer& getTCPServer();

/**
 * @return The TCP client instance
 */
TCPClient& getTCPClient();
} // namespace NetworkContainer

#endif // __NETWORKCONTAINER_H__
