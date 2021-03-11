#ifndef __NETWORKCONTAINER_H__
#define __NETWORKCONTAINER_H__

#include "INetworkDeserializer.h"
#include "INetworkManager.h"
#include "INetworkSerializer.h"
#include "logger/LoggerContainer.h"

namespace NetworkContainer {

/**
 * @return The network manager instance
 */
INetworkManager& getNetworkManager();

/**
 * @return The TCP server instance
 */
INetworkDeserializer& getTCPServer();

/**
 * @return The TCP client instance
 */
INetworkSerializer& getTCPClient();
} // namespace NetworkContainer

#endif // __NETWORKCONTAINER_H__
