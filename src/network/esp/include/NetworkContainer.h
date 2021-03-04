#ifndef __NETWORKCONTAINER_H__
#define __NETWORKCONTAINER_H__

#include "NetworkManager.h"
#include "logger/LoggerContainer.h"

namespace NetworkContainer {

/**
 * @return The network manager instance
 */
NetworkManager& getNetworkManager();
} // namespace NetworkContainer

#endif // __NETWORKCONTAINER_H__