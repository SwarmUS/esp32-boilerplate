#ifndef __NETWORKCONTAINER_H__
#define __NETWORKCONTAINER_H__

#include "../src/common/include/AbstractNetworkManager.h"
#include "INetworkBroadcast.h"
#include "INetworkInputStream.h"
#include "INetworkOutputStream.h"
#include "logger/LoggerContainer.h"

namespace NetworkContainer {

/**
 * @return The network manager instance
 */
AbstractNetworkManager& getNetworkManager();

/**
 * @return The Network input stream instance
 */
INetworkInputStream& getNetworkInputStream();

/**
 * @return The Network output stream instance
 */
INetworkOutputStream& getNetworkOutputStream();

/**
 * @return The network broadcasting instance
 */
INetworkBroadcast& getNetworkBroadcast();
} // namespace NetworkContainer

#endif // __NETWORKCONTAINER_H__
