#ifndef __NETWORKCONTAINER_H__
#define __NETWORKCONTAINER_H__

#include "INetworkInputStream.h"
#include "INetworkManager.h"
#include "INetworkOutputStream.h"
#include "logger/LoggerContainer.h"

namespace NetworkContainer {

/**
 * @return The network manager instance
 */
INetworkManager& getNetworkManager();

/**
 * @return The Network input stream instance
 */
INetworkInputStream& getNetworkInputStream();

/**
 * @return The Network output stream instance
 */
INetworkOutputStream& getNetworkOutputStream();
} // namespace NetworkContainer

#endif // __NETWORKCONTAINER_H__
