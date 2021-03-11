#ifndef HIVE_CONNECT_INETWORKMANAGER_H
#define HIVE_CONNECT_INETWORKMANAGER_H

#include <string>

enum class NetworkStatus { NotConnected = 0, Connecting, Connected };

class INetworkManager {
  public:
    /**
     * @brief Starts the network manager and attempts connection
     */
    virtual void start() = 0;

    /**
     * @brief Gets the status of the network
     */
    virtual NetworkStatus getNetworkStatus() = 0;

    /**
     * @brief Returns ID of the module in the network as a char array. In the wifi network, this
     * will be an IP address and on ROS probably a topic.
     * @return The networking ID for the module
     */
    virtual void getNetworkingID(std::string& id) = 0;
};

#endif // HIVE_CONNECT_INETWORKMANAGER_H
