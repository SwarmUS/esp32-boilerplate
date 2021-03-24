#ifndef HIVE_CONNECT_DUMMYNETWORKMANAGER_H
#define HIVE_CONNECT_DUMMYNETWORKMANAGER_H

#include "INetworkManager.h"

class DummyNetworkManager : public INetworkManager {
  public:
    DummyNetworkManager() = default;
    ~DummyNetworkManager() = default;
    void start() override {}

    /**
     * @brief Gets the status of the network
     */
    NetworkStatus getNetworkStatus() override { return NetworkStatus::NotConnected; }

    /**
     * @brief Returns ID of the module in the network as a char array. In the wifi network, this
     * will be an IP address and on ROS probably a topic.
     * @return The networking ID for the module
     */
     bool getSelfIP(char *buffer, size_t maxLength) override {
        (void)buffer;
        (void)maxLength;
        return false;
    }

    bool getIPFromRobotID(uint32_t robotID, char *buffer, size_t maxLength) override {
        (void) robotID;
        (void) buffer;
        (void) maxLength;
        return false;
    }
};

#endif // HIVE_CONNECT_DUMMYNETWORKMANAGER_H
